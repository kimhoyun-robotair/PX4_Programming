"""
이번에는 WP를 2개 정도 지정해서 다녀보자.
1번째 WP는 x로 20, y로 30, z로 -5m이고
2번째 WP는 x로 10, y 로 5, z로 -15m이고
3번째 WP는 처음 시작 위치, x로 0, y로 0, z로 -10m이다.
그 다음 착륙까지 진행한다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

np.float = float
from tf_transformations import euler_from_quaternion

class OffboardControl(Node):
    def __init__(self):
        super().__init__("Offboard_control")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # create publisher
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_profile)

        self.state = "" # 각 WP 및 이착륙 판단
        self.offboard_setpoint_counter = 0
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.runtime = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.dist = 0.0
        self.wp1 = [20, 10, -5]
        self.wp2 = [10, 40, -5]
        self.wp3 = [0, 0, -3]
        self.current_wp = [self.pos_x, self.pos_y, self.pos_z]

        self.timer = self.create_timer(0.01, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_attitude topic subscriber."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_heartbeat_ob_pos_sp(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position    = True
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw_d:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [float(x), float(y), float(z)]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw = np.max([-np.pi, np.min([np.deg2rad(yaw_d), np.pi])])
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def get_distance(self):
        dx = self.vehicle_local_position.x - self.pos_x
        dy = self.vehicle_local_position.y - self.pos_y
        dz = self.vehicle_local_position.z - self.pos_z
        return np.linalg.norm([dx,dy,dz])

    def get_attitude(self, current_wp, next_wp):
        dx = next_wp[0] - current_wp[0]
        dy = next_wp[1] - current_wp[1]

        # arctan2는 (dy, dx)를 인자로 받아 라디안 단위의 각도를 반환.
        yaw_rad = math.atan2(dy, dx)

        # 라디안을 도 단위로 변환
        yaw_deg = math.degrees(yaw_rad)
        return yaw_deg

    def timer_callback(self):
        self.vehicle_euler = euler_from_quaternion(self.vehicle_attitude.q)
        self._roll_d  = np.rad2deg(self.vehicle_euler[2])
        self._pitch_d = np.rad2deg(self.vehicle_euler[1])
        self._yaw_d   = np.rad2deg(self.vehicle_euler[0])
        print("S{} Time {:.2f}, ".format(self.state, (self.get_clock().now().nanoseconds/1000000000)%1000.0), end=' ')

        if(self.state == ""):
            self.publish_heartbeat_ob_pos_sp()
            if(self.offboard_setpoint_counter < 10):
                self.offboard_setpoint_counter += 1

            self.offboard_setpoint_counter %= 11
            self.runtime += 1

            if self.offboard_setpoint_counter < 5:
                self.pos_x = 0.0
                self.pos_y = 0.0
                self.pos_z = self.takeoff_height
                self.pos_yaw = np.rad2deg(self.vehicle_euler[0])
                self.current_wp = [self.pos_x, self.pos_y, self.pos_z]
                self.engage_offboard_mode()

            if self.offboard_setpoint_counter == 9:
                self.arm()

            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.dist = self.get_distance()
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if(self.runtime > 1000):
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.current_wp, self.wp1)
            self.pos_x = self.wp1[0]
            self.pos_y = self.wp1[1]
            self.pos_z = self.wp1[2]
            self.current_wp = [self.pos_x, self.pos_y, self.pos_z]
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.dist = self.get_distance()
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if(self.dist < 1):
                self.state = "WP1"

        elif self.state == "WP1":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.current_wp, self.wp2)
            self.pos_x = self.wp2[0]
            self.pos_y = self.wp2[1]
            self.pos_z = self.wp2[2]
            self.current_wp = [self.pos_x, self.pos_y, self.pos_z]
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.dist = self.get_distance()
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if(self.dist < 1):
                self.state = "WP2"

        elif self.state == "WP2":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.current_wp, self.wp3)
            self.pos_x = self.wp3[0]
            self.pos_y = self.wp3[1]
            self.pos_z = self.wp3[2]
            self.current_wp = [self.pos_x, self.pos_y, self.pos_z]
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.dist = self.get_distance()
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if(self.dist < 1):
                self.state = "WP3"

        elif self.state == "WP3":
            self.publish_heartbeat_ob_pos_sp()
            self.land()

def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
