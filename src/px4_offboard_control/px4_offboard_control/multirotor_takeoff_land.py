"""
본 코드에서는,
1. arm
2. 이륙고도 20m로 이륙
3. 이륙이 판정되면 다시 착륙
하는 코드로 구성되어 있다.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleRatesSetpoint
np.float = float
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class OffboardControl(Node):
    def __init__(self):
        super().__init__("Offboard_control")

        # qos profile
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # create pubblisher
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

        self.state = 0
        self.offboard_setpoint_counter = 0
        self.runtime = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()

        self.takeoff_height = -5.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.dist = 0.0

        self.timer = self.create_timer(0.01, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
    def vehicle_attitude_callback(self, vehicle_attitude):
        self.vehicle_attitude = vehicle_attitude
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, params=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command( # offboard mode로 스위칭 하겠다는 뜻
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_heartbeat_ob_pos_sp(self):
        # 위치 기반 offboard control
        msg = OffboardControlMode()
        msg.position = True
        msg.position    = True
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw_d: float):
        # 위치 기반 제어를 위한 위치 경로점 송출
        msg = TrajectorySetpoint()
        msg.position    = [float(x), float(y), float(z)]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw = np.max([-np.pi, np.min([np.deg2rad(yaw_d), np.pi])])
        # yaw 값을 rad로 변환하고, -pi부터 pi 사이로 clamping
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        # VehicleCommand 보내기
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
        return np.linalg.norm([dx, dy, dz])

    def timer_callback(self):
        self.vehicle_euler = euler_from_quaternion(self.vehicle_attitude.q)
        # 쿼터니언 to 오일러리안
        self._roll_d  = np.rad2deg(self.vehicle_euler[2])
        self._pitch_d = np.rad2deg(self.vehicle_euler[1])
        self._yaw_d   = np.rad2deg(self.vehicle_euler[0])
        print("S{:d} Time {:.2f}, ".format(self.state, (self.get_clock().now().nanoseconds/1000000000)%1000.0), end=' ')
        # print("Pxyz  {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z), end=' ')

        # Take-off
        if(self.state == 0):
            self.publish_heartbeat_ob_pos_sp()
            if(self.offboard_setpoint_counter < 10):
                self.offboard_setpoint_counter += 1
            self.offboard_setpoint_counter %= 11
            if self.offboard_setpoint_counter < 5:
                self.pos_x = 0.0
                self.pos_y = 0.0
                self.pos_z = self.takeoff_height
                self.pos_yaw = np.rad2deg(self.vehicle_euler[0])
                self.engage_offboard_mode()
            if self.offboard_setpoint_counter == 9:
                self.arm()
            self.runtime += 1
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.dist = self.get_distance()
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if self.runtime >= 1500:
                self.state = 1

        elif self.state == 1:
            self.publish_heartbeat_ob_pos_sp()
            self.land()
            exit(0)

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
