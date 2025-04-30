import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleRatesSetpoint

# sudo apt install ros-foxy-tf-transformations
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
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
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.state = 0
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -20.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.dist = 0.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_local_position topic subscriber."""
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

    def publish_heartbeat_ob_vel_sp(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = True
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_heartbeat_ob_acc_sp(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = False
        msg.acceleration= True
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_heartbeat_ob_att_sp(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = True
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

    def publish_velocity_setpoint(self, x, y, z, yaw_d:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [np.nan, np.nan, np.nan]
        msg.velocity    = [float(x), float(y), float(z)]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw = np.max([-np.pi, np.min([np.deg2rad(yaw_d), np.pi])])
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_acceleration_setpoint(self, x, y, z, yaw_d:float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [np.nan, np.nan, np.nan]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [float(x), float(y), float(z)]
        msg.yaw = np.max([-np.pi, np.min([np.deg2rad(yaw_d), np.pi])])
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_attitude_setpoint(self, roll_d, pitch_d, yaw_d, thr):
        """
            Publish the attitude and thrust setpoint.
            roll_d [deg]
            pitch_d [deg]
            yaw_d [deg]
            thr [-1 ~ 1]
        """
        roll  = np.deg2rad(roll_d)
        pitch = np.deg2rad(pitch_d)
        yaw   = np.deg2rad(yaw_d)
        msg = VehicleAttitudeSetpoint()
        # msg.roll_body = roll
        # msg.pitch_body = pitch
        # msg.yaw_body = yaw
        msg.q_d = quaternion_from_euler(yaw, pitch, roll)
        msg.thrust_body[2] = -thr # For Multi-rotor
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.attitude_setpoint_publisher.publish(msg)

    def publish_angular_rate_setpoint(self, rollrate_d, pitchrate_d, yawrate_d, thr):
        """
            Publish the attitude and thrust setpoint.
            rollrate_d  [deg/s]
            pitchrate_d [deg/s]
            yawrate_d   [deg/s]
            thr [-1 ~ 1]
        """
        rollrate  = np.deg2rad(rollrate_d)
        pitchrate = np.deg2rad(pitchrate_d)
        yawrate   = np.deg2rad(yawrate_d)
        msg = VehicleRatesSetpoint()
        msg.roll    = rollrate  # [rad/s]
        msg.pitch   = pitchrate # [rad/s]
        msg.yaw     = yawrate   # [rad/s]
        msg.thrush[2] = thr # For Multi-rotor
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.attitude_setpoint_publisher.publish(msg)

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

    def timer_callback(self) -> None:
        """Callback function for the timer."""

        self.vehicle_euler = euler_from_quaternion(self.vehicle_attitude.q)
        self._roll_d  = np.rad2deg(self.vehicle_euler[2])
        self._pitch_d = np.rad2deg(self.vehicle_euler[1])
        self._yaw_d   = np.rad2deg(self.vehicle_euler[0])
        print("S{:d} Time {:.2f}, ".format(self.state, (self.get_clock().now().nanoseconds/1000000000)%1000.0), end=' ')
        # print("Pxyz  {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z), end=' ')
        # print("Vxyz  {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.vx, self.vehicle_local_position.vy, self.vehicle_local_position.vz), end=' ')
        # print("Axyz  {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.ax, self.vehicle_local_position.ay, self.vehicle_local_position.az), end=' ')
        # print("Euler {:6.2f}, {:6.2f}, {:6.2f}".format(self._roll_d, self._pitch_d, self._yaw_d), end=' ')

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

            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.dist = self.get_distance()
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if(self.vehicle_local_position.z <= self.takeoff_height + 1 and
               self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.state = 1

        # First WP
        elif(self.state == 1):
            self.publish_heartbeat_ob_pos_sp()
            self.pos_x = 0.0
            self.pos_y = 0.0
            self.pos_z = self.takeoff_height
            self.pos_yaw = 0
            self.dist = self.get_distance()
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if(self.dist < 1):
                self.state = self.state + 1

        elif(self.state == 2): # velocity
            self.publish_heartbeat_ob_vel_sp()
            self.pos_x = 1.0
            self.pos_y = 1.0
            self.pos_z = 0.1
            self.pos_yaw = 180.0
            self.publish_velocity_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.offboard_setpoint_counter += 1
            print("Vxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.vx,
                                                          self.vehicle_local_position.vy,
                                                          self.vehicle_local_position.vz), end=' ')
            print(" / Vxyz = 1.0, 1.0, 0.1")
            if self.offboard_setpoint_counter == 50:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 3): # velocity
            self.publish_heartbeat_ob_vel_sp()
            self.pos_x = -1.0
            self.pos_y = -1.0
            self.pos_z = -0.1
            self.pos_yaw = -180.0
            self.publish_velocity_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.offboard_setpoint_counter += 1
            print("Vxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.vx,
                                                          self.vehicle_local_position.vy,
                                                          self.vehicle_local_position.vz), end=' ')
            print(" / Vxyz = -1.0, -1.0, -0.1")
            if self.offboard_setpoint_counter == 50:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 4): # acc
            self.publish_heartbeat_ob_acc_sp()
            self.publish_acceleration_setpoint(0.0, 0.0, 0.0, 0.0)
            self.offboard_setpoint_counter += 1
            print("Axyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.ax,
                                                          self.vehicle_local_position.ay,
                                                          self.vehicle_local_position.az), end=' ')
            print(" / Acc = 0.0, 0.0, 0.0")
            if self.offboard_setpoint_counter == 20:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 5): # acc
            self.publish_heartbeat_ob_acc_sp()
            self.publish_acceleration_setpoint(0.1, 0.1, 0.1, np.deg2rad(10.0))
            self.offboard_setpoint_counter += 1
            print("Axyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.ax,
                                                          self.vehicle_local_position.ay,
                                                          self.vehicle_local_position.az), end=' ')
            print(" / Acc = 0.1, 0.1, 0.1")
            if self.offboard_setpoint_counter == 20:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 6): # att
            self.publish_heartbeat_ob_att_sp()
            self.publish_attitude_setpoint(5.0, 0.0, 0.0, 0.73) # Roll/Pitch/Yaw
            self.offboard_setpoint_counter += 1
            print("Att {:6.2f}, {:6.2f}, {:6.2f} [deg]".format(self._roll_d ,
                                                               self._pitch_d,
                                                               self._yaw_d  ), end=' ')
            print(" / Att = 5.0, 0.0, 0.0")
            if self.offboard_setpoint_counter == 50:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 7): # att
            self.publish_heartbeat_ob_att_sp()
            self.publish_attitude_setpoint(0.0, 5.0, 0.0, 0.73) # Roll/Pitch/Yaw
            self.offboard_setpoint_counter += 1
            print("Att {:6.2f}, {:6.2f}, {:6.2f} [deg]".format(self._roll_d ,
                                                               self._pitch_d,
                                                               self._yaw_d  ), end=' ')
            print(" / Att = 0.0, 5.0, 0.0")
            if self.offboard_setpoint_counter == 50:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 8): # att
            self.publish_heartbeat_ob_att_sp()
            self.publish_attitude_setpoint(0.0, 0.0, 10.0, 0.71) # Roll/Pitch/Yaw
            self.offboard_setpoint_counter += 1
            print("Att {:6.2f}, {:6.2f}, {:6.2f} [deg]".format(self._roll_d ,
                                                               self._pitch_d,
                                                               self._yaw_d  ), end=' ')
            print(" / Att = 0.0, 0.0, 10.0")
            if self.offboard_setpoint_counter == 50:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 9): # Pos
            self.publish_heartbeat_ob_pos_sp()
            self.pos_x = 0.0
            self.pos_y = 10.0
            self.pos_z = self.takeoff_height
            self.pos_yaw = np.rad2deg(self.vehicle_euler[0])
            self.dist = self.get_distance()
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            if(self.dist < 1):
                self.offboard_setpoint_counter += 1
                if self.offboard_setpoint_counter == 50:
                    self.state = self.state + 1
                    self.offboard_setpoint_counter = 0

        # Landing
        elif(self.state == 10):
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
