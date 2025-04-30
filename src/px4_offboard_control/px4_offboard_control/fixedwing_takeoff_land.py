import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import math
from scipy.spatial.transform import Rotation as R

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import VehicleGlobalPosition, VehicleLocalPosition, SensorCombined
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleAttitude, VehicleRatesSetpoint
from px4_msgs.msg import ActuatorMotors, ActuatorServos

# sudo apt install ros-foxy-tf-transformations
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix

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
        self.rate_setpoint_publisher = self.create_publisher(
            VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.motor_setpoint_publisher = self.create_publisher(
            ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.servo_setpoint_publisher = self.create_publisher(
            ActuatorServos, '/fmu/in/actuator_servos', qos_profile)

        # Create subscribers
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)
        self.sensor_combined_subscriber = self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined', self.sensor_combined_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.state = 0
        self.offboard_setpoint_counter = 0
        self.vehicle_global_position = VehicleGlobalPosition()
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.sensor_combined = SensorCombined()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -100.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.dist = 0.0
        self.takeoff_lat = 0.0
        self.takeoff_lon = 0.0
        self.takeoff_alt = 0.0

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback function for vehicle_global_position topic subscriber."""
        self.vehicle_global_position = vehicle_global_position

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback function for vehicle_attitude topic subscriber."""
        self.vehicle_attitude = vehicle_attitude

    def sensor_combined_callback(self, sensor_combined):
        """Callback function for sensor_combined topic subscriber."""
        self.sensor_combined = sensor_combined

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

    def set_home(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_HOME, param1=1.0)
        self.get_logger().info("Set current location as home position")

    def set_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def takeoff(self):
        """Switch to takeoff mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            # param1=-1.0,     # Minimum pitch (if airspeed sensor present), desired pitch without sensor
            # param2=0.0,     # Empty
            # param3=0.0,     # Empty
            # param4=np.nan(),     # Desired Yaw angle (if magnetometer present), ignored without magnetometer
            # param5=np.nan(),     # Latitude
            # param6=np.nan(),     # Longitude
            param7=self.vehicle_global_position.alt + 100.0)   # Altitude
        self.get_logger().info("Switching to takeoff mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
            # ,
            # param1=0.0,   # Empty
            # param2=0.0,   # Empty
            # param3=0.0,   # Empty
            # param4=0.0,   # Desired Yaw angle
            # param5=0.0,   # Latitude
            # param6=0.0,   # Longitude
            # param7=0.0    # Altitude
            )
        self.get_logger().info("Switching to land mode")

    def do_global_loiter(self, lat, lon, alt_msl, radius_and_direction, speed):
        """ Loitering specific global point """
        self.publish_heartbeat_ob_pos_sp()
        # VEHICLE_CMD_NAV_LOITER_UNLIM
        # Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
        # | Desired yaw angle.| Latitude| Longitude| Altitude|

        # VEHICLE_CMD_DO_ORBIT
        # |Radius [m] |Velocity [m/s] |Yaw behaviour |Empty |Latitude/X |Longitude/Y |Altitude/Z |
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_ORBIT,
            param1=float(radius_and_direction),     # Radius
            param2=float(speed),    # Velocity
            param3=float(0.0),      # Yaw Behavior
            # param4=np.nan(),      # Desired Yaw angle
            param5=float(lat),      # Latitude
            param6=float(lon),      # Longitude
            param7=float(alt_msl))  # Altitude

    def do_global_reposition(self, lat, lon, alt_msl):
        """ Loitering specific global point """
        # Reposition to specific WGS84 GPS position.
        # |Ground speed [m/s] |Bitmask |Loiter radius [m] for planes |Yaw    [deg] |Latitude    |Longitude |Altitude |
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
            # param1=-1.0,     # Ground speed [m/s]
            # param2=0.0,      # Bitmask
            # param3 = radius_and_clockwise,      # Loiter radius [m] for planes
            # param4=np.nan(),     # Yaw    [deg]
            param5=float(lat),      # Latitude
            param6=float(lon),      # Longitude
            param7=float(alt_msl))  # Altitude

    def publish_heartbeat_ob(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

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

    def publish_heartbeat_ob_rate_sp(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_heartbeat_ob_actuator_sp(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position    = False
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.thrust_and_torque   = False
        msg.direct_actuator     = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw_d=0.0):
        """Publish the trajectory setpoint."""
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.set_offboard_mode()
        self.publish_heartbeat_ob_pos_sp()
        msg = TrajectorySetpoint()
        msg.position    = [float(x), float(y), float(z)]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw         = np.max([-np.pi, np.min([np.deg2rad(yaw_d), np.pi])])
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_velocity_setpoint(self, x, y, z, yaw_d=0.0):
        """Publish the trajectory setpoint."""
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.set_offboard_mode()
        self.publish_heartbeat_ob_vel_sp()
        msg = TrajectorySetpoint()
        msg.position    = [np.nan, np.nan, np.nan]
        msg.velocity    = [float(x), float(y), float(z)]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw         = np.max([-np.pi, np.min([np.deg2rad(yaw_d), np.pi])])
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_acceleration_setpoint(self, x, y, z, yaw_d=0.0):
        """Publish the trajectory setpoint."""
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.set_offboard_mode()
        self.publish_heartbeat_ob_acc_sp()
        msg = TrajectorySetpoint()
        msg.position    = [np.nan, np.nan, np.nan]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [float(x), float(y), float(z)]
        msg.yaw         = np.max([-np.pi, np.min([np.deg2rad(yaw_d), np.pi])])
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_fw_attitude_setpoint(self, roll_d, pitch_d, thr):
        """
            Publish the attitude and thrust setpoint.
            roll_d [deg]
            pitch_d [deg]
            thr [-1 ~ 1]
        """
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.set_offboard_mode()
        self.publish_heartbeat_ob_att_sp()
        roll  = np.deg2rad(roll_d)
        pitch = np.deg2rad(pitch_d)
        msg = VehicleAttitudeSetpoint()
        msg.roll_body  = float(roll)
        msg.pitch_body = float(pitch)
        msg.yaw_body   = float(0.0)
        msg.thrust_body[0] = float(thr) # For Fixed-wing
        # msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = 0.0
        # msg.thrust_body[2] = -thr # For Multi-rotor
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.attitude_setpoint_publisher.publish(msg)

    def publish_fw_angular_rate_setpoint(self, rollrate_d, pitchrate_d, yawrate_d, thr):
        """
            Publish the attitude and thrust setpoint.
            rollrate_d  [deg/s]
            pitchrate_d [deg/s]
            yawrate_d   [deg/s]
            thr [-1 ~ 1]
        """
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.set_offboard_mode()
        self.publish_heartbeat_ob_rate_sp()
        rollrate  = np.deg2rad(rollrate_d)
        pitchrate = np.deg2rad(pitchrate_d)
        yawrate   = np.deg2rad(yawrate_d)
        msg = VehicleRatesSetpoint()
        msg.roll    = float(rollrate)  # [rad/s]
        msg.pitch   = float(pitchrate) # [rad/s]
        msg.yaw     = float(yawrate)   # [rad/s]
        msg.thrust_body[0] = float(thr) # For Fixed-wing
        # msg.thrust_body[0] = 0.0
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = 0.0
        # msg.thrust_body[2] = -thr # For Multi-rotor
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.rate_setpoint_publisher.publish(msg)

    # It doesn't work
    def publish_actuator_setpoint(self, ail, elev, thr, rud):
        """Publish direct actuator command."""
        # Direct servo control
        msg_servo = ActuatorServos()
        msg_servo.control[0] = float(ail)
        msg_servo.control[1] = float(-ail)
        msg_servo.control[2] = float(elev)
        msg_servo.control[3] = float(rud)
        msg_servo.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.servo_setpoint_publisher.publish(msg_servo)

        # Direct motor control
        msg_motor = ActuatorMotors()
        msg_motor.control[0] = float(thr)
        msg_motor.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.motor_setpoint_publisher.publish(msg_motor)

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
        return np.linalg.norm([dx,dy])

    def cal_rotation(self):
        self.vehicle_euler = euler_from_quaternion(self.vehicle_attitude.q)
        roll  = self.vehicle_euler[0]
        pitch = self.vehicle_euler[1]
        yaw   = self.vehicle_euler[2]
        self.cn2b = R.from_euler('zyx', [yaw, pitch, roll], degrees=False)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.cal_rotation()
        self._roll_d  = np.rad2deg(self.vehicle_euler[2])
        self._pitch_d =-np.rad2deg(self.vehicle_euler[1])
        self._yaw_d   = np.rad2deg(self.vehicle_euler[0])
        print("S{:d} T {:.2f}, ".format(self.state, (self.get_clock().now().nanoseconds/1000000000)%1000.0), end=' ')
        # print("Pxyz  {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z), end=' ')
        # print("Vxyz  {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.vx, self.vehicle_local_position.vy, self.vehicle_local_position.vz), end=' ')
        # print("Axyz  {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.ax, self.vehicle_local_position.ay, self.vehicle_local_position.az), end=' ')
        # print("Euler {:6.2f}, {:6.2f}, {:6.2f}".format(self._roll_d, self._pitch_d, self._yaw_d), end=' ')

        # Take-off
        if(self.state == 0):
            self.publish_heartbeat_ob_pos_sp()
            self.offboard_setpoint_counter %= 20
            if self.offboard_setpoint_counter < 20:
                self.offboard_setpoint_counter += 1
            # self.set_home()

            # Set Auto_takeoff mode, and arming
            if(self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF and
               self.offboard_setpoint_counter == 5):
                    self.takeoff()
            if (self.vehicle_local_position.z > -5.0 and
                self.offboard_setpoint_counter == 15):
                self.arm()

            print(" 0, 0, {:.1f}, Takeoff".format(-self.takeoff_height))

            if( self.vehicle_local_position.z <= -10.0):
                self.state = self.state + 1

        # After take-off, set mode as Offboard
        elif(self.state == 1):
            self.publish_heartbeat_ob_pos_sp()
            self.pos_x = 0.0
            self.pos_y = 0.0
            self.pos_z = self.takeoff_height
            self.pos_yaw = 0
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)

            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(self.vehicle_local_position.x,
                                                          self.vehicle_local_position.y,
                                                          self.vehicle_local_position.z), end=' ')
            print(" / Set Offboard ")
            self.takeoff_lat = self.vehicle_global_position.lat
            self.takeoff_lon = self.vehicle_global_position.lon
            self.takeoff_alt = 100.0

            self.set_offboard_mode()
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        # Go to waiting height
        elif(self.state == 2):
            target_alt = 100.0
            self.do_global_loiter(  self.takeoff_lat,
                                    self.takeoff_lon+0.001,
                                    self.takeoff_alt, # Alt
                                    100.0, # Radius
                                    20.0) # Speed
            print("LLA {:10.6f}, {:10.6f}, {:6.2f}".format( self.vehicle_global_position.lat,
                                                            self.vehicle_global_position.lon,
                                                            self.vehicle_global_position.alt), end=' ')
            print(" / Approach Loiter Alt {} m".format(target_alt))

            if(np.abs(self.vehicle_global_position.alt - target_alt) < 10):
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 3): # First WP
            self.pos_x = 200.0
            self.pos_y = 0.0
            self.pos_z = self.takeoff_height - 20.0
            self.pos_yaw = 0
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            print("Pxyz ", end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self.vehicle_local_position.x, self.pos_x), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self.vehicle_local_position.y, self.pos_y), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self.vehicle_local_position.z, self.pos_z), end=' ')
            print(" / First WP")

            self.dist = self.get_distance()
            if self.dist < 10.0:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 4): # Second WP
            self.pos_x = 0.0
            self.pos_y = 0.0
            self.pos_z = self.takeoff_height
            self.pos_yaw = 0
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            print("Pxyz ", end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self.vehicle_local_position.x, self.pos_x), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self.vehicle_local_position.y, self.pos_y), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self.vehicle_local_position.z, self.pos_z), end=' ')
            print(" / Second WP")

            self.dist = self.get_distance()
            if self.dist < 10.0:
                self.state = self.state + 1
                self.offboard_setpoint_counter = 0

        elif(self.state == 5):  # Attitude Control
            self.publish_fw_attitude_setpoint(30.0, 10.0, 0.50) # Roll/Pitch/Thr
            self.offboard_setpoint_counter += 1
            print("Att  ", end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self._roll_d    , 30.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self._pitch_d   , 10.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self._yaw_d     , 0.0), end=' ')
            print("")
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 80:
                self.offboard_setpoint_counter = 0
                self.state = self.state + 1

        elif(self.state == 6): # Attitude Control
            self.publish_fw_attitude_setpoint(-30.0, -10.0, 0.50) # Roll/Pitch/Thr
            print("Att  ", end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self._roll_d    , -30.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self._pitch_d   , -10.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(self._yaw_d     , 0.0), end=' ')
            print("")
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 80:
                self.offboard_setpoint_counter = 0
                self.state = self.state + 1

        elif(self.state == 7):  # Rate Control
            self.publish_fw_angular_rate_setpoint(10.0, 5.0, 0.0, 0.35) # Roll/Pitch/Yaw
            print("Rate ", end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(np.rad2deg(self.sensor_combined.gyro_rad[0]), 10.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(np.rad2deg(self.sensor_combined.gyro_rad[0]), 5.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(np.rad2deg(self.sensor_combined.gyro_rad[0]), 0.0), end=' ')
            print("")
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 50:
                self.offboard_setpoint_counter = 0
                self.state = self.state + 1

        elif(self.state == 8): # Rate Control
            self.publish_fw_angular_rate_setpoint(-10.0, -5.0, 0.0, 0.35) # Roll/Pitch/Yaw
            print("Rate ", end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(np.rad2deg(self.sensor_combined.gyro_rad[0]), -10.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(np.rad2deg(self.sensor_combined.gyro_rad[0]), -5.0), end=' ')
            print("{:6.2f} [{:6.2f}] / ".format(np.rad2deg(self.sensor_combined.gyro_rad[0]), 0.0), end=' ')
            print("")
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 50:
                self.offboard_setpoint_counter = 0
                self.state = self.state + 1

        # Loitering
        elif(self.state == 9):
            self.do_global_loiter( self.takeoff_lat, self.takeoff_lon, self.takeoff_alt,
                                -100.0, # Radius
                                20.0) # Speed
            print("LLA {:10.6f}, {:10.6f}, {:6.2f}".format( self.vehicle_global_position.lat,
                                                            self.vehicle_global_position.lon,
                                                            self.vehicle_global_position.alt), end=' ')
            print(" / Loiter1 {}".format(self.offboard_setpoint_counter))
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 300:
                self.offboard_setpoint_counter = 0
                self.state = self.state + 1
 
        # Loitering
        elif(self.state == 10):
            self.do_global_loiter( self.takeoff_lat, self.takeoff_lon, self.takeoff_alt + 20.0, # Alt
                                150.0, # Radius
                                15.0) # Speed
            print("LLA {:10.6f}, {:10.6f}, {:6.2f}".format( self.vehicle_global_position.lat,
                                                            self.vehicle_global_position.lon,
                                                            self.vehicle_global_position.alt), end=' ')
            print(" / Loiter2 {}".format(self.offboard_setpoint_counter))
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 300:
                self.offboard_setpoint_counter = 0
                self.state = self.state + 1

        # # Reposition
        elif(self.state == 11):
            self.publish_heartbeat_ob_pos_sp()
            self.do_global_reposition(self.takeoff_lat + 0.001,
                                    self.takeoff_lon,
                                       self.takeoff_alt) # Alt
            print("LLA {:10.6f}, {:10.6f}, {:6.2f}".format( self.vehicle_global_position.lat,
                                                            self.vehicle_global_position.lon,
                                                            self.vehicle_global_position.alt), end=' ')
            print(" / Reposition")
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 300:
                self.offboard_setpoint_counter = 0
                self.state = self.state + 1

        # # Landing
        elif(self.state == 12):
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
