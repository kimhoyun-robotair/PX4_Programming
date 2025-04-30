###############################################################
# WGS84 좌표 기반으로 장거리 멀티로터 자율비행 (~5km)을 구현하는 코드 #
###############################################################
import rclpy
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleCommand, VehicleStatus
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleGlobalPosition

# from tf_transformations import euler_from_quaternion, quaternion_from_euler

EARTH_RADIUS = 6371000.0  # 지구 반경 (미터)

class OffboardControl(Node):
    def __init__(self):
        """ Initialize Node """
        """ Setting QoS Profile and Topic Pub / Sub """
        super().__init__("Offboard_control")

        # PX4와의 통신을 위한 QoS 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # config 파일 활용을 위한 파라미터 선언 및 디폴트 값 지정
        self.declare_parameter("topic_offboard_control_mode", "/fmu/in/offboard_control_mode")
        self.declare_parameter("topic_vehicle_attitude_setpoint", "/fmu/in/vehicle_attitude_setpoint")
        self.declare_parameter("topic_trajectory_setpoint", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("topic_vehicle_command", "/fmu/in/vehicle_command")
        self.declare_parameter("topic_vehicle_local_position", "/fmu/out/vehicle_local_position")
        self.declare_parameter("topic_vehicle_attitude", "/fmu/out/vehicle_attitude")
        self.declare_parameter("topic_vehicle_status", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("topic_vehicle_global_position", "/fmu/out/vehicle_global_position")

        # 파라미터 파일을 읽어오기
        topic_offboard_control_mode = self.get_parameter("topic_offboard_control_mode").value
        topic_vehicle_attitude_setpoint = self.get_parameter("topic_vehicle_attitude_setpoint").value
        topic_trajectory_setpoint = self.get_parameter("topic_trajectory_setpoint").value
        topic_vehicle_command = self.get_parameter("topic_vehicle_command").value
        topic_vehicle_local_position = self.get_parameter("topic_vehicle_local_position").value
        topic_vehicle_attitude = self.get_parameter("topic_vehicle_attitude").value
        topic_vehicle_status = self.get_parameter("topic_vehicle_status").value
        topic_vehicle_global_position = self.get_parameter("topic_vehicle_global_position").value

        # Topic Publisher
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, topic_offboard_control_mode, qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(
            VehicleAttitudeSetpoint, topic_vehicle_attitude_setpoint, qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, topic_trajectory_setpoint, qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, topic_vehicle_command, qos_profile)

        # Topic Subscriber
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, topic_vehicle_local_position, self.vehicle_local_position_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, topic_vehicle_attitude, self.vehicle_attitude_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, topic_vehicle_status, self.vehicle_status_callback, qos_profile)
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, topic_vehicle_global_position, self.vehicle_global_position_callback, qos_profile)

        # 좌표로 주어지는 WGS84 경로점들
        self.wgs84_waypoints = {
            "WP0": {"lat": 37.449187, "lon": 126.653021, "alt": -0.0},
            "WP1": {"lat": 37.449187, "lon": 126.653021, "alt": -20.0},
            "WP2": {"lat": 37.452717, "lon": 126.653195, "alt": -20.0},
            "WP3": {"lat": 37.454559, "lon": 126.657573, "alt": -20.0},
            "WP4": {"lat": 37.454490, "lon": 126.648858, "alt": -20.0},
            "WP5": {"lat": 37.452717, "lon": 126.653195, "alt": -20.0},
            "WP6": {"lat": 37.452059, "lon": 126.647888, "alt": -20.0},
            "WP7": {"lat": 37.452059, "lon": 126.647888, "alt": -5.0},
            "WP8": {"lat": 37.452059, "lon": 126.647888, "alt": -20.0},
            "WP9": {"lat": 37.451786, "lon": 126.659597, "alt": -20.0},
            "WP10": {"lat": 37.452717, "lon": 126.653195, "alt": -10.0},
            "WP11": {"lat": 37.449187, "lon": 126.653021, "alt": -10.0}
        }

        # WGS84에서 NED로 변환된 Local 경로점들
        self.ned_waypoints = {}

        self.pre_calculate_waypoint_and_initialize_variable()
        self.get_logger().info("Initialization complete.")

####################### Initialize and Coordinate Conversion Functions #######################
    def pre_calculate_waypoint_and_initialize_variable(self):
        """ Pre-calculate waypoints and ready vehicle """
        """ Calculate WGS84 to NED Coordinate for using in px4-msgs"""
        """ And Initialize various variables, Setting Timer Callback"""
        # 경로기준점 초기화
        self.ref_lat = math.radians(self.wgs84_waypoints["WP0"]["lat"])
        self.ref_lon = math.radians(self.wgs84_waypoints["WP0"]["lon"])
        self.ref_sin_lat = math.sin(self.ref_lat)
        self.ref_cos_lat = math.cos(self.ref_lat)
        self.ref_init_done = True

        if self.ref_init_done:
            self.get_logger().info("Reference point initialized successfully.")

        # WGS84 좌표를 NED 좌표로 변환한 후에 저장
        for wp_name, wp_data in self.wgs84_waypoints.items():
            lat = wp_data["lat"]
            lon = wp_data["lon"]
            alt = wp_data["alt"]
            x, y = self.wgs84_to_ned(lat, lon)
            self.ned_waypoints[wp_name] = {"x": x, "y": y, "z": alt}
        self.get_logger().info(f"Pre-calculated waypoints in NED coordinates:\n{self.ned_waypoints}")

        # 각종 변수들 초기화 및 선언
        self.state = "NOT_READY"
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.vehicle_status = VehicleStatus()
        self.vehicle_global_position = VehicleGlobalPosition()

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.pos_yaw = 0.0
        self.global_dist = 0.0
        self.takeoff_height = self.wgs84_waypoints["WP1"]["alt"]
        self.waypoint_reach_or_not = False

        self.transition_to_fc = False
        self.transition_to_mc = False

        self.declare_parameter("threshold_range", 3.0)
        self.threshold_range = self.get_parameter("threshold_range").value

        # timer 콜백 함수 설정   
        self.timer = self.create_timer(0.01, self.timer_callback)

    def wgs84_to_ned(self, lat, lon):
        """
        주어진 위도와 경도(degrees)를 기준점에 대해 Azimuthal Equidistant Projection 방식으로 투영하여
        로컬 좌표 (x, y) 를 미터 단위로 계산
        """
        # 입력 좌표를 라디안으로 변환
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        cos_d_lon = math.cos(lon_rad - self.ref_lon)

        # 내적 값 계산 후 -1~1 범위로 제한
        arg = self.ref_sin_lat * sin_lat + self.ref_cos_lat * cos_lat * cos_d_lon
        arg = max(min(arg, 1.0), -1.0)
        c = math.acos(arg)

        # c가 0이 아니면 비율 k 계산 (c/sin(c))
        k = c / math.sin(c) if abs(c) > 1e-6 else 1.0

        x = k * (self.ref_cos_lat * sin_lat - self.ref_sin_lat * cos_lat * cos_d_lon) * EARTH_RADIUS
        y = k * cos_lat * math.sin(lon_rad - self.ref_lon) * EARTH_RADIUS

        return x, y

    def get_distance_between_ned(self, x_now, y_now, z_now, x_next, y_next, z_next):
        # NED 좌표로 주어진 두 점 사이의 거리 구하기
        dx = x_next - x_now
        dy = y_next - y_now
        dz = z_next - z_now

        dist_xy = math.sqrt(dx*dx+dy*dy)
        dist_z = abs(dz)
        total_dist = math.sqrt(dist_xy*dist_xy + dist_z*dist_z)

        return total_dist, dist_xy, dist_z

    def is_waypoint_reached(self, waypoint):
        # NED 좌표를 기반으로 계산했을 때 경로점에 도착했는지 여부를 판단하기
        total_dist, dist_xy, dist_z = self.get_distance_between_ned(
            self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z,
            waypoint["x"], waypoint["y"], waypoint["z"])
        
        if dist_xy <= self.threshold_range and dist_z <= self.threshold_range:
            return True
        else:
            return False

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # WGS84 좌표를 기반으로, 두 점 사이의 yaw값을 계산하기 (제어 등에 사용)
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        initial_bearing = math.atan2(y, x)
        # Convert to degrees
        initial_bearing = math.degrees(initial_bearing)
        # Normalize to 0-360
        bearing = (initial_bearing + 360) % 360
        return math.radians(bearing)

    def get_attitude(self, current_wp, next_wp):
        # NED 좌표로 주어진 두 점 사이의 yaw 값 계산하기 (제어에 사용)
        dx = next_wp["x"] - current_wp["x"]
        dy = next_wp["y"] - current_wp["y"]

        # arctan2는 (dy, dx)를 인자로 받아 라디안 단위의 각도를 반환.
        yaw_rad = math.atan2(dy, dx)

        # 라디안을 도 단위로 변환
        yaw_deg = math.degrees(yaw_rad)
        return yaw_deg
    
    def get_distance_to_point_global_wgs84(self, lat_now, lon_now, lat_next, lon_next):
        # 현재 WGS84 좌표와
        # 다음 경로점까지의 WGS84 경로점 좌표를 받아서
        # 두 점 사이의 거리를 WGS84 좌표 기반으로 계산해서 출력 : 단, 고도는 고려하지 않음

        # 위도와 경도를 라디안 단위로 변환
        current_x_rad = math.radians(lat_next)
        current_y_rad = math.radians(lon_next)
        x_rad = math.radians(lat_now)
        y_rad = math.radians(lon_now)

        # 위도 및 경도 차이를 라디안 단위로 계산
        d_lat = x_rad - current_x_rad
        d_lon = y_rad - current_y_rad

        # 하버사인 공식을 이용해 두 지점 사이의 중심각(c)을 계산
        a = math.sin(d_lat / 2.0) ** 2 + math.sin(d_lon / 2.0) ** 2 * math.cos(current_x_rad) * math.cos(x_rad)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        # 수평거리 (dxy, 미터 단위) 계산
        dxy = EARTH_RADIUS * c

        # 각각의 절대값 계산
        dist_xy = abs(dxy)

        return dist_xy

######################## Callback Functions #######################
                # Functions for subscribing messages
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback for vehicle local position."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_attitude_callback(self, vehicle_attitude):
        """Callback for vehicle attitude."""
        self.vehicle_attitude = vehicle_attitude

    def vehicle_status_callback(self, vehicle_status):
        """Callback for vehicle status."""
        self.vehicle_status = vehicle_status

    def vehicle_global_position_callback(self, vehicle_global_position):
        """Callback for vehicle global position."""
        self.vehicle_global_position = vehicle_global_position

########################## Command Functions #######################
                    # Functions for Offboard Control
    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Send a command to engage offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Send a command to land the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_heartbeat_ob_pos_sp(self):
        """Publish heartbeat message for offboard control."""
        msg = OffboardControlMode()
        msg.position    = True
        msg.velocity    = False
        msg.acceleration= False
        msg.attitude    = False
        msg.body_rate   = False
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw_d: float):
        """Publish position setpoint."""
        msg = TrajectorySetpoint()
        msg.position    = [float(x), float(y), float(z)]
        msg.velocity    = [np.nan, np.nan, np.nan]
        msg.acceleration= [np.nan, np.nan, np.nan]
        msg.yaw         = np.clip(np.deg2rad(yaw_d), -np.pi, np.pi)
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params):
        """Publish vehicle command."""
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

    def transition_fw(self):
        """Publish FW Transition Command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=4.0)
        self.get_logger().info("Transition to fixed-wing command sent")

    def transition_mc(self):
        """Publish MC Transition Command"""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, param1=3.0)
        self.get_logger().info("Transition to multi-copter command sent")

############################## Timer Callback Function #######################
# Functions for Finite State Machine and Sequential Offboard Control
    def timer_callback(self):
        # Take-off 상태 처리
        if self.state == "NOT_READY":
            self.publish_heartbeat_ob_pos_sp()
            if self.offboard_setpoint_counter < 10:
                self.offboard_setpoint_counter += 1
            self.offboard_setpoint_counter %= 11
            if self.offboard_setpoint_counter < 5:
                self.pos_x = 0.0
                self.pos_y = 0.0
                self.pos_z = self.takeoff_height
                # self.pos_yaw = np.rad2deg(self.vehicle_euler[0])
                self.pos_yaw = self.get_attitude(self.ned_waypoints["WP0"], self.ned_waypoints["WP1"])
                self.engage_offboard_mode()
            if self.offboard_setpoint_counter == 9:
                self.arm()

            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Move to Home")
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP1"])
            if (self.waypoint_reach_or_not == True and
                self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.get_logger().info("WP1 REACHED")
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP0"]["lat"],
                                                                           self.wgs84_waypoints["WP0"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP0']}")
                self.state = "WAYPOINT_1"

        elif self.state == "WAYPOINT_1":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP1"], self.ned_waypoints["WP2"])
            self.pos_x = self.ned_waypoints["WP2"]["x"]
            self.pos_y = self.ned_waypoints["WP2"]["y"]
            self.pos_z = self.ned_waypoints["WP2"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.transition_fw()
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP2"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP2, and Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_2"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP2"]["lat"],
                                                                           self.wgs84_waypoints["WP2"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP2']}")

        elif self.state == "WAYPOINT_2":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP2"], self.ned_waypoints["WP3"])
            self.pos_x = self.ned_waypoints["WP3"]["x"]
            self.pos_y = self.ned_waypoints["WP3"]["y"]
            self.pos_z = self.ned_waypoints["WP3"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP3"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_3"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP3"]["lat"],
                                                                           self.wgs84_waypoints["WP3"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP3']}")

        elif self.state == "WAYPOINT_3":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP3"], self.ned_waypoints["WP4"])
            self.pos_x = self.ned_waypoints["WP4"]["x"]
            self.pos_y = self.ned_waypoints["WP4"]["y"]
            self.pos_z = self.ned_waypoints["WP4"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP4"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_4"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP4"]["lat"],
                                                                           self.wgs84_waypoints["WP4"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP4']}")

        elif self.state == "WAYPOINT_4":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP4"], self.ned_waypoints["WP5"])
            self.pos_x = self.ned_waypoints["WP5"]["x"]
            self.pos_y = self.ned_waypoints["WP5"]["y"]
            self.pos_z = self.ned_waypoints["WP5"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP5"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_5"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP5"]["lat"],
                                                                           self.wgs84_waypoints["WP5"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP5']}")

        elif self.state == "WAYPOINT_5":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP5"], self.ned_waypoints["WP6"])
            self.pos_x = self.ned_waypoints["WP6"]["x"]
            self.pos_y = self.ned_waypoints["WP6"]["y"]
            self.pos_z = self.ned_waypoints["WP6"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.transition_mc()
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP6"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_6"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP6"]["lat"],
                                                                           self.wgs84_waypoints["WP6"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP5']}")


        elif self.state == "WAYPOINT_6":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP6"], self.ned_waypoints["WP7"])
            self.pos_x = self.ned_waypoints["WP7"]["x"]
            self.pos_y = self.ned_waypoints["WP7"]["y"]
            self.pos_z = self.ned_waypoints["WP7"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP7"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_7"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP7"]["lat"],
                                                                           self.wgs84_waypoints["WP7"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP5']}")


        elif self.state == "WAYPOINT_7":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP7"], self.ned_waypoints["WP8"])
            self.pos_x = self.ned_waypoints["WP8"]["x"]
            self.pos_y = self.ned_waypoints["WP8"]["y"]
            self.pos_z = self.ned_waypoints["WP8"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP8"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_8"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP8"]["lat"],
                                                                           self.wgs84_waypoints["WP8"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP5']}")


        elif self.state == "WAYPOINT_8":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP8"], self.ned_waypoints["WP9"])
            self.pos_x = self.ned_waypoints["WP9"]["x"]
            self.pos_y = self.ned_waypoints["WP9"]["y"]
            self.pos_z = self.ned_waypoints["WP9"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP9"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_9"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP9"]["lat"],
                                                                           self.wgs84_waypoints["WP9"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP5']}")


        elif self.state == "WAYPOINT_9":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP9"], self.ned_waypoints["WP10"])
            self.pos_x = self.ned_waypoints["WP10"]["x"]
            self.pos_y = self.ned_waypoints["WP10"]["y"]
            self.pos_z = self.ned_waypoints["WP10"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP10"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_10"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP10"]["lat"],
                                                                           self.wgs84_waypoints["WP10"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP5']}")


        elif self.state == "WAYPOINT_10":
            self.publish_heartbeat_ob_pos_sp()
            self.pos_yaw = self.get_attitude(self.ned_waypoints["WP10"], self.ned_waypoints["WP11"])
            self.pos_x = self.ned_waypoints["WP11"]["x"]
            self.pos_y = self.ned_waypoints["WP11"]["y"]
            self.pos_z = self.ned_waypoints["WP11"]["z"]
            
            self.publish_position_setpoint(self.pos_x, self.pos_y, self.pos_z, self.pos_yaw)
            self.waypoint_reach_or_not = self.is_waypoint_reached(self.ned_waypoints["WP11"])
            print("Pxyz {:6.2f}, {:6.2f}, {:6.2f}".format(
                self.vehicle_local_position.x,
                self.vehicle_local_position.y,
                self.vehicle_local_position.z), end=' ')
            print(" / Fly to WP3 and Reverse Transition Completed")

            if self.waypoint_reach_or_not == True:
                self.state = "WAYPOINT_11"
                self.global_dist = self.get_distance_to_point_global_wgs84(self.vehicle_global_position.lat, 
                                                                           self.vehicle_global_position.lon, 
                                                                           self.wgs84_waypoints["WP11"]["lat"],
                                                                           self.wgs84_waypoints["WP11"]["lon"])
                self.get_logger().info(f"dist: {self.global_dist}, pos: {self.vehicle_global_position}, wp: {self.wgs84_waypoints['WP5']}")
        
        elif self.state == "WAYPOINT_11":
            self.publish_heartbeat_ob_pos_sp()
            self.land()
            self.get_logger().info("Landing...")
            rclpy.shutdown()

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
