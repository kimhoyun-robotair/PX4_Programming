import launch
import launch_ros
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 패키지 이름과 패키지 경로
    package_name = "vtol_control"
    pkg_share = launch_ros.substitutions.FindPackageShare(package=package_name).find(package_name)
    
    # 파라미터 파일 경로 (os.path.join을 사용)
    config_path = os.path.join(pkg_share, "config.yaml")
    
    # 노드 생성 시 파라미터 파일 추가
    multirotor_node = Node(
        package="vtol_control",
        executable="vtol_control",
        name="vtol_control",
        parameters=[config_path]
    )
    
    return launch.LaunchDescription([
        multirotor_node
    ])
