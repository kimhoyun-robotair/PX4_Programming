from setuptools import find_packages, setup

package_name = 'px4_offboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimhoyun',
    maintainer_email='suberkut76@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fixedwing_offboard_control = px4_offboard_control.fixedwing_offboard_control:main",
            "fixedwing_takeoff_land = px4_offboard_control.fixedwing_takeoff_land:main",
            "multicopter_offboard_control = px4_offboard_control.multicopter_offboard_control:main",
            "multirotor_takeoff_land = px4_offboard_control.multirotor_takeoff_land:main",
            "multirotor_flight_velocity = px4_offboard_control.multirotor_flight_velocity:main",
            "multirotor_flight = px4_offboard_control.multirotor_flight:main",
        ],
    },
)
