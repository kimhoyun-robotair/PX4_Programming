from setuptools import find_packages, setup

package_name = 'practice_control'

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
            "multirotor_takeoff_land = practice_control.multirotor_takeoff_land:main",
            "multirotor_flight = practice_control.multirotor_flight:main",
            "multirotor_flight_velocity = practice_control.multirotor_flight_velocity:main",
        ],
    },
)
