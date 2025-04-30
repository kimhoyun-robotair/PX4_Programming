from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_control_wgs84'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
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
            "multirotor_control = px4_control_wgs84.multirotor_control:main",
            "fixedwing_control = px4_control_wgs84.fixedwing_control:main",
            "vtol_control = px4_control_wgs84.vtol_control:main",
        ],
    },
)
