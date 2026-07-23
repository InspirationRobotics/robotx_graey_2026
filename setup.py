from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robotx_graey_2026'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='chrismartin.ee.ucsd@gmail.com',
    description='Graey UUV - RobotX 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'led_node = robotx_graey_2026.api.led.led_node:main',
            'pixhawk_led_node = robotx_graey_2026.api.led.pixhawk_led_node:main',
            'vn100_node = robotx_graey_2026.api.navigation.vn100_node:main',
            'prequal_mission = robotx_graey_2026.api.navigation.prequal_mission:main',
            'dvl_node = robotx_graey_2026.api.navigation.dvl_node:main',
            'nav_ekf_bridge = robotx_graey_2026.api.navigation.nav_ekf_bridge:main',
            'dvl_ekf_bridge = robotx_graey_2026.api.navigation.dvl_ekf_bridge:main',
        ],
    },
)
