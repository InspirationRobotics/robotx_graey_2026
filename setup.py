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
        (os.path.join('share', package_name, 'tools'), glob('tools/*.html') + glob('tools/*.png')),
        (os.path.join('share', package_name, 'params'), glob('params/*.parm')),
    ] + [(os.path.join('share', package_name, folder),
          [os.path.join(folder, name) for name in files])
         for folder, _, files in os.walk('tools/vendor') if files],
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
            'kill_switch = robotx_graey_2026.api.pixhawk.kill_switch:main',
            'dvl_node = robotx_graey_2026.api.navigation.dvl_node:main',
            'vn100_node = robotx_graey_2026.api.navigation.vn100_node:main',
            'nav_ekf_bridge = robotx_graey_2026.api.navigation.nav_ekf_bridge:main',
            'navigation_supervisor = robotx_graey_2026.api.navigation.navigation_supervisor:main',
            'pos_server = robotx_graey_2026.api.navigation.pos_server:main',
            'pole_tracker = robotx_graey_2026.api.vision.pole_tracker:main',
            'gui_node = robotx_graey_2026.api.gui.gui_node:main',
            'prequal_mission = robotx_graey_2026.api.navigation.prequal_mission:main',
            'prequal_mission_cv = robotx_graey_2026.api.navigation.prequal_mission_cv:main',
            'demo_mission = robotx_graey_2026.api.navigation.demo_mission:main',
            'sitl_sensor_sim = robotx_graey_2026.api.navigation.sitl_sensor_sim:main',
            'sitl_mission = robotx_graey_2026.api.navigation.sitl_mission:main',
        ],
    },
)
