from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robotx_graey_2026'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'robotx_graey_2026.api.gui.laptop':['*.html','*.js','*.css']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*.html') + glob('tools/*.png') + glob('tools/*.jpg') + glob('tools/*.css') + glob('tools/*.js')),
        (os.path.join('share', package_name, 'tools/scripts'),glob('tools/scripts/*.cmd')),
        (os.path.join('share', package_name, 'params'), glob('params/*.parm')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + [(os.path.join('share', package_name, folder),
          [os.path.join(folder, name) for name in files])
         for folder, _, files in os.walk('tools/vendor') if files],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='chrismartin.ee.ucsd@gmail.com',
    description='Graey UUV - RobotX 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gcs_bridge = robotx_graey_2026.api.gui.gcs_transport:bridge_main',
            'graey_gcs = robotx_graey_2026.api.gui.gcs_transport:laptop_main',
            'telemetry_gateway = robotx_graey_2026.api.pixhawk.telemetry_gateway:main',
            'competition_link_node = robotx_graey_2026.api.comms.competition_link_node:main',
            'radio_mission_node = robotx_graey_2026.api.comms.radio_mission_node:main',
            'mesh_link = robotx_graey_2026.api.comms.mesh_link:main',
            'sub_link_node = robotx_graey_2026.api.comms.sub_link_node:main',
            'led_node = robotx_graey_2026.api.led.led_node:main',
            'pixhawk_led_node = robotx_graey_2026.api.led.pixhawk_led_node:main',
            'kill_switch = robotx_graey_2026.api.pixhawk.kill_switch:main',
            'dvl_node = robotx_graey_2026.api.navigation.dvl_node:main',
            'vn100_node = robotx_graey_2026.api.navigation.vn100_node:main',
            'fused_nav_bridge = robotx_graey_2026.api.navigation.fused_nav_bridge:main',
            'navigation_supervisor = robotx_graey_2026.api.navigation.navigation_supervisor:main',
            'gui_node = robotx_graey_2026.api.gui.gui_node:main',
            'sitl_sensor_sim = robotx_graey_2026.api.navigation.sitl_sensor_sim:main',
            'sitl_mission = robotx_graey_2026.api.navigation.sitl_mission:main',
            'waypoint_mission = robotx_graey_2026.api.navigation.waypoint_mission:main',
            'north_maneuver = robotx_graey_2026.api.navigation.north_maneuver_node:main',
        ],
    },
)
