from setuptools import setup, find_packages

package_name = 'robotx_graey_2026'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris',
    maintainer_email='chm018@ucsd.edu',
    description='Graey UUV - RobotX 2026',
    license='MIT',
    entry_points={
        'console_scripts': [
            'dvl_node = robotx_graey_2026.api.navigation.dvl_node:main',
            'dvl_ekf_bridge = robotx_graey_2026.api.navigation.dvl_ekf_bridge:main',
        ],
    },
)
