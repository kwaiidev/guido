from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'guido_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='Guido Team',
    maintainer_email='todo@guido.dev',
    description='LiDAR perception, planning, control, and safety stack for Guido autonomy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_perception = guido_navigation.lidar_perception:main',
            'localization_bridge = guido_navigation.localization_bridge:main',
            'mission_manager = guido_navigation.mission_manager:main',
            'global_planner = guido_navigation.global_planner:main',
            'trajectory_controller = guido_navigation.trajectory_controller:main',
            'command_mux = guido_navigation.command_mux:main',
            'safety_monitor = guido_navigation.safety_monitor:main',
            'telemetry_bridge = guido_navigation.telemetry_bridge:main',
        ],
    },
)
