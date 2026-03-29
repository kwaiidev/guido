import os
from pathlib import Path

from setuptools import find_packages
from setuptools import setup


package_name = 'auto_nav'

# Enumerate config explicitly so removed/renamed files do not leave stale symlinks
# in build/ and install/ (e.g. slam_toolbox.yaml vs old slam_*.yaml names).
# Paths must stay relative (colcon rejects absolute data_files sources).
_config_dir = Path('config')
CONFIG_DATA_FILES = sorted(
    str(p) for p in _config_dir.iterdir() if p.is_file()
)

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            [str(p) for p in Path('launch').glob('*.launch.py')],
        ),
        (os.path.join('share', package_name, 'config'), CONFIG_DATA_FILES),
        (
            os.path.join('lib', package_name),
            [
                os.path.join('bin', 'auto_nav_command_node'),
                os.path.join('bin', 'auto_nav_navigation_node'),
                os.path.join('bin', 'frontier_explorer'),
                os.path.join('bin', 'verify_nav_motion'),
                os.path.join('bin', 'cmd_vel_bridge'),
            ],
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Guido Team',
    maintainer_email='todo@guido.dev',
    description='Autonomous navigation stack for Guido waypointing and Nav2 path following',
    license='Apache-2.0',
)
