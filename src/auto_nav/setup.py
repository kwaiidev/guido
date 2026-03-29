from glob import glob
import os

from setuptools import find_packages
from setuptools import setup


package_name = 'auto_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Guido Team',
    maintainer_email='todo@guido.dev',
    description='Recovered autonomous navigation stack for Guido waypointing and frontier exploration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'auto_nav_command_node = navigation.command_node:main',
            'auto_nav_navigation_node = navigation.navigation_node:main',
        ],
    },
)
