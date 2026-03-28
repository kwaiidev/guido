from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'guido_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Guido Team',
    maintainer_email='todo@guido.dev',
    description='Serial bridge between ROS 2 and the Guido wheelchair Arduino',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge = guido_base.serial_bridge:main',
        ],
    },
)
