from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'guido_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (
            os.path.join('lib', package_name),
            [
                os.path.join('bin', 'serial_bridge'),
                os.path.join('bin', 'keyboard_teleop'),
            ],
        ),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Guido Team',
    maintainer_email='todo@guido.dev',
    description='Serial bridge between ROS 2 and the Guido wheelchair Arduino',
    license='Apache-2.0',
)
