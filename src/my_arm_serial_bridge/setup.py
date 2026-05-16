import os
from glob import glob

from setuptools import setup

package_name = 'my_arm_serial_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='sosebeyo@gmail.com',
    description='Raw serial bridge for ESP32 arm testing',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = my_arm_serial_bridge.serial_bridge_node:main',
            'serial_console = my_arm_serial_bridge.serial_console_node:main',
        ],
    },
)
