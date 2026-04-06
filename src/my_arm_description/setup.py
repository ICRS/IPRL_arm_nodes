import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_arm_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        
        # Include config files (YAMLs)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        
        # Include Rviz configuration files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        
        # Include URDF/Xacro files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='sosebeyo@gmail.com',
    description='Package for Arm Control and Description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # executable_name = package.script_name:main_function
            'evdev_joy = my_arm_description.evdev_joy:main',
        ],
    },
)