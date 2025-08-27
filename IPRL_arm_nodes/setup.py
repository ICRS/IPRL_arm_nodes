from setuptools import find_packages, setup

package_name = 'IPRL_arm_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/arm_utilities.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mteo',
    maintainer_email='mit21@ic.ac.uk',
    description='ROS2 nodes taking input from a controller to control the IPRL robot arm',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'joy_2_command = IPRL_arm_nodes.joy_2_command:main',
                'serial_interface = IPRL_arm_nodes.serial_interface:main',
                'visualiser = IPRL_arm_nodes.visualiser:main',
        ],
    },
)
