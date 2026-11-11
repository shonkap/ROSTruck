from glob import glob

from setuptools import find_packages, setup

package_name = 'fc_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pymavlink', 'pyserial'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='MAVLink bridge between the SEQURE H743 FC (ArduPilot Rover) and ROS2.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mavlink_bridge_node = fc_bridge.mavlink_bridge_node:main',
        ],
    },
)
