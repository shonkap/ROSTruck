from glob import glob

from setuptools import find_packages, setup

package_name = 'truck_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Gazebo + ArduPilot SITL simulation of the Ground Pounder truck.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stereo_relay = truck_sim.stereo_relay:main',
            'clock_relay = truck_sim.clock_relay:main',
            'odom_eval = truck_sim.odom_eval:main',
            'sim_ctl = truck_sim.sim_ctl:main',
            'explore_room = truck_sim.explore_room:main',
            'turn_test = truck_sim.turn_test:main',
        ],
    },
)
