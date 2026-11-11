from glob import glob

from setuptools import find_packages, setup

package_name = 'truck_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Ground Pounder bringup: stereo cameras, SLAM, calibration tools.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stereo_camera_node = truck_bringup.stereo_camera_node:main',
            'stereo_check = truck_bringup.stereo_check:main',
            'map_saver = truck_bringup.map_saver:main',
            'collision_guard = truck_bringup.collision_guard:main',
            'detector_node = truck_bringup.detector_node:main',
            'recorder_node = truck_bringup.recorder_node:main',
            'scan_guard = truck_bringup.scan_guard:main',
        ],
    },
)
