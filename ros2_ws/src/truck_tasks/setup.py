from glob import glob

from setuptools import find_packages, setup

package_name = 'truck_tasks'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Task bus for the truck: task manager, navigator (goto), explorer (map_room).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_manager = truck_tasks.task_manager:main',
            'navigator = truck_tasks.navigator:main',
            'explorer = truck_tasks.explorer:main',
            'places_node = truck_tasks.places_node:main',
            'watchdog = truck_tasks.watchdog:main',
        ],
    },
)
