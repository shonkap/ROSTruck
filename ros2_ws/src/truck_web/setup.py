from glob import glob

from setuptools import find_packages, setup

package_name = 'truck_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Web page: live map, click-to-drive waypoints, task queue, planner requests.',
    license='MIT',
    entry_points={'console_scripts': ['web_node = truck_web.web_node:main']},
)
