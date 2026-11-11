from setuptools import find_packages, setup

package_name = 'truck_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='LLM planner plugin (Claude): natural-language requests -> report + tasks.',
    license='MIT',
    entry_points={'console_scripts': ['planner_node = truck_planner.planner_node:main']},
)
