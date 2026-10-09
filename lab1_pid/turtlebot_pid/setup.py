from setuptools import find_packages, setup

package_name = 'turtlebot_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sasi',
    maintainer_email='sasidhar.alavala.work@gmail.com',
    description='CP 241 Lab 1: PID go-to-goal controller for TurtleBot3',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_node = turtlebot_pid.pid_node:main',
            'pid_node_mocap = turtlebot_pid.pid_node_mocap:main',
        ],
    },
)
