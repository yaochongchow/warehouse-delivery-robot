import os
from glob import glob
from setuptools import setup

package_name = 'warehouse_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycchow',
    maintainer_email='ycchow@example.com',
    description='Task management and battery simulation for the warehouse delivery robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_manager_node = warehouse_task_manager.task_manager_node:main',
            'battery_simulator = warehouse_task_manager.battery_simulator:main',
            'manipulation_simulator = warehouse_task_manager.manipulation_simulator:main',
        ],
    },
)
