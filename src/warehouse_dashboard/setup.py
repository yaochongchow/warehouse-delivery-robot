import os
from glob import glob
from setuptools import setup

package_name = 'warehouse_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycchow',
    maintainer_email='ycchow@example.com',
    description='Dashboard and monitoring tools for the warehouse delivery robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'terminal_dashboard = warehouse_dashboard.terminal_dashboard:main',
            'rosbag_fault_recorder = warehouse_dashboard.rosbag_fault_recorder:main',
        ],
    },
)
