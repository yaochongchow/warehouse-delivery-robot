import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'warehouse_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'behavior_trees'),
            glob(os.path.join('behavior_trees', '*.xml'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*'))),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycchow',
    maintainer_email='ycchow@example.com',
    description='Navigation package for the warehouse robot using Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
