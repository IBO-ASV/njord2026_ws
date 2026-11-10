from setuptools import setup
import os
from glob import glob

package_name = 'buoy_obstacle_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='njord',
    maintainer_email='njord@njord.invalid',
    description='Buoy TF → Nav2 OccupancyGrid obstacle publisher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'buoy_obstacle_publisher = buoy_obstacle_publisher.buoy_obstacle_publisher_node:main',
            'field_boundary_publisher = buoy_obstacle_publisher.field_boundary_publisher_node:main',
            'cardinal_wall_publisher = buoy_obstacle_publisher.cardinal_wall_publisher_node:main',
        ],
    },
)
