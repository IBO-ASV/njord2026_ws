from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yolo'


def package_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.relpath(os.path.join(path, filename), package_name))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: package_files(os.path.join(package_name, 'yolov10-main')),
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osw',
    maintainer_email='oswystk15662@keio.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_node = yolo.main:main',
            'yolo_cuda_node = yolo.main:cuda_main',
            'yolo11_node = yolo.yolo11_node:main',
            'yolo_fusion_node = yolo.fusion_main:main',
        ],
    },
)
