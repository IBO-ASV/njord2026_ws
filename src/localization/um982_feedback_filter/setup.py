from setuptools import find_packages, setup

package_name = 'um982_feedback_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config',
            ['config/um982_feedback_ekf.yaml', 'config/um982_glim_imu_ekf.yaml'],
        ),
        ('share/' + package_name + '/launch', ['launch/um982_feedback.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'window_feedback_node = um982_feedback_filter.window_feedback_node:main',
        ],
    },
)
