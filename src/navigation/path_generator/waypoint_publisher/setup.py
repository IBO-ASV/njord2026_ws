from setuptools import find_packages, setup

package_name = "waypoint_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/waypoint_publisher.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/task1_waypoints.yaml",
            "config/task1_skip_1_1_waypoints.yaml",
            "config/task1_follow_waypoints.yaml",
            "config/task2_waypoints.yaml",
            "config/task3_waypoints.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aslb",
    maintainer_email="yoshitaka.osawa@fulldepth.co.jp",
    description="Waypoint publisher for task-specific navigation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "waypoint_publisher_node = waypoint_publisher.waypoint_publisher_node:main",
            "task2_gate_midpoint_publisher = waypoint_publisher.task2_gate_midpoint_publisher:main",
        ],
    },
)
