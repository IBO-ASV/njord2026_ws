import os
from glob import glob
from setuptools import find_packages, setup

package_name = "asv_trajectory_planner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="your_email@example.com",
    description="ASV trajectory planner node with Nav2 FollowPath client",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_pruner_node = asv_trajectory_planner.path_pruner_node:main",
            "planner_node = asv_trajectory_planner.planner_node:main",
            "follow_path_client_node = asv_trajectory_planner.follow_path_client_node:main",
            "task2_gps_waypoint_publisher = asv_trajectory_planner.task2_gps_waypoint_publisher:main",
            "opponent_twist_from_tf_node = asv_trajectory_planner.opponent_twist_from_tf_node:main",
            "task2_waypoint_pose_publisher = asv_trajectory_planner.task2_waypoint_pose_publisher:main",
            "task2_autonomy_ready_node = asv_trajectory_planner.task2_autonomy_ready_node:main",
            "crm_costmap_node = asv_trajectory_planner.crm_costmap_node:main",
        ],
    },
)
