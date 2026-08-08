from setuptools import find_packages, setup

package_name = "task2_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/task2_perception.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/task2_params.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="IBO-ASV",
    maintainer_email="pigleteshima@gmail.com",
    description="Task 2 LiDAR perception glue: cloud filtering for the "
                "pcl_segmentation pipeline and opponent-ship selection for "
                "the MPPI planner.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task2_cloud_filter_node = task2_perception.cloud_filter_node:main",
            "opponent_selector_node = task2_perception.opponent_selector_node:main",
        ],
    },
)
