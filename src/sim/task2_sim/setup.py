from setuptools import find_packages, setup

package_name = "task2_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/task2_sim.launch.py",
            "launch/task2_sim_bringup.launch.py",
        ]),
        (
            "share/" + package_name + "/config",
            ["config/task2_params.yaml", "config/task2_opponent_sim.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osw",
    maintainer_email="oswystk15662@keio.jp",
    description="Task2 simulation orchestrator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task2_orchestrator = task2_sim.orchestrator:main",
            "opponent_vessel_node = task2_sim.opponent_vessel:main",
            "ideal_lidar_pointcloud_node = task2_sim.ideal_lidar_pointcloud:main",
            "cmd_vel_kinematic_sim = task2_sim.cmd_vel_kinematic_sim:main",
        ],
    },
)
