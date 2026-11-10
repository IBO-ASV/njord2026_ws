from setuptools import find_packages, setup

package_name = "task1_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/task1_navigation.launch.py",
            "launch/task1_sim.launch.py",
            "launch/task1_sim_truth.launch.py",
            "launch/task1_sim_sensor_parity.launch.py",
        ]),
        ("share/" + package_name + "/behavior_trees", [
            "behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml",
        ]),
        ("share/" + package_name + "/config", [
            "config/task1_params.yaml",
            "config/task1_nav2_params_humble.yaml",
            "config/task1_nav2_params_jazzy.yaml",
            "config/task1_ekf_local.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osw",
    maintainer_email="oswystk15662@keio.jp",
    description="Task1 simulation orchestrator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task1_orchestrator = task1_sim.orchestrator:main",
            "cardinal_perception_sim = task1_sim.cardinal_perception_sim:main",
            "gui_status_dummy_publisher = task1_sim.gui_status_dummy_publisher:main",
        ],
    },
)
