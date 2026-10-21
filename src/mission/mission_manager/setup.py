from glob import glob

from setuptools import find_packages, setup


package_name = "mission_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="NJORD",
    maintainer_email="oswystk15662@keio.jp",
    description="Serialized task execution manager for NJORD.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager_node = mission_manager.mission_manager_node:main",
            "dock_wall_correction_node = mission_manager.dock_wall_correction_node:main",
            "ground_link_return_monitor_node = mission_manager.ground_link_return_node:main",
            "runtime_manager_node = mission_manager.runtime_manager_node:main",
            "operator_dispatcher_node = mission_manager.operator_dispatcher_node:main",
            "njord-task = mission_manager.njord_task:main",
            "njord-critical = mission_manager.njord_critical:main",
        ],
    },
)
