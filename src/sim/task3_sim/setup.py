from setuptools import find_packages, setup

package_name = "task3_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/task3_sim.launch.py"]),
        ("share/" + package_name + "/config", [
            "config/task3_params.yaml",
            "config/task3_ekf_global.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="osw",
    maintainer_email="oswystk15662@keio.jp",
    description="Task3 simulation orchestrator",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task3_orchestrator = task3_sim.orchestrator:main",
            "task3_gui_status_dummy = task3_sim.gui_status_dummy:main",
        ],
    },
)
