from glob import glob
import os

from setuptools import setup


package_name = "control_manager"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NJORD",
    maintainer_email="oswystk15662@keio.jp",
    description="Canonical control-mode and command safety manager.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mode_manager = control_manager.mode_manager_node:main",
            "safety_supervisor = control_manager.safety_supervisor_node:main",
            "command_arbiter = control_manager.command_arbiter_node:main",
        ],
    },
)
