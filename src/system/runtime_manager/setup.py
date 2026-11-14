from setuptools import setup


setup(
    name="runtime_manager",
    version="0.1.0",
    packages=["runtime_manager"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/runtime_manager"]),
        ("share/runtime_manager", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="NJORD",
    maintainer_email="oswystk15662@keio.jp",
    description="Own and replace the selected Nav2 runtime process",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "runtime_manager_node = runtime_manager.runtime_manager_node:main",
        ],
    },
)
