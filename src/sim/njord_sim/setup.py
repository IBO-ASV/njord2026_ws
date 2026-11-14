from glob import glob
from setuptools import setup

setup(
    name="njord_sim",
    version="0.1.0",
    packages=["njord_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/njord_sim"]),
        ("share/njord_sim", ["package.xml"]),
        ("share/njord_sim/launch", glob("launch/*.py")),
        ("share/njord_sim/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    maintainer="IBO-ASV",
    maintainer_email="ibo@example.com",
    description="Hardware-free mission integration and acceptance simulation",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "environment = njord_sim.environment:main",
            "velocity_experiment = njord_sim.velocity_experiment:main",
            "evaluate = njord_sim.evaluate:main",
        ]
    },
)
