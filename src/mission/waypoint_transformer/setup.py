from glob import glob
from setuptools import find_packages, setup


setup(
    name="waypoint_transformer",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/waypoint_transformer"]),
        ("share/waypoint_transformer", ["package.xml"]),
        ("share/waypoint_transformer/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
