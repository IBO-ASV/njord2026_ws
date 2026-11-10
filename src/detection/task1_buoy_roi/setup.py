from setuptools import find_packages, setup


setup(
    name="task1_buoy_roi",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/task1_buoy_roi"]),
        ("share/task1_buoy_roi", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "task1_buoy_roi = task1_buoy_roi.node:main",
    ]},
)
