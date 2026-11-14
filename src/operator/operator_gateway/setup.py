from setuptools import setup


setup(
    name="operator_gateway",
    version="0.1.0",
    packages=["operator_gateway"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/operator_gateway"]),
        ("share/operator_gateway", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    maintainer="NJORD",
    maintainer_email="oswystk15662@keio.jp",
    description="Validate and dispatch typed operator commands",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "operator_dispatcher_node = operator_gateway.operator_dispatcher_node:main",
            "njord-critical = operator_gateway.njord_critical:main",
        ],
    },
)
