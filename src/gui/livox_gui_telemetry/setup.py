from setuptools import setup

setup(name="livox_gui_telemetry", version="0.1.0", packages=["livox_gui_telemetry"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/livox_gui_telemetry"]),
                  ("share/livox_gui_telemetry", ["package.xml"])], install_requires=["setuptools"], zip_safe=True,
      entry_points={"console_scripts": ["livox_gui_downsampler = livox_gui_telemetry.nodes:downsampler_main", "livox_splat_mapper = livox_gui_telemetry.nodes:mapper_main"]})
