# Runtime parameters

Edit `runtime_parameters.yaml` on the ground PC while its `ground_pc.launch.py`
bringup is running. For a persistent editable location, start it with
`runtime_parameters_file:=/path/to/runtime_parameters.yaml`. The update is sent
to the miniPC and Jetson and each local `runtime_parameter_sync` applies that
role's entries through its ROS 2 parameter service.

The target node must support changing that parameter while running. For
example, `/joy_converter` accepts its `scale.*` values; startup-only YAML
settings still require a restart.
