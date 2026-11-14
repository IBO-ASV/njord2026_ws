# Sensor build support

Shared CMake helpers for CPU/GPU backend selection and build manifests. Standard
CMake detection (`CheckLanguage`, `CUDAToolkit`, `find_package`) performs dependency
checks; this package supplies workspace-wide profile policy and writes
`platform.yaml` / `platform.env` for launch and preflight inspection.

Consumers use `find_package(sensor_build_support REQUIRED)`,
`sensor_detect_backends()`, and `sensor_write_build_manifest()`. Existing
`NJORD_*` cache and environment options remain available.

This package contains no runtime node, vessel model, or hardware abstraction.
