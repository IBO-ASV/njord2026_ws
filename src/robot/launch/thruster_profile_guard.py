"""Validate the mutually-dependent physical thruster profile files."""

import os


_THREE_WHEEL_PROFILE_SETS = {
    (
        "omni_md10c3.yaml",
        "omni_3wheel.urdf",
        "omni_md10c3.yaml",
    ),
    (
        "omni_md10c3_calibration.yaml",
        "omni_3wheel.urdf",
        "omni_md10c3_calibration.yaml",
    ),
}


def validate_thruster_profile_combination(
    enable_thruster, thruster_config_file, thruster_robot_description_file,
    thruster_serial_config_file,
):
    """Reject partial selection of the incompatible three-wheel MD10C profile.

    A three-wheel launch changes the allocator, geometry, and UART packet type
    together.  Treating any one of the three files as an independent override
    can otherwise pair a three-channel wheel command with the four-ESC vessel
    firmware profile.
    """
    if str(enable_thruster).strip().lower() not in ("true", "1", "yes", "on"):
        return

    selected = (
        os.path.basename(str(thruster_config_file)),
        os.path.basename(str(thruster_robot_description_file)),
        os.path.basename(str(thruster_serial_config_file)),
    )
    selects_three_wheel_file = any(
        value.startswith("omni_md10c3") or value == "omni_3wheel.urdf"
        for value in selected
    )
    if selects_three_wheel_file and selected not in _THREE_WHEEL_PROFILE_SETS:
        raise RuntimeError(
            "Three-wheel MD10C bringup requires a matching normal or calibration "
            "profile set (host YAML, omni_3wheel.urdf, and serial YAML). "
            f"Refusing mismatched physical-output startup: {selected}."
        )
