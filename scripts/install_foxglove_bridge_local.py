#!/usr/bin/env python3
"""Install the ROS apt bridge binaries in this workspace without sudo.

Prefer `sudo apt install ros-$ROS_DISTRO-foxglove-bridge` when available.
This fallback extracts signed-repository apt downloads without running package
maintainer scripts, then registers a relocatable colcon runtime prefix.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--prefix",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "install/foxglove_bridge",
    )
    args = parser.parse_args()
    prefix = args.prefix.resolve()
    if prefix.exists():
        parser.error(f"prefix already exists: {prefix}; refusing to overwrite")
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    packages = [
        f"ros-{distro}-{name}"
        for name in ("foxglove-bridge", "rosx-introspection", "resource-retriever")
    ]
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    with tempfile.TemporaryDirectory(
        prefix="njord-foxglove-", dir=runtime
    ) as temporary:
        staging = Path(temporary)
        subprocess.run(["apt-get", "download", *packages], cwd=staging, check=True)
        for archive in staging.glob("*.deb"):
            subprocess.run(
                ["dpkg-deb", "-x", str(archive), str(staging / "unpacked")], check=True
            )
        source = staging / "unpacked/opt/ros" / distro
        shutil.copytree(source, prefix, symlinks=True)
    index = prefix / "share/colcon-core/packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / "foxglove_bridge").write_text("")
    hooks = [
        "prepend-non-duplicate;AMENT_PREFIX_PATH;",
        "prepend-non-duplicate;LD_LIBRARY_PATH;lib",
    ]
    hooks += [
        f"prepend-non-duplicate;LD_LIBRARY_PATH;{p.relative_to(prefix)}"
        for p in (prefix / "lib").glob("*-linux-gnu")
        if p.is_dir()
    ]
    (prefix / "share/foxglove_bridge/package.dsv").write_text("\n".join(hooks) + "\n")
    print(f"Installed {prefix}. Source install/setup.bash again before launching.")


if __name__ == "__main__":
    main()
