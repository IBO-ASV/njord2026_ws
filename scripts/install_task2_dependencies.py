#!/usr/bin/env python3
"""Install CPU PyTorch persistently for the Python interpreter used by ROS.

An existing unpacked CPU environment can be migrated without downloading again.
No ROS launch depends on XDG_RUNTIME_DIR or a manually exported PYTHONPATH.
"""

import argparse
import importlib.util
import os
import shutil
import site
import subprocess
import sys
import tempfile
from pathlib import Path

PACKAGES = (
    "torch",
    "functorch",
    "torchgen",
    "typing_extensions.py",
    "sympy",
    "mpmath",
    "networkx",
    "filelock",
    "fsspec",
    "jinja2",
    "markupsafe",
    "setuptools",
    "pkg_resources",
    "_distutils_hack",
    "distutils-precedence.pth",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--from-directory",
        type=Path,
        help="reuse an already installed/unpacked CPU PyTorch environment",
    )
    args = parser.parse_args()
    if importlib.util.find_spec("torch") is not None:
        subprocess.run(
            [
                sys.executable,
                "-c",
                "import torch; print(torch.__version__, torch.__file__)",
            ],
            check=True,
        )
        return
    if not site.ENABLE_USER_SITE:
        parser.error(
            "Python user site is disabled; use the ROS interpreter with user-site enabled"
        )
    destination = Path(site.getusersitepackages())
    if args.from_directory:
        source = args.from_directory.resolve()
        if not (source / "torch/__init__.py").is_file():
            parser.error("source must contain an installed torch package")
        entries = [source / name for name in PACKAGES if (source / name).exists()]
        names = {
            name.removesuffix(".py").lower().replace("_", "-") for name in PACKAGES
        }
        entries += [
            p
            for p in source.glob("*.dist-info")
            if p.name.split("-")[0].lower().replace("_", "-") in names
        ]
        destination.parent.mkdir(parents=True, exist_ok=True)
        required = sum(
            p.stat().st_size
            for entry in entries
            for p in (entry.rglob("*") if entry.is_dir() else [entry])
            if p.is_file()
        )
        if shutil.disk_usage(destination.parent).free < required + 128 * 1024**2:
            parser.error(
                f"need {required / 1024**2:.0f} MiB plus 128 MiB free for a persistent install"
            )
        destination.mkdir(exist_ok=True)
        collisions = [p.name for p in entries if (destination / p.name).exists()]
        if collisions:
            parser.error(f"refusing to overwrite existing user packages: {collisions}")
        with tempfile.TemporaryDirectory(
            prefix="njord-torch-", dir=destination.parent
        ) as temporary:
            staging = Path(temporary)
            for entry in entries:
                if entry.is_dir():
                    shutil.copytree(entry, staging / entry.name, symlinks=True)
                else:
                    shutil.copy2(entry, staging / entry.name)
            for entry in entries:
                os.replace(staging / entry.name, destination / entry.name)
    else:
        if importlib.util.find_spec("pip") is None:
            parser.error(
                "pip is unavailable; install python3-pip or use --from-directory with an existing CPU installation"
            )
        subprocess.run(
            [
                sys.executable,
                "-m",
                "pip",
                "install",
                "--user",
                "--break-system-packages",
                "--index-url",
                "https://download.pytorch.org/whl/cpu",
                "torch",
            ],
            check=True,
        )
    subprocess.run(
        [
            sys.executable,
            "-c",
            'import torch; print("CPU PyTorch ready:", torch.__version__, torch.__file__); '
            "print((torch.ones(2) + 1).tolist())",
        ],
        check=True,
    )
    print("Task2 dependencies persist across reboot; no PYTHONPATH export is needed.")


if __name__ == "__main__":
    main()
