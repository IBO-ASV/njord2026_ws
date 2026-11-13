#!/usr/bin/env python3
"""Correct NVIDIA cuSPARSELt 0.8.0 ARM wheel's nonstandard `sbsa` tag.

Only installed metadata is changed, after verifying an ELF AArch64 library.
Atomic replacement breaks uv cache hardlinks; the cached vendor wheel and all
shared libraries remain unchanged. The RECORD hash is updated as well.
"""

import base64
import csv
import hashlib
import importlib.metadata
import io
import os
import platform
import struct
import tempfile
from pathlib import Path


def replace_file(path, data):
    descriptor, name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
        os.chmod(name, path.stat().st_mode & 0o777)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def normalize(root):
    root = Path(root)
    info = root / "nvidia_cusparselt_cu13-0.8.0.dist-info"
    wheel = info / "WHEEL"
    if not wheel.exists():
        return False
    before = wheel.read_text()
    tag = "Tag: py3-none-manylinux2014_sbsa"
    if tag not in before:
        return False
    library = root / "nvidia/cusparselt/lib/libcusparseLt.so.0"
    with library.open("rb") as stream:
        header = stream.read(20)
    if (
        len(header) != 20
        or header[:6] != b"\x7fELF\x02\x01"
        or struct.unpack_from("<H", header, 18)[0] != 183
    ):
        raise RuntimeError("Refusing tag correction: cuSPARSELt is not ELF64 AArch64")
    after = before.replace(tag, "Tag: py3-none-manylinux2014_aarch64").encode()
    record = info / "RECORD"
    rows = list(csv.reader(io.StringIO(record.read_text())))
    relative = str(wheel.relative_to(root))
    digest = (
        base64.urlsafe_b64encode(hashlib.sha256(after).digest()).rstrip(b"=").decode()
    )
    matching = [row for row in rows if row[0] == relative]
    if len(matching) != 1:
        raise RuntimeError(
            "Refusing tag correction: WHEEL is absent/ambiguous in RECORD"
        )
    matching[0][1:] = [f"sha256={digest}", str(len(after))]
    output = io.StringIO()
    csv.writer(output, lineterminator="\n").writerows(rows)
    replace_file(wheel, after)
    replace_file(record, output.getvalue().encode())
    return True


def main():
    if platform.machine() != "aarch64":
        return
    try:
        distribution = importlib.metadata.distribution("nvidia-cusparselt-cu13")
    except importlib.metadata.PackageNotFoundError:
        return
    if distribution.version == "0.8.0" and normalize(distribution.locate_file("")):
        print("Corrected cuSPARSELt 0.8.0 ARM64 wheel metadata; library unchanged.")


if __name__ == "__main__":
    main()
