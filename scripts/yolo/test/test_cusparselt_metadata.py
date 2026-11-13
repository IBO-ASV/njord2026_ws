import base64
import csv
import hashlib
import importlib.util
import os
import struct
from pathlib import Path

import pytest

PATH = Path(__file__).resolve().parents[1] / "normalize_cusparselt_metadata.py"
spec = importlib.util.spec_from_file_location("normalize_cusparselt", PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def installation(tmp_path, machine):
    info = tmp_path / "nvidia_cusparselt_cu13-0.8.0.dist-info"
    info.mkdir()
    wheel = info / "WHEEL"
    wheel.write_text("Wheel-Version: 1.0\nTag: py3-none-manylinux2014_sbsa\n")
    (info / "RECORD").write_text(str(wheel.relative_to(tmp_path)) + ",,\n")
    library = tmp_path / "nvidia/cusparselt/lib/libcusparseLt.so.0"
    library.parent.mkdir(parents=True)
    header = bytearray(20)
    header[:6] = b"\x7fELF\x02\x01"
    struct.pack_into("<H", header, 18, machine)
    library.write_bytes(header)
    return wheel, info / "RECORD"


def test_correction_breaks_cache_hardlinks_and_updates_integrity_record(tmp_path):
    wheel, record = installation(tmp_path, 183)
    cached = tmp_path / "cached-wheel-metadata"
    os.link(wheel, cached)
    original = cached.read_bytes()
    assert module.normalize(tmp_path)
    assert cached.read_bytes() == original
    assert wheel.read_bytes() != original
    assert "aarch64" in wheel.read_text()
    row = next(csv.reader(record.read_text().splitlines()))
    digest = (
        base64.urlsafe_b64encode(hashlib.sha256(wheel.read_bytes()).digest())
        .rstrip(b"=")
        .decode()
    )
    assert row[1:] == [f"sha256={digest}", str(wheel.stat().st_size)]
    assert not module.normalize(tmp_path)


def test_wrong_architecture_is_never_relabelled(tmp_path):
    wheel, _ = installation(tmp_path, 62)
    original = wheel.read_bytes()
    with pytest.raises(RuntimeError, match="not ELF64 AArch64"):
        module.normalize(tmp_path)
    assert wheel.read_bytes() == original
