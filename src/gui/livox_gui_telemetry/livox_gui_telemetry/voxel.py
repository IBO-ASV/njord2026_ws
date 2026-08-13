"""Small dependency-free PointCloud2 helpers shared by the GUI nodes."""
import math
import random
import struct


def voxelize(points, size, limit):
    """Keep one randomly selected representative per voxel, bounded by limit."""
    cells = {}
    for point in points:
        x, y, z, _ = point
        key = (math.floor(x / size), math.floor(y / size), math.floor(z / size))
        if key not in cells or random.randrange(2) == 0:
            cells[key] = point
    result = list(cells.values())
    return result if len(result) <= limit else random.sample(result, limit)


def cloud_points(message):
    """Read x/y/z and optional intensity from common float32 PointCloud2 layouts."""
    fields = {field.name: field.offset for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields) or message.point_step < 12:
        return []
    endian = ">" if message.is_bigendian else "<"
    intensity = fields.get("intensity")
    data = memoryview(message.data)
    points = []
    for offset in range(0, len(data) - message.point_step + 1, message.point_step):
        x, y, z = struct.unpack_from(endian + "fff", data, offset + fields["x"])
        value = struct.unpack_from(endian + "f", data, offset + intensity)[0] if intensity is not None else 0.0
        if all(math.isfinite(v) for v in (x, y, z, value)):
            points.append((x, y, z, value))
    return points
