"""Make the task2_perception package importable without installation.

The library modules (cloud_ops, wall_fit, tracking_glue, smoothing) are pure
numpy (no rclpy), so these tests run on any machine with
`python3 -m pytest src/detection/task2_perception/test -q`.
"""

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
