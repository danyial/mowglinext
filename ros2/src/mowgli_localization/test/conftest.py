"""
pytest conftest for mowgli_localization Python tests.

The package is `ament_cmake` (not `ament_python`), so there is no
setup.py to put `scripts/` on PYTHONPATH at install time. For colcon
test (and for host-side `pytest --collect-only`) we prepend the
in-source `scripts/` directory to sys.path so test files can do
`import dock_scan_capture` (and other sibling helpers later).

This is a test-time only path injection; runtime ROS2 nodes are
launched by `ros2 run mowgli_localization <script>.py` which uses the
file already installed under `lib/mowgli_localization/` and works
without the path manipulation.
"""

import os
import sys

_SCRIPTS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir, "scripts")
)
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)
