"""Make the workspace packages importable for the tests (no build or ROS needed)."""
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[1]
for pkg in ('amw_bringup', 'amw_teleop', 'amw_tools'):
    path = str(WS / 'src' / pkg)
    if path not in sys.path:
        sys.path.insert(0, path)
sys.path.insert(0, str(WS / 'tests'))
