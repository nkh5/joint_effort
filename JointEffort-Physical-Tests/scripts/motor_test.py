#!/usr/bin/env python3
"""Run the physical motor test from this folder without building ROS."""
import os
from pathlib import Path
import sys
workspace = Path(__file__).resolve().parents[1]/'ros2_ws'
sys.path.insert(0, str(workspace/'src/joint_effort_hardware'))
os.chdir(workspace)
from joint_effort_hardware.commission import main
raise SystemExit(main())
