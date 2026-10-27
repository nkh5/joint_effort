"""Datum measurement: drag each joint of the RViz model onto your physical 0 deg mark.

Visual only. Publishes /joint_states for robot_state_publisher; never talks to motors.
Started by scripts/run_datum.sh (launch/datum.launch.py). For each joint, drag its slider until
the RViz link sits on the same reference you used for Calibration.csv (link parallel to the
motor's long side). Then press "Print YAML" (or close the window) and copy the datum_offset_rad
lines into sts_calibration.yaml.
"""
import math
import signal
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
ROLES = ['base', 'shoulder', 'elbow', 'wrist 1', 'wrist 2']
STEP = 0.1                     # degrees per slider tick


def home_positions(urdf):
    """Mock initial_value of each joint (the pose RViz starts in), default 0."""
    root = ET.parse(urdf).getroot()
    out = {n: 0.0 for n in JOINTS}
    for j in root.findall('ros2_control/joint'):
        p = j.find("state_interface[@name='position']/param[@name='initial_value']")
        if j.get('name') in out and p is not None:
            out[j.get('name')] = float(p.text)
    return out


def yaml_lines(values):
    table = '\n'.join(f'  {n}: {values[n]:+.4f} rad  ({math.degrees(values[n]):+.1f} deg)' for n in JOINTS)
    command = 'python3 tools/fit_calibration.py --write --datum ' + ' '.join(
        f'{n}={values[n]:+.4f}' for n in JOINTS)
    return f'{table}\n\nRun from ros2_ws to store them:\n  {command}'


def main():
    from PyQt5.QtCore import Qt, QTimer
    from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QPushButton, QSlider,
                                 QVBoxLayout, QWidget)
    rclpy.init()
    node = Node('iron_datum_sliders')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    urdf = Path(get_package_share_directory('joint_effort_iron')) / 'urdf/iron.urdf'
    values = home_positions(urdf)

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    window = QWidget()
    window.setWindowTitle('Iron datum: drag each link onto your 0 deg mark (visual only)')
    layout = QVBoxLayout(window)
    layout.addWidget(QLabel('Sliders move the RViz model only. Range -180..180 deg (ignores URDF limits).'))
    labels = {}
    sliders = {}
    for name, role in zip(JOINTS, ROLES):
        row = QHBoxLayout()
        label = QLabel()
        label.setMinimumWidth(260)
        slider = QSlider(Qt.Horizontal)
        slider.setRange(int(-180 / STEP), int(180 / STEP))
        slider.setValue(int(round(math.degrees(values[name]) / STEP)))

        def changed(v, n=name, lab=label, r=role):
            values[n] = math.radians(v * STEP)
            lab.setText(f'{n} ({r}): {values[n]:+.4f} rad  {math.degrees(values[n]):+.1f} deg')
        changed(slider.value())
        slider.valueChanged.connect(changed)
        row.addWidget(label)
        row.addWidget(slider)
        layout.addLayout(row)
        labels[name], sliders[name] = label, slider
    printer = QPushButton('Print YAML (datum_offset_rad for sts_calibration.yaml)')
    printer.clicked.connect(lambda: print('\n' + yaml_lines(values), flush=True))
    layout.addWidget(printer)

    def tick():
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [values[n] for n in JOINTS]
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.0)
    timer = QTimer()
    timer.timeout.connect(tick)
    timer.start(50)
    window.resize(820, 260)
    window.show()
    try:
        app.exec_()
    finally:
        print('\nDatum values (URDF angle of each physical 0 deg mark):\n' + yaml_lines(values), flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
