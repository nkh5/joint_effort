#!/usr/bin/env python3
"""Build the running iron model from the R02 source. Does not edit calibration or tuning."""
import copy
import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

W = Path(__file__).resolve().parents[1]
P = W / 'src/joint_effort_iron'
SOURCE = W / 'src/joint_effort_arm_calibration_description'
NAMES = ['base_rotation_joint', 'shoulder_joint', 'elbow_joint', 'wrist_yaw_joint', 'wrist_pitch_joint']
DTYPE = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])


def main():
    source = SOURCE / 'urdf/joint_effort_arm_calibration.urdf'
    root = ET.parse(source).getroot()
    old = ET.parse(P / 'urdf/iron.urdf').getroot()
    model = json.loads((P / 'config/model.json').read_text())
    root.set('name', 'joint_effort_iron')
    for i, name in enumerate(NAMES, 1):
        joint = root.find(f"joint[@name='{name}']")
        joint.set('name', f'joint_{i}')
        # These are software test bounds, not measured stops or a motor torque rating.
        joint.find('limit').set('velocity', '.15')
        joint.find('limit').set('effort', '.49')
    reports = []
    for link in root.findall('link'):
        mesh = link.find('visual/geometry/mesh')
        if mesh is None:
            continue
        path = SOURCE / 'meshes' / Path(mesh.get('filename')).name
        raw = path.read_bytes()
        count = struct.unpack('<I', raw[80:84])[0]
        points = np.unique(np.frombuffer(raw, dtype=DTYPE, count=count, offset=84)['v'].reshape(-1, 3), axis=0)
        hull = ConvexHull(points)
        triangles = points[hull.simplices].copy()
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        flip = np.sum(normals * hull.equations[:, :3], axis=1) < 0
        triangles[flip] = triangles[flip][:, [0, 2, 1]]
        normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        payload = np.zeros(len(triangles), dtype=DTYPE)
        payload['normal'], payload['v'] = normals, triangles
        dest = P / 'meshes' / (link.get('name') + '_hull.stl')
        dest.write_bytes(b'Joint Effort R02 convex envelope'.ljust(80, b'\0') + struct.pack('<I', len(triangles)) + payload.tobytes())
        # Keep the R02 collision origin, including the fixed base's clocking correction.
        link.find('collision/geometry/mesh').set('filename', f'package://joint_effort_iron/meshes/{dest.name}')
        error = max(float(np.max(points @ e[:3] + e[3])) for e in hull.equations)
        assert error < 1e-7
        reports.append(dict(link=link.get('name'), visual_triangles=count, hull_triangles=len(triangles),
                            max_vertex_outside_hull_m=error, collision_sha256=hashlib.sha256(dest.read_bytes()).hexdigest()))
    # Reuse the provisional table and board sizes. World now agrees with the R02 base axes.
    for name in ['world', 'table', 'pcb_fixture', 'pcb']:
        root.append(copy.deepcopy(old.find(f"link[@name='{name}']")))
    for name in ['iron_mount', 'table_fixed', 'pcb_fixture_fixed', 'pcb_fixed']:
        root.append(copy.deepcopy(old.find(f"joint[@name='{name}']")))
    root.find("joint[@name='iron_mount']/origin").set('rpy', '0 0 0')
    ET.SubElement(root, 'link', name='iron_tcp')
    tcp = ET.SubElement(root, 'joint', name='iron_tcp_axis', type='fixed')
    ET.SubElement(tcp, 'parent', link='iron_tip')
    ET.SubElement(tcp, 'child', link='iron_tcp')
    # R02 iron_tip +Z follows the shaft toward the tip. The solver frees rotation about +X.
    ET.SubElement(tcp, 'origin', xyz='0 0 0', rpy=f'0 {-math.pi/2} 0')
    control = ET.SubElement(root, 'ros2_control', name='IronR02Mock', type='system')
    ET.SubElement(ET.SubElement(control, 'hardware'), 'plugin').text = 'mock_components/GenericSystem'
    initial = [0., -.20, .20, 0., .15]
    for i, q in enumerate(initial, 1):
        joint = ET.SubElement(control, 'joint', name=f'joint_{i}')
        ET.SubElement(joint, 'command_interface', name='position')
        state = ET.SubElement(joint, 'state_interface', name='position')
        ET.SubElement(state, 'param', name='initial_value').text = str(q)
        ET.SubElement(joint, 'state_interface', name='velocity')
    ET.indent(root)
    ET.ElementTree(root).write(P / 'urdf/iron.urdf', encoding='utf-8', xml_declaration=True)
    srdf = ET.parse(P / 'config/iron.srdf').getroot()
    for state in srdf.findall('group_state'):
        for joint, q in zip(state.findall('joint'), initial):
            joint.set('value', str(q))
    ET.indent(srdf)
    ET.ElementTree(srdf).write(P / 'config/iron.srdf', encoding='utf-8', xml_declaration=True)
    model.update(model_revision='R02', scope='R02 geometry; mock tested separately from physical calibration.',
                 source_urdf_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                 base_yaw_rad=0., initial_positions_rad=initial,
                 joint_name_map={f'joint_{i}': name for i, name in enumerate(NAMES, 1)},
                 calibration_confirmed=False, tcp_physically_measured=False,
                 travel_limits_confirmed=False, joint_limits_deg=[-30, 30])
    (P / 'config/model.json').write_text(json.dumps(model, indent=2) + '\n')
    out = W / 'reports/r02'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'collision-hulls.json').write_text(json.dumps(reports, indent=2) + '\n')
    print('Generated R02 runtime model and six conservative collision envelopes.')


if __name__ == '__main__':
    main()
