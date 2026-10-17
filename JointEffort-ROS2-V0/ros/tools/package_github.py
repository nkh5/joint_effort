#!/usr/bin/env python3
"""Build the source ZIP and its file list. This does not upload anything."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
SKIP = {'__pycache__', 'build', 'install', 'log', 'work', '.git'}
OMIT = {'source-inventory.json', 'export-recipe.json', '.DS_Store'}
GUIDES = (
    'README.md', '.gitignore', 'THIRD_PARTY.md',
    'docs/README.md', 'docs/SETUP.md', 'docs/CODE.md', 'docs/FILES.md',
    'iron_prototype/README.md', 'iron_prototype/DECISIONS.md',
    'iron_prototype/PICO-CONTROLLER-GUIDE.md',
    'iron_prototype/calibration/README.md',
    'iron_prototype/calibration/measurements-template.csv',
    'ros2_ws/README.md', 'ros2_ws/.gitignore', 'tools/package_github.py',
)
REPORTS = (
    'README.md', 'workspace-study.json', 'collision-endpoints.json',
    'joint-reference.csv', 'iron/workspace-study.json',
    'iron/collision-hulls.json', 'iron/collision-samples.json',
    'iron/proposed-motor-map.csv', 'iron/input-check.json',
    'iron/cycle-check.json', 'iron/calibration-check.json', 'iron/unit-tests.txt',
)
PURPOSES = {
    'README.md': 'Start here: build and run the iron-arm mock.',
    '.gitignore': 'Keep local build files and caches out of Git.',
    'THIRD_PARTY.md': 'Source credits and license notes.',
    'docs/README.md': 'Guide index.',
    'docs/SETUP.md': 'Install dependencies and build on another computer.',
    'docs/CODE.md': 'Explain the movement and input code.',
    'docs/FILES.md': 'List every bundled file and its purpose.',
    'tools/package_github.py': 'Build this source ZIP and check its contents.',
    'iron_prototype/README.md': 'Run, move, plan, cycle, and stop the mock arm.',
    'iron_prototype/DECISIONS.md': 'Record provisional model and motor settings.',
    'iron_prototype/PICO-CONTROLLER-GUIDE.md': 'Check the Pico and test joystick input.',
    'iron_prototype/calibration/README.md': 'Collect and fit joint measurements.',
    'iron_prototype/calibration/measurements-template.csv': 'Blank worksheet for real angle and encoder readings.',
    'ros2_ws/README.md': 'Workspace layout and start commands.',
    'ros2_ws/.gitignore': 'Ignore local workspace build files.',
    'ros2_ws/reports/README.md': 'Explain the saved studies and test results.',
    'ros2_ws/reports/workspace-study.json': 'Earlier dual-arm reach and five-axis study.',
    'ros2_ws/reports/collision-endpoints.json': 'Earlier collision results used by the plot tool.',
    'ros2_ws/reports/joint-reference.csv': 'Earlier dual-arm joint reference.',
    'ros2_ws/reports/iron/workspace-study.json': 'Iron-arm target study and checked hover candidates.',
    'ros2_ws/reports/iron/collision-hulls.json': 'Collision mesh sizes and simplification results.',
    'ros2_ws/reports/iron/collision-samples.json': 'Sampled iron-arm collision results.',
    'ros2_ws/reports/iron/proposed-motor-map.csv': 'Proposed motor IDs and joint conversion settings.',
    'ros2_ws/reports/iron/input-check.json': 'Synthetic joystick integration test results.',
    'ros2_ws/reports/iron/cycle-check.json': 'Cold-cycle movement and interruption test results.',
    'ros2_ws/reports/iron/calibration-check.json': 'Offline tests using made-up encoder readings.',
    'ros2_ws/reports/iron/unit-tests.txt': 'Python unit test results.',
}
TOOL_PURPOSES = {
    'check_input_live.py': 'Test synthetic input through the running mock.',
    'check_iron_calibration.py': 'Test the offline calibration tool.',
    'check_iron_cycle.py': 'Test cold cycles, stopping, and feedback loss.',
    'check_iron_demo.py': 'Check the visible iron-arm demo.',
    'check_iron_live.py': 'Check iron-arm controllers, motion, and collision handling.',
    'check_live.py': 'Check the earlier dual-arm mock.',
    'check_shutdown.py': 'Check that the earlier mock shuts down cleanly.',
    'demo_both.py': 'Move both earlier mock arms at once.',
    'demo_motion.py': 'Move one earlier mock arm.',
    'generate_assets.py': 'Generate the earlier dual-arm model and settings.',
    'generate_iron.py': 'Generate the current iron-arm model, fixtures, and collision hulls.',
    'input_demo.py': 'Send simulated joystick input without a controller.',
    'input_probe.py': 'Read joystick values and save their observed ranges.',
    'iron_calibration.py': 'Fit joint zero, direction, and external ratio from a CSV.',
    'iron_cycle.py': 'Preview or run the cold hover/lower/wait/retreat cycle.',
    'iron_demo.py': 'Run small iron-arm demonstration movements.',
    'iron_motion.py': 'Share motion checks, planning, feedback, and execution helpers.',
    'iron_plan.py': 'Preview or execute a checked planned movement.',
    'iron_status.py': 'Print iron-arm controller, feedback, and collision status.',
    'iron_stop.py': 'Request a stop from the project movement tools.',
    'iron_trajectory.py': 'Build and check timed joint trajectories.',
    'model_status.py': 'Print status for the earlier dual-arm mock.',
    'plot_study.py': 'Plot the earlier workspace and collision study.',
    'render_iron.py': 'Draw the iron-arm model and workspace study.',
    'review_cad_urdf.py': 'Inspect CAD joint frames, dimensions, and mesh bounds.',
    'study_iron_workspace.py': 'Find candidate iron-tip positions and pointing directions.',
    'study_workspace.py': 'Study reach and five-axis motion for the earlier model.',
}
SCRIPT_PURPOSES = {
    'with_ros.sh': 'Run a command in a clean ROS Jazzy environment.',
    'start_clean.sh': 'Load Jazzy, the local build, domain, and log folders.',
    'run_iron.sh': 'Start the current iron-arm mock on domain 88.',
    'run_input.sh': 'Start the joystick input nodes on domain 88.',
    'run_mock.sh': 'Start the earlier dual-arm mock on domain 87.',
}
TEST_PURPOSES = {
    'test_bridge.py': 'Test the earlier velocity bridge.',
    'test_input.py': 'Test joystick mapping and enable rules.',
    'test_input_forwarding.py': 'Test command timing, speed scaling, and forwarding.',
    'test_iron.py': 'Test iron-arm model geometry and settings.',
    'test_iron_bridge.py': 'Test the iron-arm velocity bridge.',
    'test_iron_workflows.py': 'Test planning, trajectories, and calibration helpers.',
    'test_kinematics.py': 'Test tip transforms, Jacobians, and motion limits.',
}
SOURCE_PURPOSES = {
    'kinematics.py': 'Calculate tip position, Jacobians, and limited joint speeds.',
    'task_bridge.py': 'Translate tip velocity into joint commands for the earlier mock.',
    'bridge.py': 'Send checked iron-arm commands to Servo and the mock controller.',
    'core.py': 'Map joystick input and enforce mode and enable rules.',
    'forwarder.py': 'Scale tip speed and reject stale or invalid commands.',
    'lease.py': 'Allow only one project movement tool to own the arm.',
    'display.py': 'Show input mode, speed, and errors in the terminal.',
    'synthetic.py': 'Provide simulated joystick messages for demos and tests.',
    'teleop.py': 'Start joy_teleop with the project shutdown handling.',
    'cad_rviz.cpp': 'Show the model and frame status in RViz.',
    'review_pose.py': 'Publish CAD review joint angles.',
    'review_sliders.py': 'Adjust CAD review joint angles with sliders.',
    'servo_lifetime.cpp': 'Stop Servo worker threads before closing the ROS context.',
    'move_group_lifetime.cpp': 'Run MoveIt with corrected shutdown object lifetimes.',
    'no_execution_controller_manager.cpp': 'Disable ordinary MoveIt execution in the mock launch.',
    'iron_collision_audit.cpp': 'Check model states and sampled paths for collisions.',
    'mount_clearance.cpp': 'Measure clearance around the modeled mount.',
    'controllers_plugin.xml': 'Register the MoveIt execution-blocking plugin.',
    'UPSTREAM.md': 'Explain the copied MoveIt source and local change.',
    'UPSTREAM.json': 'Record the vendored teleop source version.',
    'iron.urdf': 'Describe the current arm, fixtures, TCP, and mock hardware.',
    'joint_effort.urdf': 'Describe the earlier dual-arm mock.',
    'cad_arm.urdf': 'Describe the arm for isolated CAD review.',
    'joint_effort_arm_draft.urdf': 'Keep the imported CAD link and joint definitions.',
    'input.yaml': 'Set the joystick name, axes, buttons, speed, and timeouts.',
    'teleop.yaml': 'Map normalized joystick values to raw tip velocity.',
    'model.json': 'Store model dimensions and motion assumptions.',
    'controllers.yaml': 'Configure mock joint controllers and state publishing.',
    'joint_limits.yaml': 'Set joint speed and acceleration limits.',
    'kinematics.yaml': 'Select the MoveIt inverse-kinematics settings.',
    'servo.yaml': 'Configure iron-arm Servo topics, limits, and collision checks.',
    'iron_servo.yaml': 'Configure Servo for the earlier iron arm.',
    'wire_servo.yaml': 'Configure Servo for the earlier wire arm.',
}


def selected_files():
    paths = [ROOT/name for name in GUIDES]
    paths.extend(ROOT/'ros2_ws/reports'/name for name in REPORTS)
    for folder in ['src', 'scripts', 'tools', 'tests']:
        for path in (ROOT/'ros2_ws'/folder).rglob('*'):
            if (path.is_file() and not SKIP.intersection(path.relative_to(ROOT).parts)
                    and path.name not in OMIT and path.suffix not in {'.pyc', '.log'}):
                if path.is_symlink():
                    raise SystemExit('Review source symlink: ' + str(path))
                paths.append(path)
    return sorted(set(paths), key=lambda p: (p.parent.relative_to(ROOT).as_posix(), p.name))


def purpose(path):
    name = path.relative_to(ROOT).as_posix()
    if name in PURPOSES:
        return PURPOSES[name]
    if name.startswith('ros2_ws/tools/'):
        return TOOL_PURPOSES[path.name]
    if name.startswith('ros2_ws/scripts/'):
        return SCRIPT_PURPOSES[path.name]
    if name.startswith('ros2_ws/tests/'):
        return TEST_PURPOSES[path.name]
    vendor = '/vendor/' in name
    if vendor:
        if path.name.startswith('test_') or path.name == 'joy_teleop_testing_common.py':
            return 'Upstream joy_teleop test or shared test helper.'
        if path.name == 'joy_teleop.py':
            return 'Upstream joystick-to-message, service, and action mappings.'
        if path.name == 'incrementer_server.py':
            return 'Upstream example action server.'
        if path.name == 'Increment.action':
            return 'Upstream increment action message definition.'
        if path.name == 'joy_teleop_example.yaml':
            return 'Upstream example joystick configuration.'
        if path.name == 'example.launch.py':
            return 'Upstream example launch file.'
        if path.name == 'CHANGELOG.rst':
            return 'Upstream release history and credits.'
    if path.name in SOURCE_PURPOSES:
        return SOURCE_PURPOSES[path.name]
    if path.name == 'package.xml':
        return 'Declare the ROS package, dependencies, maintainer, and license.'
    if path.name == 'CMakeLists.txt':
        return 'Build and install this ROS package.'
    if path.name == 'setup.py':
        return 'Install the Python package, commands, and data files.'
    if path.name == 'setup.cfg':
        return 'Set the ROS Python installation folders.'
    if path.name == '__init__.py':
        return 'Mark this folder as a Python package.'
    if path.parent.name == 'resource':
        return 'Register the Python package in the ROS package index.'
    if path.name == 'README.md':
        return 'Upstream teleop package guide.' if vendor else 'Explain the imported CAD model and visual preview.'
    if path.suffix == '.stl':
        return 'Simplified collision shape for ' + path.stem.removesuffix('_hull') + '.' if path.stem.endswith('_hull') else 'CAD visual mesh for ' + path.stem + '.'
    if path.suffix == '.rviz':
        return 'Save the RViz model, frame, and display settings.'
    if path.suffix == '.srdf':
        return 'Define planning groups and permitted collision pairs.'
    if path.name == 'input.launch.py':
        return 'Start joystick mapping, forwarding, and terminal status nodes.'
    if path.name == 'mock.launch.py':
        return 'Start mock hardware, controllers, MoveIt, Servo, and RViz.'
    if path.name in {'display.launch.py', 'review.launch.py'}:
        return 'Start the isolated CAD visual preview.'
    raise SystemExit('Add a file description: ' + name)


def write_file_list(paths):
    lines = ['# Files in this bundle', '',
             'Paths are relative to the `ros` folder. Build files and local recordings are left out.', '']
    previous = None
    for path in paths:
        relative = path.relative_to(ROOT)
        parent = relative.parent.as_posix()
        if parent != previous:
            lines.extend(['', '## ' + (parent if parent != '.' else 'Top level'), '', '| File | Purpose |', '| --- | --- |'])
            previous = parent
        lines.append(f'| `{path.name}` | {purpose(path)} |')
    lines.extend(['', '## Generated file', '', '| File | Purpose |', '| --- | --- |', '| `MANIFEST.json` | Record file sizes and SHA-256 checksums. |', ''])
    (ROOT/'docs/FILES.md').write_text('\n'.join(lines))


def check_reports():
    for name in ['input-check.json', 'cycle-check.json', 'calibration-check.json']:
        report = json.loads((ROOT/'ros2_ws/reports/iron'/name).read_text())
        if report.get('result') != 'PASS':
            raise SystemExit('Run the failing check before packaging: ' + name)
    report = json.loads((ROOT/'ros2_ws/reports/iron/input-check.json').read_text())
    for name, expected in report['source_sha256'].items():
        if hashlib.sha256((ROOT/'ros2_ws'/name).read_bytes()).hexdigest() != expected:
            raise SystemExit('Input source changed; run check_input_live.py again: ' + name)


def main():
    check_reports()
    paths = selected_files()
    write_file_list(paths)
    contents = {p.relative_to(ROOT).as_posix(): p.read_bytes() for p in paths}
    manifest = {'files': {name: {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
                          for name, data in contents.items()}}
    contents['MANIFEST.json'] = (json.dumps(manifest, indent=2)+'\n').encode()
    release = ROOT/'releases'
    release.mkdir(exist_ok=True)
    archive = release/'JointEffort-ROS-GitHub.zip'
    temporary = archive.with_suffix('.zip.new')
    with zipfile.ZipFile(temporary, 'w', zipfile.ZIP_DEFLATED) as bundle:
        for name, data in contents.items():
            info = zipfile.ZipInfo('ros/' + name)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3
            mode = 0o755 if name != 'MANIFEST.json' and (ROOT/name).stat().st_mode & 0o111 else 0o644
            info.external_attr = (0o100000 | mode) << 16
            bundle.writestr(info, data)
    with zipfile.ZipFile(temporary) as bundle:
        assert bundle.testzip() is None, 'ZIP integrity failed'
        assert set(bundle.namelist()) == {'ros/'+name for name in contents}
        for name, data in contents.items():
            assert bundle.read('ros/'+name) == data, name
    temporary.replace(archive)
    archive.with_suffix('.sha256').write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n')
    print(f'Created {archive.name}: {len(contents)} files, {archive.stat().st_size/1e6:.2f} MB')


if __name__ == '__main__':
    main()
