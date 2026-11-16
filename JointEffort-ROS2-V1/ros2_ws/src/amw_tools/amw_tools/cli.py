"""amw - one command for the AMW arm. Run `amw <command> --help` for details.

  amw build [--clean]                 fetch + patch the motor driver, build the workspace
  amw start mock|real [options]       start the arm (and joystick, Servo, RViz if a display exists)
  amw status [--once]                 why is it (not) moving? everything on one screen
  amw stop [--estop]                  turn the arm OFF  (--estop: torque off on every motor, arm goes limp)
  amw calibrate zero|check|limits     set zeros, check directions, measure travel (ROS stopped)
  amw jog JOINT DEG                   move one joint a few degrees (needs `amw start ... --no-teleop`)
  amw track JOINT|all                 jog sequence with recording and plots
  amw motors scan|read|gains|tune|punch|set-id|recenter   servo bus tools (ROS stopped)
  amw joystick                        print raw Joy-Con input
  amw lcd [--demo|--fb]               status screen (desktop window, or the Pi's framebuffer)
  amw test                            unit tests (no ROS or hardware needed)
  amw ros2 ...                        any ros2 command in the AMW environment (domain 88)
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

from .common import ARM_YAML, WS

DRIVER_URL = 'https://github.com/adityakamath/sts_hardware_interface.git'
DRIVER_COMMIT = 'eeb0ad0'
DRIVER_DIR = WS / 'src/sts_hardware_interface'
DRIVER_PATCH = WS / 'driver/amw_driver.patch'
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
DOMAIN = os.environ.get('AMW_DOMAIN_ID', '88')
# Environment variables that belong to one ROS install; dropped when switching to Jazzy.
DISTRO_VARS = ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'PYTHONPATH', 'LD_LIBRARY_PATH',
               'ROS_DISTRO', 'ROS_VERSION', 'ROS_PYTHON_VERSION', 'ROS_PACKAGE_PATH')
NEEDS_ROS = {'build', 'start', 'status', 'stop', 'jog', 'track', 'joystick', 'lcd', 'ros2'}
NEEDS_INSTALL = NEEDS_ROS - {'build'}


# ------------------------------------------------------------------ environment

def ensure_ros(command):
    """Re-run this command with Jazzy and the workspace sourced (once)."""
    if os.environ.get('AMW_ENV') == '1':
        return
    if not Path(ROS_SETUP).exists():
        raise SystemExit(f'ROS 2 Jazzy not found ({ROS_SETUP}). See docs/setup.md.')
    if command in NEEDS_INSTALL and not (WS / 'install/setup.bash').exists():
        raise SystemExit('The workspace is not built yet. Run: amw build')
    env = dict(os.environ)
    if env.get('ROS_DISTRO') and env['ROS_DISTRO'] != 'jazzy':
        for key in DISTRO_VARS:
            env.pop(key, None)
    env.update(AMW_ENV='1', AMW_WS=str(WS), ROS_DOMAIN_ID=DOMAIN)
    if os.environ.get('ROS_DOMAIN_ID') not in (None, DOMAIN):
        print(f'[amw] using ROS_DOMAIN_ID={DOMAIN} (your shell has {os.environ["ROS_DOMAIN_ID"]}; '
              f'bare `ros2` commands there will not see the arm - use `amw ros2 ...`)', file=sys.stderr)
    script = (f'source {ROS_SETUP} && '
              f'if [ -f "{WS}/install/setup.bash" ]; then source "{WS}/install/setup.bash"; fi && exec "$@"')
    os.execvpe('bash', ['bash', '-c', script, 'amw', 'python3', str(WS / 'amw'), *sys.argv[1:]], env)


def run(cmd, **kw):
    print('[amw] ' + ' '.join(str(c) for c in cmd), flush=True)
    return subprocess.call([str(c) for c in cmd], **kw)


# ------------------------------------------------------------------ commands

def cmd_build(a):
    if a.clean:
        for d in ('build', 'install', 'log'):
            if (WS / d).exists():
                print(f'[amw] removing {d}/')
                shutil.rmtree(WS / d)
    if not (DRIVER_DIR / '.git').exists():
        if run(['git', 'clone', DRIVER_URL, DRIVER_DIR]):
            raise SystemExit('Could not download the motor driver (network?).')
    have = subprocess.run(['git', '-C', DRIVER_DIR, 'cat-file', '-e', DRIVER_COMMIT + '^{commit}'],
                          capture_output=True).returncode == 0
    if not have:
        run(['git', '-C', DRIVER_DIR, 'fetch', '--quiet', 'origin'])
    # The driver folder is managed here: reset it to the pinned commit and apply our one patch.
    if run(['git', '-C', DRIVER_DIR, 'checkout', '--quiet', '-f', DRIVER_COMMIT]):
        raise SystemExit(f'Could not check out driver commit {DRIVER_COMMIT}.')
    if run(['git', '-C', DRIVER_DIR, 'apply', DRIVER_PATCH]):
        raise SystemExit('The driver patch did not apply. Delete src/sts_hardware_interface and run amw build again.')
    cmd = ['colcon', 'build', '--symlink-install', '--base-paths', 'src', '--parallel-workers', str(a.workers)]
    if a.packages:
        cmd += ['--packages-select', *a.packages]
    cmd += ['--cmake-args', '-DCMAKE_BUILD_TYPE=Release']      # must stay last: colcon takes everything after it
    env = dict(os.environ, CMAKE_BUILD_PARALLEL_LEVEL=str(a.workers))
    return run(cmd, cwd=WS, env=env)


def cmd_start(a):
    args = ['ros2', 'launch', 'amw_bringup', 'amw.launch.py', f'hardware:={a.hardware}',
            f'arm_config:={a.arm or ARM_YAML}']
    if a.readonly:
        if a.hardware != 'real':
            raise SystemExit('--readonly only makes sense with `amw start real`')
        args.append('readonly:=true')
    if a.no_teleop:
        args.append('teleop:=false')
    if a.no_joystick:
        args.append('joystick:=false')
    if a.rviz is not None:
        args.append(f'rviz:={str(a.rviz).lower()}')
    if a.port:
        args.append(f'port:={a.port}')
    print('[amw] ' + ' '.join(args), flush=True)
    os.execvp('ros2', args)


def cmd_stop(a):
    code = run(['ros2', 'service', 'call', '/amw/stop', 'std_srvs/srv/Trigger', '{}'])
    if a.estop:
        print('[amw] EMERGENCY STOP: torque off on every motor. The arm goes limp - support it.')
        code = run(['ros2', 'service', 'call', '/emergency_stop', 'std_srvs/srv/SetBool', '{data: true}']) or code
        print('[amw] Do not release the e-stop with data:false (the arm would jump back to its old goal). '
              'Ctrl-C `amw start` instead.')
    return code


def cmd_calibrate(a):
    from . import calibrate
    if a.what == 'zero':
        return calibrate.zero(a.joints, a.yes)
    if a.what == 'check':
        return calibrate.check(a.joints)
    return calibrate.limits(a.joints, a.margin)


def cmd_motors(a):
    from . import motors
    w = a.what
    if w == 'scan':
        return motors.scan(a.max, a.port)
    if w == 'read':
        return motors.read(a.port)
    if w == 'gains':
        return motors.gains(a.port)
    if w == 'tune':
        return motors.tune(a.port, a.yes)
    if w == 'punch':
        if len(a.args) != 2:
            raise SystemExit('usage: amw motors punch JOINT|all VALUE')
        return motors.punch(a.args[0], int(a.args[1]), a.port)
    if w == 'set-id':
        if len(a.args) != 2:
            raise SystemExit('usage: amw motors set-id OLD NEW')
        return motors.set_id(int(a.args[0]), int(a.args[1]), a.port)
    if w == 'recenter':
        if len(a.args) != 1:
            raise SystemExit('usage: amw motors recenter JOINT')
        return motors.recenter(a.args[0], a.port, a.yes)
    raise SystemExit(f'unknown motors command {w}')


def cmd_test(a):
    return run([sys.executable, '-m', 'unittest', 'discover', '-s', WS / 'tests', '-t', WS / 'tests',
                *(['-v'] if a.verbose else [])], cwd=WS)


def main(argv):
    p = argparse.ArgumentParser(prog='amw', description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='command', metavar='command')

    b = sub.add_parser('build', help='fetch + patch the driver, colcon build')
    b.add_argument('--clean', action='store_true', help='delete build/ install/ log/ first (do this once after switching branches)')
    b.add_argument('--workers', type=int, default=2, help='parallel compile jobs (default 2; use 1 if memory runs out)')
    b.add_argument('packages', nargs='*', help='only these packages')

    s = sub.add_parser('start', help='start the arm')
    s.add_argument('hardware', choices=['mock', 'real'])
    s.add_argument('--readonly', action='store_true', help='real arm with torque OFF (calibration and checks)')
    s.add_argument('--no-teleop', action='store_true', help='controllers only, for amw jog / amw track')
    s.add_argument('--no-joystick', action='store_true', help='teleop without the Joy-Con node (for a synthetic /joy)')
    s.add_argument('--rviz', dest='rviz', action='store_true', default=None, help='force RViz on')
    s.add_argument('--no-rviz', dest='rviz', action='store_false', help='force RViz off')
    s.add_argument('--port', help='servo adapter (default: serial_port in arm.yaml)')
    s.add_argument('--arm', help='arm.yaml to use (default: the one in the source tree)')

    st = sub.add_parser('status', help='live status screen')
    st.add_argument('--once', action='store_true')

    so = sub.add_parser('stop', help='turn the arm OFF')
    so.add_argument('--estop', action='store_true', help='also torque OFF on every motor (arm goes limp)')

    c = sub.add_parser('calibrate', help='zero / direction / limits (ROS stopped, torque off)')
    c.add_argument('what', choices=['zero', 'check', 'limits'])
    c.add_argument('joints', nargs='*', help='default: all five')
    c.add_argument('--yes', action='store_true', help='zero: do not ask')
    c.add_argument('--margin', type=float, default=2.0, help='limits: degrees to stay inside what you reached')

    j = sub.add_parser('jog', help='move one joint a few degrees', add_help=False)
    j.add_argument('rest', nargs=argparse.REMAINDER)
    t = sub.add_parser('track', help='recorded jog sequence with plots', add_help=False)
    t.add_argument('rest', nargs=argparse.REMAINDER)

    m = sub.add_parser('motors', help='servo bus tools (ROS stopped)')
    m.add_argument('what', choices=['scan', 'read', 'gains', 'tune', 'punch', 'set-id', 'recenter'])
    m.add_argument('args', nargs='*')
    m.add_argument('--port', help='servo adapter (default: serial_port in arm.yaml)')
    m.add_argument('--max', type=int, default=20, help='scan: highest ID to try')
    m.add_argument('--yes', action='store_true')

    sub.add_parser('joystick', help='print raw Joy-Con input')
    lcd = sub.add_parser('lcd', help='status screen', add_help=False)
    lcd.add_argument('rest', nargs=argparse.REMAINDER)
    te = sub.add_parser('test', help='unit tests')
    te.add_argument('-v', '--verbose', action='store_true')
    r = sub.add_parser('ros2', help='run a ros2 command in the AMW environment', add_help=False)
    r.add_argument('rest', nargs=argparse.REMAINDER)

    a = p.parse_args(argv)
    if not a.command:
        p.print_help()
        return 1
    if a.command in NEEDS_ROS:
        ensure_ros(a.command)

    if a.command == 'build':
        return cmd_build(a)
    if a.command == 'start':
        return cmd_start(a)
    if a.command == 'status':
        from .status import main as status_main
        return status_main(a.once)
    if a.command == 'stop':
        return cmd_stop(a)
    if a.command == 'calibrate':
        return cmd_calibrate(a)
    if a.command == 'jog':
        from .jog import main as jog_main
        return jog_main(a.rest)
    if a.command == 'track':
        from .track import main as track_main
        return track_main(a.rest)
    if a.command == 'motors':
        return cmd_motors(a)
    if a.command == 'joystick':
        from .joystick import main as joystick_main
        return joystick_main()
    if a.command == 'lcd':
        os.execvp('ros2', ['ros2', 'run', 'amw_teleop', 'amw_lcd', *a.rest])
    if a.command == 'test':
        return cmd_test(a)
    if a.command == 'ros2':
        os.execvp('ros2', ['ros2', *a.rest])
    return 1
