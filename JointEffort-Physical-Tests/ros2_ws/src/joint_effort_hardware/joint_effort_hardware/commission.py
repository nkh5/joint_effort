"""Three-motor tests: hold base ID 1 stationary; move shoulder ID 2 or elbow ID 3."""
import argparse
import json
import math
from pathlib import Path
import time

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028914-if00'
IDS = {'base': 1, 'shoulder': 2, 'elbow': 3}
MOVABLE = {'shoulder': 2, 'elbow': 3}
MOTOR_IDS = tuple(IDS.values())
PROFILE = 'three-motor-fixed-base-v1'
HOLD_TOLERANCE = 2              # Encoder counts; never retarget a drifting base.
SPAN = 8                         # About 0.70 degrees at the motor shaft.
SPEED = 10                       # Nonzero; phase 12 selects counts/second.
FEEDBACK_DEADLINE = .015         # Complete three-motor read budget.
PERIOD = .02
MOVE_INTERVAL = .1
SETTLE_SECONDS = .5
OBSERVE_SECONDS = 1.
TURN_COUNTS = 116                # 10.20 degrees requested; allow two counts of final error.
TURN_MINIMUM = 114               # 10.02 degrees of measured shaft movement.
TURN_WINDOW = 118


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def word(data, offset=0):
    return data[offset] | data[offset+1] << 8


def frame(ident, opcode, parameters):
    require(type(ident) is int and ident in MOTOR_IDS, 'Only motor IDs 1, 2 and 3 are allowed')
    body = bytes([ident, len(parameters)+2, opcode, *parameters])
    return b'\xff\xff'+body+bytes([~sum(body) & 255])


class Bus:
    def __init__(self, port):
        self.port = port
        self.selected = None
        self.bounds = None
        self.hold_target = None
        self.healthy = True
        self.writes = []

    def exchange(self, ident, opcode, parameters, count):
        require(self.healthy, 'Serial fault is latched; start a new test')
        try:
            require(not self.port.in_waiting, 'Unexpected serial bytes; another reply or process may be present')
            tx = frame(ident, opcode, parameters)
            require(self.port.write(tx) == len(tx), 'Incomplete serial write')
            self.port.flush()
            raw = bytearray()
            deadline = time.monotonic()+.012
            while len(raw) < count+6 and time.monotonic() < deadline:
                raw.extend(self.port.read(count+6-len(raw)))
            require(len(raw) == count+6, 'Motor response deadline missed')
            require(raw[:2] == b'\xff\xff' and raw[2] == ident and raw[3] == count+2,
                    'Invalid motor response header/ID/length')
            require(sum(raw[2:]) & 255 == 255, 'Motor response checksum failed')
            require(raw[4] == 0, f'Motor {ident} reported fault bits {raw[4]}')
            require(not self.port.in_waiting, 'Extra serial data after reply')
            return list(raw[5:-1])
        except BaseException:
            self.healthy = False
            raise

    def read(self, ident, address, count=1):
        require((address, count) in {(2, 7), (9, 10), (30, 1), (33, 38), (41, 7), (55, 1)},
                'Read outside commissioning scope')
        return self.exchange(ident, 2, [address, count], count)

    def goal(self, ident, target):
        if ident == IDS['base']:
            require(type(self.hold_target) is int and target == self.hold_target,
                    'Motor 1 is stationary; only its latched starting position is allowed')
        else:
            require(ident in MOVABLE.values() and ident == self.selected and self.bounds is not None,
                    'Motor is not selected')
            require(type(target) is int and self.bounds[0] <= target <= self.bounds[1], 'Target outside test window')
        require(type(target) is int and 0 <= target <= 4095, 'Invalid encoder target')
        # RAM only: acceleration, goal position, time=0, nonzero speed.
        parameters = [41, 1, target & 255, target >> 8, 0, 0, SPEED, 0]
        self.writes.append({'id': ident, 'address': 41, 'target': target})
        self.exchange(ident, 3, parameters, 0)

    def enable(self, ident):
        require((ident == IDS['base'] and type(self.hold_target) is int)
                or (ident in MOVABLE.values() and ident == self.selected), 'Motor is not selected')
        self.writes.append({'id': ident, 'address': 40, 'value': 1})
        self.exchange(ident, 3, [40, 1], 0)


def identity(bus, ident):
    info = bus.read(ident, 2, 7)
    require(info[0] == 0 and word(info, 1) == 777 and info[3] == ident and info[4] == 0,
            f'Motor {ident}: identity, byte order or baud differs from the verified setup')
    require(info[6] == 1, f'Motor {ident}: write acknowledgements must be enabled')
    limits = bus.read(ident, 9, 10)
    require(limits[9] == 12 and bus.read(ident, 30) == [1],
            f'Motor {ident}: unsupported phase or encoder resolution; no settings were changed')
    require(bus.read(ident, 55) == [1], f'Motor {ident}: settings lock is not enabled')
    return {'id': ident, 'model_code': 777, 'phase': limits[9],
            'motor_min': word(limits), 'motor_max': word(limits, 2)}


def sample(bus):
    start = time.monotonic()
    states = {}
    for ident in MOTOR_IDS:
        data = bus.read(ident, 33, 38)
        raw = word(data, 23)
        voltage = data[29]/10
        require(data[0] == 0 and data[7] in (0, 1), f'Motor {ident}: mode or torque state changed')
        require(raw <= 4095, f'Motor {ident}: wrapped/multiturn position is not supported')
        # The supply was measured at 12.6 V; internal sensors read 12.6--12.8 V.
        # This is a sensor guard, not permission to exceed the motor voltage rating.
        require(10 <= voltage <= 12.9 and data[30] <= 50 and data[32] == 0,
                f'Motor {ident}: voltage, temperature or status fault')
        states[str(ident)] = {'counts': raw, 'torque': data[7], 'voltage_v': voltage,
                             'temperature_c': data[30], 'moving': data[33], 'status': data[32]}
    elapsed = time.monotonic()-start
    require(elapsed <= FEEDBACK_DEADLINE, 'Three-motor feedback took more than 15 ms')
    return states, elapsed


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, indent=2)+'\n')
    temporary.replace(path)


def prepare(bus, port, session_path, seconds=3.):
    info = {str(i): identity(bus, i) for i in MOTOR_IDS}
    first = None
    latencies = []
    ticks = max(2, math.ceil(seconds/PERIOD))
    deadline = time.monotonic()
    last_tick = deadline
    started = deadline
    max_interval = 0.
    for _ in range(ticks):
        now = time.monotonic()
        require(now-last_tick <= .05, 'Feedback loop missed its timing deadline')
        max_interval = max(max_interval, now-last_tick)
        last_tick = now
        states, latency = sample(bus)
        latencies.append(latency)
        if first is None:
            first = states
        for key, state in states.items():
            require(abs(state['counts']-first[key]['counts']) <= 1 and state['moving'] == 0,
                    f'Motor {key} moved during preparation; support it and try again')
        deadline += PERIOD
        time.sleep(max(0., deadline-time.monotonic()))
    motors = {}
    for key, state in states.items():
        center = state['counts']
        require(info[key]['motor_min']+SPAN <= center <= info[key]['motor_max']-SPAN,
                f'Motor {key}: too close to the motor angle boundary')
        motors[key] = {**info[key], **state, 'anchor': center, 'min': center-SPAN, 'max': center+SPAN}
    result = {'profile': PROFILE,
              'scope': 'Three-motor commissioning; base ID 1 fixed; not arm calibration or MoveIt control',
              'port': port, 'created_unix': time.time(), 'motors': motors,
              'samples': ticks, 'requested_rate_hz': 50,
              'measured_rate_hz': ticks/(time.monotonic()-started), 'max_interval_ms': 1000*max_interval,
              'max_feedback_read_ms': 1000*max(latencies),
              'writes_sent': len(bus.writes), 'multimeter_supply_v': 12.6}
    save(session_path, result)
    return result


def prepare_turn(bus, port, session_path, motor, direction, seconds=3.):
    require(motor in MOVABLE and direction in (-1, 1), 'Motor 1 is stationary; choose shoulder or elbow and a valid direction')
    session = prepare(bus, port, session_path, seconds)
    session_path.unlink()  # Only save the larger window after all its checks pass.
    saved = session['motors'][str(IDS[motor])]
    anchor = saved['anchor']
    low, high = ((anchor-2, anchor+TURN_WINDOW) if direction == 1
                 else (anchor-TURN_WINDOW, anchor+2))
    require(saved['motor_min'] <= low < high <= saved['motor_max'],
            'Ten-degree movement would cross a motor angle boundary; no wrapping allowed')
    saved['min'], saved['max'] = low, high
    session['turn'] = {'motor': motor, 'direction': direction,
                       'requested_counts': TURN_COUNTS, 'minimum_measured_counts': TURN_MINIMUM}
    save(session_path, session)
    return session


def step(bus, port, session, motor, delta, ten_degree=False):
    require(motor in MOVABLE, 'Motor 1 is stationary; choose shoulder or elbow')
    require(session.get('profile') == PROFILE and set(session.get('motors', {})) == {'1', '2', '3'},
            'Run prepare again for the three-motor fixed-base setup; old sessions are not accepted')
    if ten_degree:
        turn = session.get('turn', {})
        require(turn.get('motor') == motor and turn.get('direction') in (-1, 1)
                and turn.get('requested_counts') == TURN_COUNTS
                and turn.get('minimum_measured_counts') == TURN_MINIMUM
                and delta == turn['direction']*TURN_COUNTS,
                'Prepare a dedicated ten-degree test for this motor and direction')
    else:
        require('turn' not in session, 'Use the turn command with a ten-degree preparation')
        require(type(delta) is int and 1 <= abs(delta) <= SPAN, 'Use a nonzero count step between -8 and 8')
    require(session['port'] == port and 0 <= time.time()-session['created_unix'] <= 900,
            'Preparation is missing, expired or for a different port; run prepare again')
    for ident_check in MOTOR_IDS:
        saved = session['motors'][str(ident_check)]
        expected_min, expected_max = saved['anchor']-SPAN, saved['anchor']+SPAN
        if ten_degree and ident_check == IDS[motor]:
            expected_min, expected_max = ((saved['anchor']-2, saved['anchor']+TURN_WINDOW)
                if delta > 0 else (saved['anchor']-TURN_WINDOW, saved['anchor']+2))
        require(saved['id'] == ident_check and saved['model_code'] == 777 and saved['phase'] == 12
                and type(saved['anchor']) is int and saved['min'] == expected_min
                and saved['max'] == expected_max
                and saved['motor_min'] <= saved['min'] < saved['max'] <= saved['motor_max'],
                'Invalid prepared motor window; run prepare again')
    ident = IDS[motor]
    key = str(ident)
    for i in MOTOR_IDS:
        identity(bus, i)
    states, _ = sample(bus)
    bounds = session['motors'][key]
    start = states[key]['counts']
    target = start+delta
    if ten_degree:
        require(abs(start-bounds['anchor']) <= 2, 'Ten-degree starting position changed; prepare again')
    require(bounds['min'] <= target <= bounds['max'], 'Step exceeds the prepared count window')
    for k, state in states.items():
        limit = session['motors'][k]
        require(limit['min'] <= state['counts'] <= limit['max'] and state['moving'] == 0,
                'Motor moved outside the prepared window or is already moving; prepare again')
        if state['torque']:
            old_goal = word(bus.read(int(k), 41, 7), 1)
            require(abs(old_goal-state['counts']) <= 2, 'An enabled motor has a pending movement; stopped')
    stationary = str(IDS['base'])
    hold_target = states[stationary]['counts']
    require(abs(hold_target-session['motors'][stationary]['anchor']) <= HOLD_TOLERANCE,
            'Stationary motor 1 moved since preparation; support it and prepare again')
    other = str(next(i for i in MOVABLE.values() if i != ident))
    other_start = states[other]['counts']
    other_torque = states[other]['torque']
    bus.hold_target = hold_target
    bus.selected, bus.bounds = ident, (bounds['min'], bounds['max'])
    last_states = states
    last_receipt = time.monotonic()
    wrote = False
    complete = False
    latencies = []
    try:
        # Latch and verify ID 1's measured goal BEFORE enabling its hold torque.
        # This goal never follows drift and is never changed to a movement target.
        wrote = True
        for preload_id, preload_target in ((IDS['base'], hold_target), (ident, start)):
            bus.goal(preload_id, preload_target)
            goal = bus.read(preload_id, 41, 7)
            require(goal == [1, preload_target & 255, preload_target >> 8, 0, 0, SPEED, 0],
                    'Measured-position preload did not verify')
            states, _ = sample(bus)
            last_states, last_receipt = states, time.monotonic()
            require(abs(states[str(preload_id)]['counts']-preload_target) <= 1,
                    'Motor moved during preload')
            require(abs(states[stationary]['counts']-hold_target) <= HOLD_TOLERANCE,
                    'Stationary motor 1 moved during preload')
            if states[str(preload_id)]['torque'] == 0:
                bus.enable(preload_id)
            states, _ = sample(bus)
            last_states, last_receipt = states, time.monotonic()
            require(states[str(preload_id)]['torque'] == 1, 'Motor torque enable did not verify')
            require(abs(states[stationary]['counts']-hold_target) <= HOLD_TOLERANCE,
                    'Stationary motor 1 moved while enabling hold')
            require(abs(states[other]['counts']-other_start) <= 2
                    and states[other]['torque'] == other_torque, 'Unselected motor moved or changed torque state')
        scheduled = start
        began = time.monotonic()
        deadline = began
        previous_tick = began
        # Settle at the measured position, move one count per 0.1 s, then observe.
        duration = SETTLE_SECONDS+abs(delta)*MOVE_INTERVAL+OBSERVE_SECONDS
        while time.monotonic()-began < duration:
            now = time.monotonic()
            require(now-previous_tick <= .05, 'Control loop missed its timing deadline')
            previous_tick = now
            states, latency = sample(bus)
            last_states, last_receipt = states, time.monotonic()
            latencies.append(latency)
            measured = states[key]['counts']
            require(states[stationary]['torque'] == 1
                    and abs(states[stationary]['counts']-hold_target) <= HOLD_TOLERANCE,
                    'Stationary motor 1 moved or lost hold torque; stop and support it')
            require(states[key]['torque'] == 1, 'Selected motor lost torque')
            require(bounds['min'] <= measured <= bounds['max'], 'Measured position exceeded the test window')
            require(abs(measured-scheduled) <= 4, 'Motor is not following the small test command')
            require(abs(states[other]['counts']-other_start) <= 2 and states[other]['torque'] == other_torque,
                    'Unselected motor moved or changed torque state; stop and support it')
            for k, state in states.items():
                require(abs(state['voltage_v']-session['motors'][k]['voltage_v']) <= .21,
                        'Motor voltage changed during the test')
            fraction = min(abs(delta), max(0, int((now-began-SETTLE_SECONDS)/MOVE_INTERVAL)))
            next_goal = start+(1 if delta > 0 else -1)*fraction
            if next_goal != scheduled:
                bus.goal(ident, next_goal)
                scheduled = next_goal
            deadline += PERIOD
            time.sleep(max(0., deadline-time.monotonic()))
        end = last_states[key]['counts']
        require(scheduled == target and abs(end-target) <= 2 and (end-start)*delta > 0,
                'Expected movement was not confirmed; inspect the motor/load before another test')
        if ten_degree:
            require((end-start)*(1 if delta > 0 else -1) >= TURN_MINIMUM,
                    'Less than ten degrees of measured movement; no additional movement will be attempted')
        complete = True
        return {'result': 'PASS', 'motor': motor, 'id': ident, 'start_counts': start,
                'target_counts': target, 'end_counts': end, 'measured_change_counts': end-start,
                'shaft_change_degrees': (end-start)*360/4096,
                'max_feedback_read_ms': 1000*max(latencies), 'final_states': last_states,
                'torque_left_enabled': True, 'stationary_motor_id': 1,
                'stationary_target_counts': hold_target,
                'stationary_end_counts': last_states[stationary]['counts']}
    finally:
        # Do not release a loaded arm. On a healthy connection, stop at the latest
        # measured position. Never resend stale feedback after a serial fault.
        if wrote and not complete:
            if bus.healthy and time.monotonic()-last_receipt <= .05:
                measured = last_states[key]['counts']
                if bounds['min'] <= measured <= bounds['max']:
                    bus.goal(ident, measured)
            print('TEST STOPPED. Torque was not released. Support the arm before disconnecting power.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--session', type=Path, default=Path('work/three-motor-session.json'))
    parser.add_argument('--report', type=Path, default=Path('reports/iron/three-motor-last-test.json'))
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('prepare', help='Read all three motors at 50 Hz and record a small test window; no writes')
    run = sub.add_parser('step', help='Hold motor 1 and move shoulder 2 or elbow 3; leave torque enabled')
    run.add_argument('--motor', choices=MOVABLE, required=True)
    run.add_argument('--counts', type=int, default=4)
    run.add_argument('--execute', action='store_true', required=True)
    larger = sub.add_parser('prepare-turn', help='Prepare one bounded ten-degree test; reads only')
    larger.add_argument('--motor', choices=MOVABLE, required=True)
    larger.add_argument('--direction', choices=['positive', 'negative'], default='positive')
    turn = sub.add_parser('turn', help='Run the prepared ten-degree shaft movement')
    turn.add_argument('--motor', choices=MOVABLE, required=True)
    turn.add_argument('--execute', action='store_true', required=True)
    assign = sub.add_parser('assign-id', help='Permanently assign one physically isolated motor ID in EEPROM')
    assign_target = assign.add_mutually_exclusive_group(required=True)
    assign_target.add_argument('--motor', choices=IDS, help='Role alias for base 1, shoulder 2 or elbow 3')
    assign_target.add_argument('--new-id', type=int, choices=range(1, 7), help='Permanent target ID, 1 through 6')
    assign.add_argument('--old-id', type=int, choices=range(254), required=True, metavar='0..253')
    assign.add_argument('--single-motor-connected', action='store_true', required=True)
    assign.add_argument('--disable-torque', action='store_true', help='Explicit torque-off for an unloaded/supported motor')
    assign.add_argument('--execute', action='store_true', required=True)
    verify = sub.add_parser('verify-id', help='Read-only check of the assigned ID after an operator power cycle')
    verify_target = verify.add_mutually_exclusive_group(required=True)
    verify_target.add_argument('--motor', choices=IDS)
    verify_target.add_argument('--id', type=int, choices=range(1, 7), help='ID to verify, 1 through 6')
    verify.add_argument('--single-motor-connected', action='store_true', required=True)
    args = parser.parse_args()
    if args.command in ('prepare', 'prepare-turn', 'assign-id') and args.session.exists():
        args.session.unlink()  # Failed preparation must not leave an old valid session.
    result = {'result': 'FAIL', 'command': args.command, 'writes': []}
    try:
        import serial
        write_timeout = .2 if args.command in ('assign-id', 'verify-id') else .01
        with serial.Serial(args.port, 1000000, timeout=.001, write_timeout=write_timeout, exclusive=True) as port:
            if args.command in ('assign-id', 'verify-id'):
                from .id_assignment import IDBus, assign_isolated, verify_isolated
                bus = IDBus(port)
                result['writes'] = bus.writes
                if args.command == 'assign-id':
                    target_id = IDS[args.motor] if args.motor else args.new_id
                    result.update(assign_isolated(bus, args.old_id, target_id,
                        single_motor_connected=args.single_motor_connected, disable_torque=args.disable_torque))
                else:
                    target_id = IDS[args.motor] if args.motor else args.id
                    result.update(verify_isolated(bus, target_id, single_motor_connected=args.single_motor_connected))
                result['motor'] = args.motor or f'ID {target_id}'
            else:
                bus = Bus(port)
                result['writes'] = bus.writes
            if args.command in ('assign-id', 'verify-id'):
                pass
            elif args.command == 'prepare':
                result.update(prepare(bus, args.port, args.session)); result['result'] = 'PASS'
            elif args.command == 'prepare-turn':
                result.update(prepare_turn(bus, args.port, args.session, args.motor,
                    1 if args.direction == 'positive' else -1)); result['result'] = 'PASS'
            elif args.command == 'turn':
                session = json.loads(args.session.read_text())
                require(session.get('turn', {}).get('direction') in (-1, 1), 'Run prepare-turn first')
                result.update(step(bus, args.port, session, args.motor,
                    session['turn']['direction']*TURN_COUNTS, ten_degree=True))
            else:
                result.update(step(bus, args.port, json.loads(args.session.read_text()), args.motor, args.counts))
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or 'Interrupted by operator'
    result['recorded_unix'] = time.time()
    save(args.report, result)
    save(args.report.parent/f'three-motor-{args.command}-{time.time_ns()}.json', result)
    print(json.dumps(result, indent=2), flush=True)
    if args.command in ('step', 'turn'):
        print('Torque is not automatically released. Support the arm before disconnecting servo power.')
    return 0 if result['result'] == 'PASS' else 1
