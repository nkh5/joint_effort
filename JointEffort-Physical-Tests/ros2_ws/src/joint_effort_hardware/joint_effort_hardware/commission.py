"""Small, measured-position tests for the base and shoulder motors."""
import argparse
import json
import math
from pathlib import Path
import time

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14028914-if00'
IDS = {'base': 1, 'shoulder': 2}
SPAN = 8                         # About 0.70 degrees at the motor shaft.
SPEED = 10                       # Nonzero; phase 12 selects counts/second.
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
    require(ident in (1, 2), 'Only motor IDs 1 and 2 are allowed')
    body = bytes([ident, len(parameters)+2, opcode, *parameters])
    return b'\xff\xff'+body+bytes([~sum(body) & 255])


class Bus:
    def __init__(self, port):
        self.port = port
        self.selected = None
        self.bounds = None
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
        require(ident == self.selected and self.bounds is not None, 'Motor is not selected')
        require(type(target) is int and self.bounds[0] <= target <= self.bounds[1], 'Target outside test window')
        # RAM only: acceleration, goal position, time=0, nonzero speed.
        parameters = [41, 1, target & 255, target >> 8, 0, 0, SPEED, 0]
        self.writes.append({'id': ident, 'address': 41, 'target': target})
        self.exchange(ident, 3, parameters, 0)

    def enable(self, ident):
        require(ident == self.selected, 'Motor is not selected')
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
    for ident in (1, 2):
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
    require(elapsed <= .015, 'Two-motor feedback took more than 15 ms')
    return states, elapsed


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, indent=2)+'\n')
    temporary.replace(path)


def prepare(bus, port, session_path, seconds=3.):
    info = {str(i): identity(bus, i) for i in (1, 2)}
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
    result = {'scope': 'Two-motor commissioning only; not arm calibration or MoveIt control',
              'port': port, 'created_unix': time.time(), 'motors': motors,
              'samples': ticks, 'requested_rate_hz': 50,
              'measured_rate_hz': ticks/(time.monotonic()-started), 'max_interval_ms': 1000*max_interval,
              'max_pair_read_ms': 1000*max(latencies),
              'writes_sent': len(bus.writes), 'multimeter_supply_v': 12.6}
    save(session_path, result)
    return result


def prepare_turn(bus, port, session_path, motor, direction, seconds=3.):
    require(motor in IDS and direction in (-1, 1), 'Invalid motor or direction')
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
    for ident_check in (1, 2):
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
    for i in (1, 2):
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
    other = str(3-ident)
    other_start = states[other]['counts']
    other_torque = states[other]['torque']
    bus.selected, bus.bounds = ident, (bounds['min'], bounds['max'])
    last_states = states
    last_receipt = time.monotonic()
    wrote = False
    complete = False
    latencies = []
    try:
        wrote = True
        bus.goal(ident, start)
        goal = bus.read(ident, 41, 7)
        require(goal == [1, start & 255, start >> 8, 0, 0, SPEED, 0], 'Measured-position preload did not verify')
        states, _ = sample(bus)
        last_states, last_receipt = states, time.monotonic()
        require(abs(states[key]['counts']-start) <= 1, 'Selected motor moved during preload')
        if states[key]['torque'] == 0:
            bus.enable(ident)
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
                'max_pair_read_ms': 1000*max(latencies), 'final_states': last_states,
                'torque_left_enabled': True}
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
    parser.add_argument('--session', type=Path, default=Path('work/two-motor-session.json'))
    parser.add_argument('--report', type=Path, default=Path('reports/iron/two-motor-last-test.json'))
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('prepare', help='Read both motors at 50 Hz and record a small test window; no writes')
    run = sub.add_parser('step', help='Move one selected motor; leave torque enabled at the end')
    run.add_argument('--motor', choices=IDS, required=True)
    run.add_argument('--counts', type=int, default=4)
    run.add_argument('--execute', action='store_true', required=True)
    larger = sub.add_parser('prepare-turn', help='Prepare one bounded ten-degree test; reads only')
    larger.add_argument('--motor', choices=IDS, required=True)
    larger.add_argument('--direction', choices=['positive', 'negative'], default='positive')
    turn = sub.add_parser('turn', help='Run the prepared ten-degree shaft movement')
    turn.add_argument('--motor', choices=IDS, required=True)
    turn.add_argument('--execute', action='store_true', required=True)
    args = parser.parse_args()
    if args.command in ('prepare', 'prepare-turn') and args.session.exists():
        args.session.unlink()  # Failed preparation must not leave an old valid session.
    result = {'result': 'FAIL', 'command': args.command, 'writes': []}
    try:
        import serial
        with serial.Serial(args.port, 1000000, timeout=.001, write_timeout=.01, exclusive=True) as port:
            bus = Bus(port)
            result['writes'] = bus.writes
            if args.command == 'prepare':
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
    save(args.report.parent/f'two-motor-{args.command}-{time.time_ns()}.json', result)
    print(json.dumps(result, indent=2), flush=True)
    if args.command in ('step', 'turn'):
        print('Torque is not automatically released. Support the arm before disconnecting servo power.')
    return 0 if result['result'] == 'PASS' else 1
