#!/usr/bin/env python3
"""Bench tests on unloaded STS servos over the raw serial bus (no ROS). Run via bench/run_bench.sh.

  A  settings audit (EEPROM/RAM registers, flags motors that differ)
  B  raw bus loop: SyncWrite + SyncRead of all motors, flat out and paced at 50 Hz while all move
  C  step response per motor: 0.5/1/3/10 deg, both ways, at ACC 100 (driver default) and ACC 20
  D  deadband: smallest step (counts) that actually moves each motor
  E  slow ramp 3 deg over 3 s with 50 Hz targets (= the jog test), for several ACC/speed settings
  F  (--manual) protractor check of real output angle and backlash

All motion is relative to where each motor starts (max +-15 deg). Torque is switched off at the end.
"""
import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from stsbus import Bus, now  # noqa: E402

CPD = 4096 / 360.0                      # counts per degree
REGS = [  # name, address, size
    ('firmware_major', 0, 1), ('firmware_minor', 1, 1), ('model', 3, 2), ('id', 5, 1), ('baud', 6, 1),
    ('return_delay', 7, 1), ('status_level', 8, 1), ('min_angle_limit', 9, 2), ('max_angle_limit', 11, 2),
    ('max_temp', 13, 1), ('max_volt', 14, 1), ('min_volt', 15, 1), ('max_torque', 16, 2), ('phase', 18, 1),
    ('unload_condition', 19, 1), ('led_alarm', 20, 1), ('pos_P', 21, 1), ('pos_D', 22, 1), ('pos_I', 23, 1),
    ('min_start_force', 24, 2), ('cw_deadband', 26, 1), ('ccw_deadband', 27, 1), ('protect_current', 28, 2),
    ('angular_resolution', 30, 1), ('offset', 31, 2), ('mode', 33, 1), ('protect_torque', 34, 1),
    ('protect_time', 35, 1), ('overload_torque', 36, 1), ('speed_P', 37, 1), ('overcurrent_time', 38, 1),
    ('speed_I', 39, 1), ('torque_enable', 40, 1), ('acc', 41, 1), ('torque_limit', 48, 2), ('lock', 55, 1),
    ('voltage', 62, 1), ('temperature', 63, 1)]
SAME_EXPECTED = {'id', 'offset', 'voltage', 'temperature', 'torque_enable', 'acc', 'torque_limit'}


def log(msg=''):
    print(msg, flush=True)


# ---------------------------------------------------------------------------------- A
def audit(bus, ids):
    raw = {}
    for sid in ids:
        mem = bytearray(71)
        for start in range(0, 71, 20):
            n = min(20, 71 - start)
            d = bus.read(sid, start, n)
            if d is None:
                raise SystemExit(f'ID {sid}: no reply while reading registers {start}..{start + n - 1}')
            mem[start:start + n] = d
        regs = {}
        for name, a, size in REGS:
            v = mem[a] if size == 1 else mem[a] | mem[a + 1] << 8
            if name == 'offset':
                v = -(v & 0x7FF) if v & 0x800 else v & 0x7FF
            regs[name] = v
        regs['voltage'] /= 10
        raw[sid] = regs
    differ = {}
    for name, _, _ in REGS:
        if name in SAME_EXPECTED:
            continue
        values = {sid: raw[sid][name] for sid in ids}
        if len(set(values.values())) > 1:
            differ[name] = values
    warnings = []
    for sid in ids:
        r = raw[sid]
        if r['mode'] != 0:
            warnings.append(f'ID {sid}: mode {r["mode"]} (expected 0 = position)')
        if r['min_angle_limit'] or r['max_angle_limit'] not in (0, 4095):
            warnings.append(f'ID {sid}: angle limits {r["min_angle_limit"]}..{r["max_angle_limit"]} '
                            '(servo refuses goals outside them)')
        if not 10.5 <= r['voltage'] <= 13.0:
            warnings.append(f'ID {sid}: supply {r["voltage"]} V')
    return {'registers': raw, 'differ': differ, 'warnings': warnings}


# ---------------------------------------------------------------------------------- B
def bus_loop(bus, ids, start, seconds_flat, seconds_paced, rate=50.0, amp_deg=15.0, freq=0.5):
    goals = [start[s] for s in ids]
    res = {}
    # flat out, holding still
    times, misses = [], {s: 0 for s in ids}
    t_end = now() + seconds_flat
    while now() < t_end:
        t0 = now()
        bus.sync_write_pos(ids, goals, acc=[100] * len(ids))
        fb = bus.sync_read(ids)
        times.append(now() - t0)
        for s in ids:
            misses[s] += fb[s] is None
    res['flat'] = {'cycles': len(times), 'mean_ms': 1000 * statistics.mean(times),
                   'p99_ms': 1000 * sorted(times)[int(0.99 * (len(times) - 1))], 'max_ms': 1000 * max(times),
                   'max_rate_hz': 1 / statistics.mean(times), 'misses': misses}
    # paced, all motors moving (load the supply and bus at the same time)
    period = 1.0 / rate
    track, misses, late, volts, cur = {s: [] for s in ids}, {s: 0 for s in ids}, 0, {s: [] for s in ids}, {s: [] for s in ids}
    all_missed = 0
    t_start = now()
    k = 0
    while now() - t_start < seconds_paced:
        t = k * period
        tgt = [start[s] + amp_deg * CPD * math.sin(2 * math.pi * freq * t + 2 * math.pi * i / len(ids))
               for i, s in enumerate(ids)]
        bus.sync_write_pos(ids, [round(v) for v in tgt], acc=[100] * len(ids))
        fb = bus.sync_read(ids)
        all_missed += all(fb[s] is None for s in ids)
        for i, s in enumerate(ids):
            if fb[s] is None:
                misses[s] += 1
            else:
                f = Bus.decode_feedback(fb[s])
                track[s].append((t, tgt[i], f['pos']))
                volts[s].append(f['volt'])
                if f['current_a'] is not None:
                    cur[s].append(f['current_a'])
        k += 1
        sleep = t_start + k * period - now()
        if sleep > 0:
            time.sleep(sleep)
        else:
            late += 1
    stats = {}
    for i, s in enumerate(ids):
        lag, rms, mx = best_lag(track[s], lambda t, i=i, s=s: start[s] + amp_deg * CPD * math.sin(
            2 * math.pi * freq * t + 2 * math.pi * i / len(ids)))
        stats[s] = {'samples': len(track[s]), 'misses': misses[s], 'lag_ms': lag * 1000,
                    'rms_err_deg': rms / CPD, 'max_err_deg': mx / CPD,
                    'min_volt': min(volts[s]) if volts[s] else None,
                    'max_current_a': max(cur[s]) if cur[s] else None}
    res['paced'] = {'rate_hz': rate, 'cycles': k, 'late_cycles': late, 'cycles_all_missed': all_missed, 'amp_deg': amp_deg, 'freq_hz': freq,
                    'motors': stats}
    # back to start
    bus.sync_write_pos(ids, [start[s] for s in ids], speed=[600] * len(ids), acc=[50] * len(ids))
    time.sleep(1.0)
    return res


def best_lag(samples, target_fn, max_lag=0.3):
    if not samples:
        return 0.0, float('nan'), float('nan')
    best = None
    for lag_ms in range(0, int(max_lag * 1000) + 1, 5):
        lag = lag_ms / 1000
        errs = [m - target_fn(t - lag) for t, _, m in samples if t - lag >= 0.5]
        if not errs:
            continue
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        if best is None or rms < best[1]:
            best = (lag, rms, max(abs(e) for e in errs))
    return best


# ---------------------------------------------------------------------------------- C, D
def step(bus, sid, s0, goal, acc, window=0.7, speed=0):
    bus.goto(sid, goal, speed=speed, acc=acc)
    t0 = now()
    samples = []
    while now() - t0 < window:
        p = bus.position(sid)
        if p is not None:
            samples.append((now() - t0, p))
    return samples


def compact(samples):
    """Keep only samples where the value changes (and the last one) - enough to redraw the curve."""
    out = []
    for k, (t, p) in enumerate(samples):
        if not out or p != out[-1][1] or k == len(samples) - 1:
            out.append((round(t, 4), p))
    return out


def step_metrics(samples, s0, goal):
    dist = goal - s0
    sign = 1 if dist > 0 else -1
    thr = max(1, min(3, round(0.25 * abs(dist))))
    latency = next((t for t, p in samples if abs(p - s0) >= thr), None)
    t90 = next((t for t, p in samples if (p - s0) * sign >= 0.9 * abs(dist)), None)
    over = max([0.0] + [(p - s0) * sign - abs(dist) for _, p in samples])
    tail = [p for t, p in samples if t >= samples[-1][0] - 0.1] or [samples[-1][1]]
    final = statistics.mean(tail)
    settle = None
    for k in range(len(samples) - 1, -1, -1):
        if abs(samples[k][1] - goal) > 1:
            settle = samples[k + 1][0] if k + 1 < len(samples) else None
            break
    else:
        settle = 0.0
    return {'asked_counts': dist, 'moved_counts': final - s0, 'final_err_counts': final - goal,
            'latency_ms': None if latency is None else 1000 * latency, 't90_ms': None if t90 is None else 1000 * t90,
            'settle_ms': None if settle is None else 1000 * settle, 'overshoot_counts': over}


def step_tests(bus, ids, start, sizes_deg, accs, reps, record):
    out = {}
    for sid in ids:
        out[sid] = {}
        for acc in accs:
            rows = []
            for size in sizes_deg:
                c = max(1, round(size * CPD))
                for rep in range(reps):
                    for sign in (1, -1):
                        s0 = bus.position(sid)
                        goal = start[sid] + sign * c
                        if abs(s0 - start[sid]) > 2:          # come back exactly to start first
                            bus.goto(sid, start[sid], acc=acc)
                            time.sleep(0.4)
                            s0 = bus.position(sid)
                        smp = step(bus, sid, s0, goal, acc)
                        m = step_metrics(smp, s0, goal)
                        m.update({'size_deg': size, 'dir': sign, 'rep': rep})
                        rows.append(m)
                        if record is not None and rep == 0 and size in (1, 3, 10):
                            record.setdefault(str(sid), {}).setdefault(f'acc{acc}', {})[f'{sign * size:+g}'] = compact(smp)
                        back = step(bus, sid, goal, start[sid], acc, window=0.4)
                        del back
            out[sid][acc] = rows
            log(f'  ID {sid} ACC {acc}: ' + ', '.join(
                f'{sz:g}deg err {statistics.mean(abs(r["final_err_counts"]) for r in rows if r["size_deg"] == sz):.1f}c'
                for sz in sizes_deg))
    return out


def deadband_summary(rows):
    """Smallest step the motor actually moves for (both directions), and how far short of the goal it stops."""
    counts = sorted({r['counts'] for r in rows})
    both = [c for c in counts if all(r['moved'] * r['dir'] >= 1 for r in rows if r['counts'] == c)]
    moving = [abs(r['err']) for r in rows if r['moved'] * r['dir'] >= 1]
    return {'smallest_moving_counts': both[0] if both else None,
            'stop_short_counts': round(statistics.median(moving), 1) if moving else None}


def deadband_tests(bus, ids, start, counts=(1, 2, 3, 4, 6, 8)):
    out = {}
    for sid in ids:
        rows = []
        for c in counts:
            for sign in (1, -1):
                bus.goto(sid, start[sid], acc=100)
                time.sleep(0.3)
                s0 = bus.position(sid)
                goal = s0 + sign * c
                smp = step(bus, sid, s0, goal, 100, window=0.4)
                final = statistics.mean(p for t, p in smp if t > 0.3) if smp else s0
                rows.append({'counts': c, 'dir': sign, 'moved': final - s0, 'err': final - goal})
        out[sid] = {'rows': rows, **deadband_summary(rows)}
        log(f'  ID {sid}: smallest step that moves both ways: {out[sid]["smallest_moving_counts"] or "none up to 8"} '
            f'counts; stops short by {out[sid]["stop_short_counts"] if out[sid]["stop_short_counts"] is not None else "-"} counts')
        bus.goto(sid, start[sid], acc=100)
        time.sleep(0.3)
    return out


# ---------------------------------------------------------------------------------- E
RAMP_VARIANTS = [  # name, acc, speed (0 = max, as the driver sends)
    ('driver default (ACC 100, max speed)', 100, 0),
    ('ACC 50', 50, 0),
    ('ACC 20', 20, 0),
    ('ACC 10', 10, 0),
    ('ACC 100, speed 60 c/s', 100, 60),
]


def ramp(bus, sid, s0, deg=3.0, seconds=3.0, rate=50.0, acc=100, speed=0):
    period = 1 / rate
    total = round(deg * CPD)
    samples, targets = [], []
    t_start = now()
    next_tick = 0.0
    k = 0
    n_ticks = int(seconds * rate)
    while True:
        t = now() - t_start
        if t >= next_tick:
            if k > n_ticks + int(0.5 * rate):
                break
            g = s0 + round(total * min(1.0, k / n_ticks))
            bus.goto(sid, g, speed=speed, acc=acc)
            targets.append((t, s0 + total * min(1.0, k / n_ticks)))
            k += 1
            next_tick = k * period
        p = bus.position(sid)
        if p is not None:
            samples.append((now() - t_start, p))
    return samples, targets, total


def ramp_metrics(samples, s0, total, seconds=3.0, deg=3.0):
    rate = abs(total) / seconds
    ideal = lambda t: s0 + total * max(0.0, min(1.0, t / seconds))   # noqa: E731
    lag, rms, mx = best_lag([(t, 0, p) for t, p in samples if 0.15 * seconds <= t <= 0.85 * seconds + 0.3],
                            lambda t: ideal(t), max_lag=0.3)
    mid = [(t, p) for t, p in samples if 0.15 * seconds <= t <= 0.85 * seconds]
    moves, pauses, last_t = [], [], mid[0][0] if mid else 0
    for (t1, p1), (t2, p2) in zip(mid, mid[1:]):
        if p2 != p1:
            moves.append(abs(p2 - p1))
            pauses.append(t2 - last_t)
            last_t = t2
    ideal_pause = 1.0 / rate if rate else float('inf')
    return {'lag_ms': 1000 * lag, 'rms_err_deg': rms / CPD, 'max_err_deg': mx / CPD,
            'move_events': len(moves), 'mean_move_counts': statistics.mean(moves) if moves else None,
            'max_move_counts': max(moves) if moves else None,
            'max_pause_ms': 1000 * max(pauses) if pauses else None,
            'ideal_pause_ms': 1000 * ideal_pause,
            'final_err_counts': statistics.mean(p for t, p in samples[-20:]) - (s0 + total)}


def ramp_tests(bus, ids, start, variants, record):
    out = {}
    for sid in ids:
        out[sid] = {}
        for name, acc, speed in variants:
            bus.goto(sid, start[sid], acc=100)
            time.sleep(0.4)
            s0 = bus.position(sid)
            up, tu, total = ramp(bus, sid, s0, acc=acc, speed=speed)
            mu = ramp_metrics(up, s0, total)
            s1 = bus.position(sid)
            down, td, total_d = ramp(bus, sid, s1, deg=-3.0, acc=acc, speed=speed)
            md = ramp_metrics(down, s1, total_d)
            out[sid][name] = {'up': mu, 'down': md, 'acc': acc, 'speed': speed}
            record.setdefault(str(sid), {})[name] = {'samples': compact(up), 'targets': tu}
            log(f'  ID {sid} {name:38s} mean move {mu["mean_move_counts"] or 0:.1f}c  max pause '
                f'{mu["max_pause_ms"] or 0:.0f}ms  max err {mu["max_err_deg"]:.2f}deg')
    return out


# ---------------------------------------------------------------------------------- F
def ask_float(prompt):
    while True:
        s = input(prompt).strip()
        if s.lower() in ('s', 'skip', ''):
            return None
        try:
            return float(s)
        except ValueError:
            print('  number in degrees, or Enter to skip')


def manual_tests(bus, ids, start):
    log('\n=== F. Manual protractor checks (Enter = skip a reading, "q" at the motor prompt = stop) ===')
    out = {}
    for sid in ids:
        a = input(f'\nMotor {sid}: put it on the protractor rig and press Enter (or "s" skip, "q" stop): ').strip()
        if a.lower() == 'q':
            break
        if a.lower() == 's':
            continue
        rows = []
        for rel in (0, 10, 30, -10, -30, 0):
            goal = start[sid] + round(rel * CPD)
            bus.goto(sid, goal, speed=600, acc=50)
            time.sleep(1.0)
            enc = bus.position(sid)
            deg = ask_float(f'  commanded {rel:+d} deg (encoder {enc}). Protractor reading: ')
            rows.append({'commanded_rel_deg': rel, 'encoder': enc, 'protractor_deg': deg})
        bus.goto(sid, start[sid], speed=600, acc=50)
        time.sleep(1.0)
        log('  Backlash: torque stays ON. Push the stick gently one way and hold.')
        b1 = ask_float('  protractor reading pushed one way: ')
        b2 = ask_float('  push gently the other way. Protractor reading: ')
        out[sid] = {'points': rows, 'backlash_deg': None if b1 is None or b2 is None else abs(b1 - b2),
                    'encoder_during_push': bus.position(sid)}
    return out


# ---------------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', required=True)
    ap.add_argument('--ids', default='1,2,3,4,5')
    ap.add_argument('--out', required=True)
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--manual', action='store_true')
    ap.add_argument('--yes', action='store_true')
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(',')]
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    bus = Bus(args.port)
    result = {'ids': ids, 'quick': args.quick, 'started': time.strftime('%Y-%m-%d %H:%M:%S')}
    traces = {'steps': {}, 'ramps': {}}
    try:
        log('=== A. Settings audit (read-only) ===')
        missing = [s for s in ids if not bus.ping(s)]
        if missing:
            raise SystemExit(f'No reply from IDs {missing}. Check power (12 V), cable, adapter jumper B, IDs.')
        result['audit'] = audit(bus, ids)
        for w in result['audit']['warnings']:
            log('  WARNING ' + w)
        for name, vals in result['audit']['differ'].items():
            log(f'  differs between motors: {name} {vals}')
        if not result['audit']['differ']:
            log('  all motors have the same settings')
        start = {s: bus.position(s) for s in ids}
        result['start_counts'] = start
        for s in ids:
            lo = 15 * CPD
            if not lo < start[s] < 4095 - lo:
                raise SystemExit(f'ID {s} is at {start[s]} counts - within 15 deg of the encoder wrap. '
                                 'Turn it towards the middle (torque is off) and rerun.')
        if not args.yes:
            input('\nThe motors will now get torque and swing up to +-15 deg around where they are.\n'
                  'Clear the sticks, keep the 12 V disconnect in reach, press Enter to start (Ctrl-C aborts): ')
        for s in ids:
            bus.hold_here(s)
        start = {s: bus.position(s) for s in ids}

        log('\n=== B. Raw bus: SyncWrite + SyncRead of all motors ===')
        result['bus'] = bus_loop(bus, ids, start, 3 if args.quick else 5, 10 if args.quick else 20)
        f, p = result['bus']['flat'], result['bus']['paced']
        log(f'  flat out: {f["cycles"]} cycles, {f["mean_ms"]:.2f} ms/cycle (max ~{f["max_rate_hz"]:.0f} Hz), '
            f'misses {f["misses"]}')
        log(f'  50 Hz with all moving: misses ' + str({s: m["misses"] for s, m in p["motors"].items()})
            + f', late cycles {p["late_cycles"]}')

        log('\n=== C. Step response ===')
        sizes = [0.5, 1, 3, 10]
        result['steps'] = step_tests(bus, ids, start, sizes, [100] if args.quick else [100, 20],
                                     1 if args.quick else 2, traces['steps'])

        log('\n=== D. Deadband ===')
        result['deadband'] = deadband_tests(bus, ids, start)

        log('\n=== E. Slow ramp 3 deg / 3 s, 50 Hz targets ===')
        variants = RAMP_VARIANTS[:3] if args.quick else RAMP_VARIANTS
        result['ramps'] = ramp_tests(bus, ids, start, variants, traces['ramps'])

        if args.manual:
            result['manual'] = manual_tests(bus, ids, start)
        result['bus_read_retries'] = bus.errors
    except KeyboardInterrupt:
        result['aborted'] = True
        log('\nAborted.')
    finally:
        for s in ids:
            try:
                bus.torque(s, False)
            except Exception:
                pass
        bus.close()
        (out_dir / 'serial.json').write_text(json.dumps(result, indent=1, default=str))
        (out_dir / 'serial_traces.json').write_text(json.dumps(traces))
        log(f'\nTorque off. Raw results: {out_dir / "serial.json"}')
    return 1 if result.get('aborted') else 0


if __name__ == '__main__':
    sys.exit(main())
