#!/usr/bin/env python3
"""H. Servo gain test on the loose STS servos (no load, no ROS). Run INSIDE the distrobox, from ros2_ws:

  python3 bench/bench_gains.py            full (~13 min): 7 settings x 5 motors
  python3 bench/bench_gains.py --quick    ~7 min: 5 settings, 1 repeat
  python3 bench/bench_gains.py --restore bench/results/gains-<time>/originals.json   (only if a run was killed)

For each motor and each setting (P, D, I, deadband) it runs: steps of 4 counts / 1 / 3 / 10 deg both ways,
a 1.5 s hold (hunting = position wobbling around the goal), and the 3 deg / 3 s slow ramp (= the jog test).

P/D/I/deadband live in each servo's EEPROM. The original values are saved to originals.json BEFORE anything
is changed and written back at the end (also on Ctrl-C / errors). Torque is off at the end.
Needs bench/stsbus.py and bench/bench_serial.py next to it. Other options: --port, --ids 1,2,3,4,5, --yes.
"""
import argparse
import json
import signal
import statistics
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from stsbus import Bus, now  # noqa: E402
from bench_serial import CPD, ramp, ramp_metrics, step, step_metrics  # noqa: E402

ADDR = {'P': 21, 'D': 22, 'I': 23, 'cw_db': 26, 'ccw_db': 27}
LOCK, CURRENT = 55, 69
#            name                          P   D   I  deadband
SETTINGS = [('now (P32 D32 I0 db1)',      32, 32, 0, 1),
            ('deadband 0',                32, 32, 0, 0),
            ('I 2',                       32, 32, 2, 1),
            ('I 5',                       32, 32, 5, 1),
            ('P 48',                      48, 32, 0, 1),
            ('P 64',                      64, 32, 0, 1),
            ('P 48 + I 2 + deadband 0',   48, 32, 2, 0)]
QUICK = [0, 1, 3, 5, 6]


def log(msg=''):
    print(msg, flush=True)


def default_port():
    cal = HERE.parent / 'src/joint_effort_iron/config/sts_calibration.yaml'
    try:
        import yaml
        return yaml.safe_load(cal.read_text())['serial_port']
    except Exception:
        return None


def read_gains(bus, sid):
    d = bus.read(sid, 21, 7)                      # 21..27
    if d is None:
        raise SystemExit(f'ID {sid}: no reply reading gains')
    return {'P': d[0], 'D': d[1], 'I': d[2], 'cw_db': d[5], 'ccw_db': d[6]}


def write_gains(bus, sid, g):
    """Unlock EEPROM, write P/D/I/deadband, lock again, read back; rewrite anything that did not stick
    (an EEPROM write occasionally gets dropped)."""
    back = None
    for attempt in range(5):
        bus.write(sid, LOCK, [0])
        time.sleep(0.005)
        for k, a in ADDR.items():
            if back is None or back[k] != int(g[k]):
                bus.write(sid, a, [int(g[k])])
                time.sleep(0.005)              # give the EEPROM time to store each byte
        bus.write(sid, LOCK, [1])
        time.sleep(0.03)
        back = read_gains(bus, sid)
        if all(back[k] == int(g[k]) for k in ADDR):
            if attempt:
                log(f'    (ID {sid}: needed {attempt + 1} tries to write the settings)')
            return
    raise RuntimeError(f'ID {sid}: wrote {g}, read back {back}')


def as_regs(setting):
    _, p, d, i, db = setting
    return {'P': p, 'D': d, 'I': i, 'cw_db': db, 'ccw_db': db}


def restore(bus, originals):
    ok = True
    for sid, g in originals.items():
        try:
            bus.torque(int(sid), False)
            write_gains(bus, int(sid), g)
            log(f'  ID {sid} restored to P{g["P"]} D{g["D"]} I{g["I"]} deadband {g["cw_db"]}/{g["ccw_db"]}')
        except Exception as e:                      # keep going for the other motors
            ok = False
            log(f'  ID {sid}: RESTORE FAILED ({e}) - rerun with --restore')
    return ok


def hold_test(bus, sid, seconds=1.5):
    """Sit at the current goal: count wobble (hunting) and the current needed to hold still."""
    pos, cur = [], []
    t0 = now()
    k = 0
    while now() - t0 < seconds:
        if k % 2 == 0:
            p = bus.position(sid)
            if p is not None:
                pos.append(p)
        else:
            c = bus.read_u16(sid, CURRENT)
            if c is not None:
                cur.append((c & 0x7FFF) * 0.0065)
        k += 1
    changes = [b - a for a, b in zip(pos, pos[1:]) if b != a]
    reversals = sum(1 for a, b in zip(changes, changes[1:]) if a * b < 0)
    return {'pk_pk_counts': (max(pos) - min(pos)) if pos else None, 'reversals': reversals,
            'mean_current_a': statistics.mean(cur) if cur else None}


def run_setting(bus, sid, start, reps):
    rows = []
    sizes = [('4 c', 4)] + [(f'{d:g} deg', round(d * CPD)) for d in (1, 3, 10)]
    for label, c in sizes:
        for _ in range(reps):
            for sign in (1, -1):
                s0 = bus.position(sid)
                if abs(s0 - start) > 2:
                    bus.goto(sid, start, acc=100)
                    time.sleep(0.4)
                    s0 = bus.position(sid)
                smp = step(bus, sid, s0, start + sign * c, 100)
                m = step_metrics(smp, s0, start + sign * c) if smp else {}
                m.update(size=label, dir=sign)
                rows.append(m)
    # hold at +3 deg, after arriving
    bus.goto(sid, start + round(3 * CPD), acc=100)
    time.sleep(0.6)
    hold = hold_test(bus, sid)
    # slow ramp up and down
    bus.goto(sid, start, acc=100)
    time.sleep(0.5)
    s0 = bus.position(sid)
    up, _, total = ramp(bus, sid, s0)
    s1 = bus.position(sid)
    down, _, total_d = ramp(bus, sid, s1, deg=-3.0)
    rm = [ramp_metrics(up, s0, total), ramp_metrics(down, s1, total_d)]
    bus.goto(sid, start, acc=100)
    time.sleep(0.4)
    errs = [abs(r['final_err_counts']) for r in rows if 'final_err_counts' in r]
    big = [abs(r['final_err_counts']) for r in rows if 'final_err_counts' in r and r['size'] != '4 c']
    return {'steps': rows,
            'mean_err_counts': statistics.mean(errs) if errs else None,
            'max_err_counts': max(errs) if errs else None,
            'mean_err_big_steps': statistics.mean(big) if big else None,
            'small_step_moved': all(r.get('moved_counts', 0) * r['dir'] >= 1 for r in rows if r['size'] == '4 c'),
            'overshoot_counts': max((r.get('overshoot_counts', 0) for r in rows), default=0),
            't90_3deg_ms': statistics.mean([r['t90_ms'] for r in rows if r['size'] == '3 deg' and r.get('t90_ms')]
                                           or [float('nan')]),
            'hold': hold,
            'ramp_max_err_deg': max(r['max_err_deg'] for r in rm),
            'ramp_max_pause_ms': max(r['max_pause_ms'] or 0 for r in rm),
            'ramp_lag_ms': statistics.mean(r['lag_ms'] for r in rm)}


def hunting(r):
    h = r['hold']
    return (h['pk_pk_counts'] or 0) >= 2 and h['reversals'] >= 2


def verdict(r):
    if hunting(r):
        return 'HUNTS'
    if r['overshoot_counts'] > 2:
        return 'overshoots'
    return 'ok'


def pick_best(candidates, err, names):
    """Lowest error, but a change from the current setting must win by >= 0.3 counts (else it is noise);
    ties go to the simpler setting (earlier in the list)."""
    base = names[0]
    best = min(candidates, key=lambda n: (round(err(n), 1), names.index(n)))
    if base in candidates and err(base) - err(best) < 0.3:
        return base
    return best


def report(res, names, out_dir):
    L = ['# H. Servo gain test (no load)', '',
         'Per setting: mean / max |final error| over all steps (counts, 1 c = 0.088 deg), whether a 4-count step '
         'moves at all, overshoot, time to 90% of a 3 deg step, hold wobble (peak-peak counts / reversals in 1.5 s) '
         'and current while holding, worst error in the 3 deg / 3 s ramp. HUNTS = wobbles around the goal while '
         'holding (with load and backlash this only gets worse - do not use).', '']
    best = {}
    for sid, by in res.items():
        L += [f'## Motor {sid}', '',
              '| setting | mean err c | max err c | 4 c step moves | overshoot c | t90 3 deg ms | hold pk-pk c '
              '/ reversals | hold A | ramp max err deg | verdict |',
              '|---|---|---|---|---|---|---|---|---|---|']
        for n in names:
            r = by.get(n)
            if not r:
                continue
            h = r['hold']
            L.append(f'| {n} | {r["mean_err_counts"]:.1f} | {r["max_err_counts"]:.0f} | '
                     f'{"yes" if r["small_step_moved"] else "no"} | {r["overshoot_counts"]:.0f} | '
                     f'{r["t90_3deg_ms"]:.0f} | {h["pk_pk_counts"]} / {h["reversals"]} | '
                     f'{h["mean_current_a"] or 0:.3f} | {r["ramp_max_err_deg"]:.2f} | {verdict(r)} |')
        ok = [n for n in names if n in by and verdict(by[n]) == 'ok']
        if ok:
            best[sid] = pick_best(ok, lambda n: by[n]['mean_err_counts'], names)
        L += ['', f'Best for motor {sid}: **{best.get(sid, "none")}**', '']
    # one setting for all
    common = [n for n in names if all(n in by and verdict(by[n]) == 'ok' for by in res.values())]
    L += ['## Summary', '']
    if common:
        pick = pick_best(common, lambda n: max(by[n]['mean_err_counts'] for by in res.values()), names)
        base = names[0]
        L += [f'- Best single setting that no motor hunts with: **{pick}** - worst motor mean error '
              f'{max(by[pick]["mean_err_counts"] for by in res.values()):.1f} c vs '
              f'{max(by[base]["mean_err_counts"] for by in res.values()):.1f} c now.']
    hunts = sorted({n for by in res.values() for n in by if verdict(by[n]) == 'HUNTS'})
    if hunts:
        L.append(f'- Settings that hunt on at least one motor (too aggressive even with no load): {", ".join(hunts)}')
    L += [f'- Best per motor: ' + ', '.join(f'{k}: {v}' for k, v in best.items()), '',
          'This is a ceiling, not the final tuning: on the arm, use the per-joint keys p_coefficient / '
          'i_coefficient / d_coefficient / deadband (written at every start). Start from the setting above and '
          'back off if a joint buzzes. All motors were restored to their original values after this test.', '']
    text = '\n'.join(L)
    (out_dir / 'gains_report.md').write_text(text)
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=default_port())
    ap.add_argument('--ids', default='1,2,3,4,5')
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--yes', action='store_true')
    ap.add_argument('--restore', metavar='originals.json')
    args = ap.parse_args()
    if not args.port:
        raise SystemExit('No serial port: pass --port /dev/serial/by-id/...')
    if subprocess.run(['pgrep', '-f', '^[^ ]*ros2_control_node'], capture_output=True).returncode == 0:
        raise SystemExit('A ros2_control_node is running - stop it first (only one program may use the adapter):\n'
                         '  pkill -INT -f ros2_control_node; sleep 3; pkill -9 -f ros2_control_node')
    bus = Bus(args.port)

    if args.restore:
        originals = json.loads(Path(args.restore).read_text())
        log('Restoring:')
        ok = restore(bus, originals)
        bus.close()
        return 0 if ok else 1

    ids = [int(i) for i in args.ids.split(',')]
    for s in ids:
        if not bus.ping(s):
            raise SystemExit(f'ID {s} does not answer')
    out_dir = HERE / 'results' / ('gains-' + time.strftime('%Y%m%d-%H%M%S'))
    out_dir.mkdir(parents=True, exist_ok=True)
    originals = {str(s): read_gains(bus, s) for s in ids}
    (out_dir / 'originals.json').write_text(json.dumps(originals, indent=1))
    log(f'Results: {out_dir}')
    log('Original gains (saved to originals.json): ' +
        '; '.join(f'ID {s} P{g["P"]} D{g["D"]} I{g["I"]} db {g["cw_db"]}/{g["ccw_db"]}' for s, g in originals.items()))

    start = {s: bus.position(s) for s in ids}
    for s in ids:
        if not 15 * CPD < start[s] < 4095 - 15 * CPD:
            raise SystemExit(f'ID {s} is at {start[s]} counts - within 15 deg of the encoder wrap. '
                             'Turn it towards the middle (torque is off) and rerun.')
    settings = [SETTINGS[k] for k in QUICK] if args.quick else SETTINGS
    names = [s[0] for s in settings]
    reps = 1 if args.quick else 2
    if not args.yes:
        input(f'\n{len(settings)} settings x {len(ids)} motors. Motors get torque and move up to +-10 deg; their '
              'P/I/D/deadband are changed temporarily and restored at the end.\nPress Enter to start (Ctrl-C aborts): ')

    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    res = {str(s): {} for s in ids}
    aborted = False
    try:
        for s in ids:
            log(f'\n=== Motor {s} ===')
            for st in settings:
                bus.torque(s, False)
                write_gains(bus, s, as_regs(st))
                bus.hold_here(s)
                bus.goto(s, start[s], acc=100)
                time.sleep(0.4)
                r = run_setting(bus, s, start[s], reps)
                res[str(s)][st[0]] = r
                h = r['hold']
                log(f'  {st[0]:28s} mean err {r["mean_err_counts"]:.1f} c (max {r["max_err_counts"]:.0f}), '
                    f'4 c step {"moves" if r["small_step_moved"] else "ignored"}, overshoot '
                    f'{r["overshoot_counts"]:.0f} c, hold wobble {h["pk_pk_counts"]} c/{h["reversals"]} rev, '
                    f'ramp max err {r["ramp_max_err_deg"]:.2f} deg -> {verdict(r)}')
            bus.torque(s, False)
            write_gains(bus, s, originals[str(s)])
    except KeyboardInterrupt:
        aborted = True
        log('\nAborted.')
    except Exception as e:                          # stop cleanly: restore + report what finished
        aborted = True
        log(f'\nStopped by an error: {e}')
    finally:
        log('\nRestoring original gains:')
        ok = restore(bus, originals)
        bus.close()
        (out_dir / 'gains.json').write_text(json.dumps({'originals': originals, 'start': start, 'settings': settings,
                                                        'results': res, 'aborted': aborted}, indent=1, default=str))
        if not ok:
            log(f'\n!! Some motors were NOT restored. Run:  python3 bench/bench_gains.py --restore '
                f'{out_dir / "originals.json"}')
    done = {s: by for s, by in res.items() if by}
    if done:
        text = report(done, names, out_dir)
        log('\n' + text[text.index('## Summary'):])
        log(f'Full report: {out_dir / "gains_report.md"}')
    log('Torque off.')
    return 1 if aborted else 0


if __name__ == '__main__':
    sys.exit(main())
