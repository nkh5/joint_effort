#!/usr/bin/env python3
"""Turn a bench results folder into report.md (+ plots/*.png if matplotlib is installed).

  python3 bench/bench_report.py bench/results/<timestamp>
"""
import json
import math
import re
import statistics
import sys
from pathlib import Path

CPD = 4096 / 360.0


def fmt(v, nd=2, unit=''):
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.{nd}f}{unit}'
    return f'{v}{unit}'


def verdict(ok):
    return 'PASS' if ok else '**CHECK**'


def table(header, rows):
    out = ['| ' + ' | '.join(header) + ' |', '|' + '---|' * len(header)]
    out += ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows]
    return out


def serial_section(s, lines, findings):
    ids = s['ids']
    a = s.get('audit', {})
    lines += ['## A. Settings audit', '']
    regs = a.get('registers', {})
    keys = ['model', 'firmware_major', 'firmware_minor', 'mode', 'pos_P', 'pos_I', 'pos_D', 'cw_deadband',
            'ccw_deadband', 'min_angle_limit', 'max_angle_limit', 'max_torque', 'protect_current',
            'overload_torque', 'offset', 'voltage', 'temperature']
    if regs:
        lines += table(['register'] + [f'ID {i}' for i in ids],
                       [[k] + [regs[str(i)][k] if str(i) in regs else regs[i][k] for i in ids] for k in keys])
        lines.append('')
    differ = a.get('differ', {})
    for w in a.get('warnings', []):
        findings.append(('CHECK', w))
    if differ:
        findings.append(('CHECK', 'Settings differ between motors: ' + ', '.join(differ)))
        lines.append('Differs between motors: ' + ', '.join(f'`{k}`' for k in differ))
    else:
        lines.append('All compared settings are identical across motors.')
    lines.append('')

    b = s.get('bus')
    if b:
        f, p = b['flat'], b['paced']
        tot_flat = sum(f['misses'].values())
        tot_paced = sum(m['misses'] for m in p['motors'].values())
        lines += ['## B. Raw bus (no ROS): SyncWrite + SyncRead of all motors', '',
                  f'- Flat out: {f["cycles"]} cycles, {f["mean_ms"]:.2f} ms per cycle (p99 {f["p99_ms"]:.2f}, '
                  f'max {f["max_ms"]:.2f}) -> up to ~{f["max_rate_hz"]:.0f} Hz possible; missed replies {tot_flat}. '
                  f'{verdict(tot_flat == 0)}',
                  f'- {p["rate_hz"]:g} Hz, all motors moving +-{p["amp_deg"]:g} deg at {p["freq_hz"]:g} Hz: '
                  f'{p["cycles"]} cycles, late cycles {p["late_cycles"]}, missed replies {tot_paced}. '
                  f'{verdict(tot_paced == 0)}', '']
        lines += table(['motor', 'missed', 'lag ms', 'rms err deg', 'max err deg', 'min V', 'max A'],
                       [[k, m['misses'], fmt(m['lag_ms'], 0), fmt(m['rms_err_deg']), fmt(m['max_err_deg']),
                         fmt(m['min_volt'], 1), fmt(m['max_current_a'])] for k, m in p['motors'].items()])
        lines.append('')
        whole = p.get('cycles_all_missed', 0)
        reads = len(p['motors']) * (p['cycles'] + f['cycles'])
        if (tot_flat or tot_paced) and (tot_flat + tot_paced) <= 0.001 * reads and not whole:
            findings.append(('PASS', f'Raw serial bus: {tot_flat + tot_paced} missed reply in {reads} reads '
                                     f'({100 * (tot_flat + tot_paced) / reads:.2f}%) - normal; the driver simply '
                                     'reuses the last position for that one cycle'))
        elif tot_flat or tot_paced:
            single = tot_paced - whole * len(p['motors'])
            hint = (f'{whole} cycle(s) where ALL motors missed (host/USB hiccup, not one motor)' if whole else '')
            if single > 0 or tot_flat:
                hint += ('; ' if hint else '') + 'individual motors missed replies - wiring/power/adapter'
            findings.append(('CHECK', f'Raw bus missed {tot_flat + tot_paced} replies: {hint}'))
        else:
            findings.append(('PASS', 'Raw serial bus: no missed replies, flat out or at 50 Hz with all motors moving'))
        vmin = [m['min_volt'] for m in p['motors'].values() if m['min_volt']]
        if vmin and min(vmin) < 11.5:
            idle = {k: v.get('voltage') for k, v in s.get('audit', {}).get('registers', {}).items()}
            drops = ', '.join(f'ID {k} {idle.get(k, float("nan")) - m["min_volt"]:.1f} V'
                              for k, m in p['motors'].items() if m['min_volt'])
            findings.append(('CHECK', f'Supply dipped to {min(vmin):.1f} V with all motors moving (drop from idle: '
                                      f'{drops}). A drop that grows along the chain is wiring resistance; it grows '
                                      'with load on the arm'))

    st = s.get('steps')
    if st:
        lines += ['## C. Step response (no load)', '',
                  'final err = where it settled minus where it was told to go (1 count = 0.088 deg).', '']
        rows, worst = [], 0
        for sid, by_acc in st.items():
            for acc, rs in by_acc.items():
                for size in sorted({r['size_deg'] for r in rs}):
                    sel = [r for r in rs if r['size_deg'] == size]
                    errs = [abs(r['final_err_counts']) for r in sel]
                    worst = max(worst, max(errs))
                    rows.append([sid, acc, f'{size:g}', fmt(statistics.mean(errs), 1), fmt(max(errs), 1),
                                 fmt(statistics.mean(r['moved_counts'] / r['asked_counts'] for r in sel), 3),
                                 fmt(statistics.mean([r['latency_ms'] for r in sel if r['latency_ms'] is not None]
                                                     or [float('nan')]), 0),
                                 fmt(statistics.mean([r['t90_ms'] for r in sel if r['t90_ms'] is not None]
                                                     or [float('nan')]), 0),
                                 fmt(max(r['overshoot_counts'] for r in sel), 1)])
        lines += table(['motor', 'ACC', 'step deg', 'mean |err| c', 'max |err| c', 'moved/asked', 'latency ms',
                        't90 ms', 'overshoot c'], rows)
        lines.append('')
        if worst <= 2:
            findings.append(('PASS', f'No-load steps land within {worst:.0f} counts ({worst / CPD:.2f} deg) of target'))
        else:
            findings.append(('CHECK', f'No-load steps end up to {worst:.0f} counts ({worst / CPD:.2f} deg) off target'))

    db = s.get('deadband')
    if db:
        summ = {k: deadband_summary(v['rows']) for k, v in db.items()}
        lines += ['## D. Deadband (steps of 1-8 counts)', '']
        lines += table(['motor', 'smallest step that moves (both ways)', 'stops short of goal by (median)'],
                       [[k, f'{v["smallest_moving_counts"]} c' if v['smallest_moving_counts'] else 'none <= 8 c',
                         f'{v["stop_short_counts"]} c' if v['stop_short_counts'] is not None else '-']
                        for k, v in summ.items()])
        lines += ['', 'Steps smaller than the first column are ignored; larger steps stop short by the second '
                  'column (the servo stops pushing inside its deadband). 1 count = 0.088 deg.', '']
        big = [k for k, v in summ.items() if not v['smallest_moving_counts'] or v['smallest_moving_counts'] > 3]
        if big:
            findings.append(('INFO', f'Motors {big} ignore steps below 4 counts (0.35 deg) and stop ~2 counts short: '
                                     'a goal creeping slower than that is followed in 1-count hops'))

    rp = s.get('ramps')
    if rp:
        lines += ['## E. Slow ramp, 3 deg over 3 s, new goal every 20 ms (= the jog test)', '',
                  'Smooth = moves of ~1 count with pauses near the ideal; stair-stepping = big moves after long pauses.',
                  '']
        rows, best_by_motor, scores = [], {}, {}
        for sid, variants in rp.items():
            for name, v in variants.items():
                u, d = v['up'], v['down']
                mm = max(x for x in (u['mean_move_counts'], d['mean_move_counts']) if x is not None) \
                    if u['mean_move_counts'] or d['mean_move_counts'] else None
                rows.append([sid, name, fmt(u['mean_move_counts'], 1) + ' / ' + fmt(d['mean_move_counts'], 1),
                             fmt(u['max_move_counts']) + ' / ' + fmt(d['max_move_counts']),
                             fmt(u['max_pause_ms'], 0) + ' / ' + fmt(d['max_pause_ms'], 0),
                             fmt(u['ideal_pause_ms'], 0), fmt(max(u['max_err_deg'], d['max_err_deg'])),
                             fmt(u['final_err_counts'], 1) + ' / ' + fmt(d['final_err_counts'], 1)])
                score = (mm or 99) + (max(u['max_pause_ms'] or 999, d['max_pause_ms'] or 999) / 100)
                scores.setdefault(sid, {})[name] = score
                if sid not in best_by_motor or score < best_by_motor[sid][0] - 0.3:   # must be clearly better
                    best_by_motor[sid] = (score, name)
        lines += table(['motor', 'setting', 'mean move c (up/down)', 'max move c', 'max pause ms', 'ideal pause ms',
                        'max err deg', 'final err c'], rows)
        lines.append('')
        names = [n for _, n in best_by_motor.values()]
        if names:
            top = max(set(names), key=names.count)
            if top == next(iter(next(iter(rp.values())))):
                lines.append('No setting is clearly smoother than the driver default.')
                findings.append(('INFO', 'Slow ramp: no ACC/speed setting is clearly smoother than the driver default'))
            else:
                lines.append(f'Smoothest setting for most motors: **{top}**.')
                findings.append(('INFO', f'Smoothest slow ramp: "{top}" ({names.count(top)}/{len(names)} motors) - '
                                         'try it on the arm with run_real.sh servo_acc:=N'))
            default = 'driver default (ACC 100, max speed)'
            jag = [sid for sid, vs in rp.items() if default in vs and
                   max(vs[default]['up']['mean_move_counts'] or 0, vs[default]['down']['mean_move_counts'] or 0) > 2]
            if jag:
                findings.append(('CHECK', f'Driver default stair-steps with NO load on motors {jag} '
                                          '(mean move > 2 counts): jaggedness is partly control, not only the arm'))
            else:
                findings.append(('PASS', 'Driver default ramps smoothly with no load: arm jaggedness comes from '
                                         'load / friction / backlash'))
        lines.append('')

    man = s.get('manual')
    if man:
        lines += ['## F. Protractor check', '']
        for sid, m in man.items():
            pts = [p for p in m['points'] if p['protractor_deg'] is not None]
            if len(pts) >= 2:
                p0 = pts[0]
                rows = [[p['commanded_rel_deg'], p['encoder'], p['protractor_deg'],
                         fmt((p['encoder'] - p0['encoder']) / CPD, 1),
                         fmt(p['protractor_deg'] - p0['protractor_deg'], 1)] for p in pts]
                lines += [f'Motor {sid}:'] + table(['commanded rel deg', 'encoder', 'protractor deg',
                                                    'encoder rel deg', 'protractor rel deg'], rows)
            lines.append(f'Motor {sid} backlash: {fmt(m.get("backlash_deg"), 1, " deg")}')
            lines.append('')


def ros_section(results_dir, lines, findings):
    files = sorted(results_dir.glob('bus_*.json'), key=lambda p: int(re.findall(r'\d+', p.stem)[0]))
    if not files:
        return
    lines += ['## G. Through ROS: sts_hardware_interface streaming goals to all motors', '']
    rows = []
    for f in files:
        r = json.loads(f.read_text())
        rate = r['rate_hz']
        logf = results_dir / f'launch_{int(rate)}.log'
        text = logf.read_text(errors='replace') if logf.exists() else ''
        fails = len(re.findall(r'Failed to read feedback', text))
        recov = len(re.findall(r'attempting recovery', text))
        overruns = len(re.findall(r'Overrun detected', text))
        js = r.get('joint_states', {})
        worst_rms = max((m['0.25Hz']['rms_err_deg'] or 0) for m in r['motors'].values())
        worst_max = max((m['1Hz']['max_err_deg'] or 0) for m in r['motors'].values())
        rc = recount_stalls(r)
        if rc is not None:
            for n, m in r['motors'].items():
                m['stalls_total'] = rc[n]
        else:
            for m in r['motors'].values():
                m['stalls_total'] = m['0.25Hz']['stalls'] + m['1Hz']['stalls']
        stalls = sum(m['stalls_total'] for m in r['motors'].values())
        rate_ok = js and abs(js['mean_hz'] - rate) < 0.05 * rate
        ok = fails == 0 and recov == 0 and stalls == 0 and rate_ok
        rows.append([f'{rate:g} Hz', fmt(js.get('mean_hz'), 1), fmt(js.get('max_gap_ms'), 0), fails, recov,
                     overruns, stalls, fmt(worst_rms), fmt(worst_max), verdict(ok)])
        findings.append(('PASS' if ok else 'CHECK',
                         f'ROS driver at {rate:g} Hz: {fails} read failures, {recov} recoveries, {overruns} overruns, '
                         f'{stalls} stalls, joint_states {fmt(js.get("mean_hz"), 1)} Hz'))
        per = [[n, fmt(m['0.25Hz']['lag_ms'], 0), fmt(m['0.25Hz']['rms_err_deg']), fmt(m['0.25Hz']['max_err_deg']),
                fmt(m['1Hz']['lag_ms'], 0), fmt(m['1Hz']['rms_err_deg']), fmt(m['1Hz']['max_err_deg']),
                m['stalls_total'], fmt(m['min_voltage'], 1), fmt(m['max_current_a']),
                fmt(m['max_temp_c'], 0)] for n, m in r['motors'].items()]
        lines += [f'### {rate:g} Hz', ''] + table(
            ['motor', 'slow lag ms', 'slow rms deg', 'slow max deg', 'fast lag ms', 'fast rms deg', 'fast max deg',
             'stalls', 'min V', 'max A', 'max C'], per) + ['']
    lines += ['### Summary', ''] + table(
        ['rate', 'joint_states Hz', 'max gap ms', 'read failures', 'recoveries', 'overruns', 'stalls',
         'worst slow rms deg', 'worst fast max deg', 'result'], rows) + ['']
    lines += ['Lag is the servo following a moving goal (expected 20-80 ms); rms/max are after removing it. '
              'A stall = a motor\'s reported position frozen >= 4 cycles while its goal kept moving the same way '
              '(missed packets or failed reads); pauses where the goal turns around are the deadband, not stalls. "max C" is the temperature the driver reports; if it is far above the '
              'section A reading, it is a bad reading (the servo cuts out at its 70 C limit).', '']


def deadband_summary(rows):
    counts = sorted({r['counts'] for r in rows})
    both = [c for c in counts if all(r['moved'] * r['dir'] >= 1 for r in rows if r['counts'] == c)]
    moving = [abs(r['err']) for r in rows if r['moved'] * r['dir'] >= 1]
    return {'smallest_moving_counts': both[0] if both else None,
            'stop_short_counts': round(statistics.median(moving), 1) if moving else None}


def bus_target(phases, start, t):
    n = len(start)
    t_phase = 0.0
    for kind, dur, amp, freq in phases:
        if t < t_phase + dur:
            if kind == 'sine':
                tt = t - t_phase
                ramp = min(1.0, tt / 0.5, (t_phase + dur - t) / 0.5)
                return [s + ramp * amp * math.pi / 180 * math.sin(2 * math.pi * freq * tt + 2 * math.pi * i / n)
                        for i, s in enumerate(start)]
            return list(start)
        t_phase += dur
    return list(start)


def recount_stalls(r):
    """Stalls = reported position frozen >= 4 cycles while the goal kept moving the same way by > 3 counts.
    A freeze while the goal turns around is the servo's deadband (it must cross ~4 counts to reverse), not a stall."""
    tr = r.get('trace')
    if not tr:
        return None
    start = [d * math.pi / 180 for d in r['start_deg']]
    cnt = 2 * math.pi / 4096
    names = list(r['motors'])
    spans, t0 = [], 0.0
    for kind, dur, amp, freq in r['phases']:
        spans.append((kind, t0, t0 + dur))
        t0 += dur
    out = {n: 0 for n in names}
    for i, n in enumerate(names):
        frozen_since, counted = None, False
        prev_t, prev_p = None, None
        for t, p in zip(tr['t'], tr['pos']):
            if not any(kind == 'sine' and a + 0.7 <= t <= b - 0.7 for kind, a, b in spans):
                frozen_since, prev_t, prev_p = None, t, p[i]      # skip holds and the fade in/out (start from rest)
                continue
            if prev_p is not None and p[i] == prev_p:
                if frozen_since is None:
                    frozen_since, counted = prev_t, False
                g = [bus_target(r['phases'], start, x)[i] for x in (frozen_since - 0.2, frozen_since, t)]
                reversal = (g[1] - g[0]) * (g[2] - g[1]) < 0      # goal turned around: deadband dwell, not a stall
                if not counted and not reversal and t - frozen_since >= 3.5 / r['rate_hz'] and abs(g[2] - g[1]) > 3 * cnt:
                    out[n] += 1
                    counted = True
            else:
                frozen_since = None
            prev_t, prev_p = t, p[i]
    return out


def plots(results_dir):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return []
    out = results_dir / 'plots'
    out.mkdir(exist_ok=True)
    made = []
    tr_file = results_dir / 'serial_traces.json'
    if tr_file.exists():
        tr = json.loads(tr_file.read_text())
        for sid, variants in tr.get('ramps', {}).items():
            fig, ax = plt.subplots(figsize=(8, 4))
            for name, d in variants.items():
                s = d['samples']
                if not s:
                    continue
                p0 = s[0][1]
                ax.step([t for t, _ in s], [(p - p0) / CPD for _, p in s], where='post', label=name, lw=1)
            tg = next(iter(variants.values()))['targets']
            if tg:
                ax.plot([t for t, _ in tg], [(g - tg[0][1]) / CPD for _, g in tg], 'k--', lw=1, label='goal')
            ax.set(xlabel='s', ylabel='deg from start', title=f'Motor {sid}: 3 deg ramp, 50 Hz goals')
            ax.legend(fontsize=7)
            fig.tight_layout()
            f = out / f'ramp_motor{sid}.png'
            fig.savefig(f, dpi=110)
            plt.close(fig)
            made.append(f)
        for sid, accs in tr.get('steps', {}).items():
            fig, ax = plt.subplots(figsize=(8, 4))
            for acc, steps in accs.items():
                for label, s in steps.items():
                    if s:
                        p0 = s[0][1]
                        ax.plot([1000 * t for t, _ in s], [(p - p0) / CPD for _, p in s], lw=1,
                                label=f'{acc} {label} deg')
            ax.set(xlabel='ms', ylabel='deg moved', title=f'Motor {sid}: step responses')
            ax.legend(fontsize=6, ncol=2)
            fig.tight_layout()
            f = out / f'steps_motor{sid}.png'
            fig.savefig(f, dpi=110)
            plt.close(fig)
            made.append(f)
    for bf in sorted(results_dir.glob('bus_*.json')):
        r = json.loads(bf.read_text())
        t, pos = r['trace']['t'], r['trace']['pos']
        if not t:
            continue
        fig, ax = plt.subplots(figsize=(9, 4))
        for i in range(len(pos[0])):
            ax.plot(t, [(p[i] - pos[0][i]) * 180 / 3.141592653589793 for p in pos], lw=1, label=f'motor {i + 1}')
        ax.set(xlabel='s', ylabel='deg from start', title=f'ROS driver at {r["rate_hz"]:g} Hz: measured positions')
        ax.legend(fontsize=7)
        fig.tight_layout()
        f = out / f'{bf.stem}.png'
        fig.savefig(f, dpi=110)
        plt.close(fig)
        made.append(f)
    return made


def main():
    d = Path(sys.argv[1])
    lines, findings = [f'# STS bench report - {d.name}', ''], []
    sp = d / 'serial.json'
    s = json.loads(sp.read_text()) if sp.exists() else None
    body = []
    if s:
        if s.get('aborted'):
            findings.append(('CHECK', 'Serial tests were aborted - results incomplete'))
        serial_section(s, body, findings)
    ros_section(d, body, findings)
    made = plots(d)
    lines += ['## Summary', '']
    order = {'CHECK': 0, 'INFO': 1, 'PASS': 2}
    for level, text in sorted(findings, key=lambda x: order[x[0]]):
        lines.append(f'- **{level}** {text}')
    lines += ['', *body]
    if made:
        lines += ['## Plots', ''] + [f'![{p.stem}](plots/{p.name})' for p in made]
    else:
        lines += ['(Install python3-matplotlib for plots.)']
    (d / 'report.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines[:4 + len(findings)]))
    print(f'\nFull report: {d / "report.md"}')


if __name__ == '__main__':
    main()
