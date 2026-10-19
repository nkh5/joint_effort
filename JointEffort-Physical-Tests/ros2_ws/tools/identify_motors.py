#!/usr/bin/env python3
"""Read motor addresses and capture raw replies. No movement or setting changes."""
import argparse
import json
from pathlib import Path
import time

READS = {(5, 1), (3, 3), (33, 1), (40, 1), (56, 2), (62, 2)}


def read_request(ident, address, count):
    if type(ident) is not int or not 0 <= ident <= 253 or (address, count) not in READS:
        raise ValueError('Only individual-ID identification reads are allowed')
    body = bytes([ident, 4, 2, address, count])
    return b'\xff\xff'+body+bytes([(~sum(body)) & 255])


def decode(raw, request, ident, count):
    """Separate diagnostic frames from noise; never turn them into control feedback."""
    replies = []; noise = bytearray(); echoes = 0; bad_checksums = 0; offset = 0
    while offset < len(raw):
        if raw.startswith(request, offset):
            echoes += 1; offset += len(request); continue
        if raw[offset:offset+2] != b'\xff\xff' or len(raw)-offset < 4:
            noise.append(raw[offset]); offset += 1; continue
        length = raw[offset+3]
        if not 2 <= length <= 64 or offset+length+4 > len(raw):
            noise.append(raw[offset]); offset += 1; continue
        frame = raw[offset:offset+length+4]
        if sum(frame[2:]) & 255 != 255:
            bad_checksums += 1; noise.append(raw[offset]); offset += 1; continue
        if frame[2] == ident and length == count+2:
            replies.append({'error': frame[4], 'data': list(frame[5:-1])})
        else:
            noise.extend(frame)
        offset += len(frame)
    return {'replies': replies, 'echoes': echoes, 'bad_checksums': bad_checksums,
            'unmatched_hex': bytes(noise[:64]).hex(' '), 'received_hex': raw[:128].hex(' ')}


def capture(port, seconds):
    deadline = time.monotonic()+seconds; raw = bytearray()
    while time.monotonic() < deadline and len(raw) < 2048:
        raw.extend(port.read(min(max(port.in_waiting, 1), 2048-len(raw))))
    return bytes(raw)


def query(port, ident, address, count, window=.03):
    stale = capture(port, .003) if port.in_waiting else b''
    request = read_request(ident, address, count)
    if port.write(request) != len(request): raise OSError('Incomplete serial write')
    port.flush()
    result = decode(capture(port, window), request, ident, count)
    result['pre_request_hex'] = stale[:64].hex(' ')
    return result


def one_reply(result):
    replies = result['replies']
    if len(replies) != 1 or replies[0]['error']:
        return None
    return replies[0]['data']


def identify(port, ids, window=.03, progress=None):
    found = []; observations = []; replied_with_fault = []
    for index, ident in enumerate(ids):
        result = query(port, ident, 5, 1, window)
        for reply in result['replies']:
            if reply['error']: replied_with_fault.append({'id': ident, 'error': reply['error']})
        data = one_reply(result)
        if data == [ident]:
            identity = query(port, ident, 3, 3, window)
            verified = one_reply(identity)
            if verified is not None and verified[2] == ident:
                row = {'id': ident, 'reported_id': verified[2],
                       'reported_model_number': verified[0] | verified[1] << 8,
                       'diagnostic_only': True, 'fields': {}, 'capture_warnings': []}
                records = [result, identity]
                for name, address, count in [('mode',33,1), ('torque',40,1),
                                              ('position_counts',56,2), ('voltage_temperature',62,2)]:
                    sample = query(port, ident, address, count, window); records.append(sample)
                    value = one_reply(sample)
                    if value is None:
                        row['fields'][name] = None
                        row['capture_warnings'].append({name: sample})
                    elif name == 'voltage_temperature':
                        row['fields']['voltage_v'] = value[0]/10
                        row['fields']['temperature_c'] = value[1]
                    else:
                        raw = value[0] if count == 1 else value[0] | value[1] << 8
                        row['fields'][name] = -(raw & 0x7fff) if count == 2 and raw & 0x8000 else raw
                row['clean_capture'] = all(not r['unmatched_hex'] and not r['bad_checksums']
                    and not r['echoes'] and not r['pre_request_hex'] and len(r['replies']) == 1 for r in records)
                found.append(row)
            elif len(observations) < 8:
                observations.append({'id': ident, 'identity': identity})
        if result['received_hex'] and len(observations) < 8:
            observations.append({'id': ident, 'address_read': result})
        if progress is not None and (index % 25 == 0 or index+1 == len(ids)):
            progress(index+1, len(ids))
    return {'motors': found, 'fault_replies': replied_with_fault, 'raw_examples': observations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='Servo adapter /dev/serial/by-id/... path')
    parser.add_argument('--baud', type=int, choices=[1000000,500000,250000,128000,115200,76800,57600,38400], default=1000000)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument('--scan', action='store_true', help='Read addresses 0..253 once')
    selection.add_argument('--ids', type=int, nargs='+', help='Read only these addresses; default 1 and 2')
    parser.add_argument('--label', choices=['base','shoulder','chain'], default='chain',
                        help='Your label for what is connected; not an automatic physical identification')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    ids = list(range(254)) if args.scan else args.ids if args.ids is not None else [1,2]
    if len(set(ids)) != len(ids) or any(not 0 <= i <= 253 for i in ids):
        parser.error('Give distinct motor addresses in 0..253')
    import serial
    output = args.output or Path(__file__).resolve().parents[1]/'reports/iron'/f'motor-identification-{args.label}.json'
    report = {'scope': 'Read-only identification; not calibration or motion approval',
              'physical_label_supplied_by_operator': args.label, 'port': args.port,
              'baud': args.baud, 'instructions_sent': ['READ_DATA'], 'addresses_checked': ids,
              'capture_window_s': .03, 'note': 'Responding IDs are not a count of physical motors. Duplicate IDs require physical isolation.'}
    print('READ ONLY: no ID assignment, torque changes, or motion commands.', flush=True)
    print('For physical identification, connect one motor at a time with the arm supported.', flush=True)
    try:
        with serial.Serial(args.port, args.baud, timeout=.002, write_timeout=.2, exclusive=True) as port:
            startup = capture(port,.5)
            report['startup_hex'] = startup[:128].hex(' ')
            print('Before request:', report['startup_hex'] or '(no bytes)', flush=True)
            def progress(done,total): print(f'Checked {done}/{total} addresses', flush=True)
            report.update(identify(port,ids,progress=progress))
    except (OSError,serial.SerialException) as exc:
        report['error'] = str(exc)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n')
    for row in report.get('motors',[]): print('FOUND:',json.dumps(row),flush=True)
    if not report.get('motors'): print('No motor identity verified. This does not establish that no motor is connected.')
    for example in report.get('raw_examples',[])[:3]: print('RAW:',json.dumps(example),flush=True)
    if 'error' in report: print('ERROR:',report['error'],flush=True)
    print('Saved:',output,flush=True)
    print('A diagnostic reply does not verify the 50 Hz control-loop timing or the motor voltage variant.')
    return 0 if report.get('motors') and 'error' not in report else 1


if __name__ == '__main__': raise SystemExit(main())
