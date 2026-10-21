#!/usr/bin/python3
"""Right/B bare-sensor hardware zero with before/after verification; no motion calls."""
import argparse
import ctypes as C
from contextlib import nullcontext
from datetime import datetime
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np

from ft_calibration_core import finite_array, write_json


def now_iso():
    return datetime.now().astimezone().isoformat()


def statistics(rows):
    if len(rows) < 200:
        raise RuntimeError(f'Insufficient fresh samples: {len(rows)}; need at least 200')
    w = np.asarray([r['wrench_si'] for r in rows])
    q = np.asarray([r['joint_deg'] for r in rows])
    raw = np.asarray([r['wrench_raw'] for r in rows])
    return {'samples': len(rows), 'duration_s': rows[-1]['monotonic_s'] - rows[0]['monotonic_s'],
            'mean_si': w.mean(axis=0).tolist(), 'std_si': w.std(axis=0).tolist(),
            'joint_mean_deg': q.mean(axis=0).tolist(), 'joint_span_deg': np.ptp(q, axis=0).tolist(),
            'unique_vectors': len(np.unique(raw, axis=0)),
            'all_six_always_zero': not bool(np.any(raw))}


def require_stable(summary):
    if (max(summary['joint_span_deg']) > .15 or
            max(summary['std_si'][:3]) > .1 or max(summary['std_si'][3:]) > .005):
        raise RuntimeError('Joint drift or unstable force data; keep the bare sensor still')
    if summary['all_six_always_zero'] or summary['unique_vectors'] < 2:
        raise RuntimeError('Sensor signal is all zero or frozen; check sensor connection')


def sample(feedback, output, phase, duration, last_frame, expected_mode):
    start = last_fresh = time.monotonic()
    rows = []
    with ((output / (phase + '.jsonl')).open('x') if output is not None
          else nullcontext(None)) as stream:
        while time.monotonic() - start < duration:
            row = feedback.poll()
            now = time.monotonic()
            if now - last_fresh > .2:
                raise RuntimeError('Feedback gap/frozen controller frame exceeds 0.2 s')
            if row is not None and row['frame'] != last_frame:
                if stream is not None:
                    stream.write(json.dumps(row, allow_nan=False) + '\n')
                q = finite_array(row['joint_deg'], (7,), 'joint degrees')
                v = finite_array(row['velocity_deg_s'], (7,), 'joint velocities')
                finite_array(row['wrench_raw'], (6,), 'raw wrench')
                finite_array(row['wrench_si'], (6,), 'SI wrench')
                t = float(row['monotonic_s'])
                if (not np.isfinite(t) or abs(t - now) > .2 or
                        (rows and t <= rows[-1]['monotonic_s'])):
                    raise RuntimeError('Invalid feedback timestamp')
                if row['channel'] != 216:
                    raise RuntimeError('Expected right FT telemetry channel 216')
                if row['mode'] not in (0, 1) or row['mode'] != expected_mode:
                    raise RuntimeError('Keep the initial IDLE or Position mode unchanged')
                if (row['low_speed'] != 1 or np.max(np.abs(v)) > .2 or
                        np.max(np.abs(q)) > .5):
                    raise RuntimeError('Right arm must remain stationary at seven-axis zero')
                last_frame, last_fresh = row['frame'], now
                rows.append(row)
            time.sleep(.0025)
    return rows, last_frame


def print_statistics(label, summary):
    print(f'{label} [Fx Fy Fz Tx Ty Tz] (N, N·m)', flush=True)
    print('  mean:', ', '.join(f'{v:.6f}' for v in summary['mean_si']), flush=True)
    print('  std: ', ', '.join(f'{v:.6f}' for v in summary['std_si']), flush=True)


def run_zero(args, feedback_factory=None):
    if not args.connect or not args.no_tool:
        raise ValueError('Hardware zero requires --connect --no-tool; remove tool and all external contact first')
    if feedback_factory is None:
        from ft_calibration_sdk import Feedback
        feedback_factory = Feedback
    output = Path(args.output).expanduser().resolve() if args.output else None
    if output is not None:
        output.mkdir(parents=True, exist_ok=False)
    result = {'arm': 'right/B', 'created_at': now_iso(), 'operator_confirmed_no_tool': True,
              'motion_commanded': False, 'mode_commanded': False,
              'reset_attempted': False, 'reset_api_invocations': 0, 'zero_verified': False,
              'status': 'created', 'scale': .0001, 'units': ['N'] * 3 + ['Nm'] * 3,
              'verification_mean_threshold_si': [.05] * 3 + [.002] * 3,
              'output_directory': str(output) if output is not None else None}
    def save():
        if output is not None:
            write_json(output / 'summary.json', result)
    save()
    try:
        with feedback_factory(args.sdk_root, args.robot_ip, connect=True) as feedback:
            mode, frame = feedback.initial['mode'], feedback.initial['frame']
            result['status'] = 'reading_before'
            save()
            before, frame = sample(feedback, output, 'before', 2., frame, mode)
            result['before'] = statistics(before)
            print_statistics('BEFORE', result['before'])
            save()
            require_stable(result['before'])
            reset = feedback.lib.OnReset6DofForceSensor
            reset.argtypes, reset.restype = [C.c_char], C.c_bool
            # Persist an attempt marker before the side effect. No automatic retry.
            result.update(status='reset_attempted', reset_attempted=True, reset_time=now_iso())
            save()
            result['reset_api_invocations'] = 1
            result['reset_return'] = bool(reset(b'B'))
            # SDK implementation returns true even after failed internal attempts.
            result['reset_return_is_not_acknowledgement'] = True
            result['status'] = 'reading_after'
            save()
            _, frame = sample(feedback, output, 'settling', 1., frame, mode)
            after, frame = sample(feedback, output, 'after_settled', 5., frame, mode)
            result['after_settled'] = statistics(after)
            print_statistics('AFTER', result['after_settled'])
            save()
            require_stable(result['after_settled'])
            if np.max(np.abs(np.array(result['after_settled']['joint_mean_deg']) -
                             result['before']['joint_mean_deg'])) > .15:
                raise RuntimeError('Joint pose changed across hardware zero')
            mean = np.abs(result['after_settled']['mean_si'])
            verified = bool(result['reset_return'] and np.all(mean <= result['verification_mean_threshold_si']))
            result.update(zero_verified=verified, status='zero_verified' if verified else 'verification_failed')
    except BaseException as error:
        result.update(status='interrupted' if isinstance(error, KeyboardInterrupt) else 'failed',
                      error=str(error))
        raise
    finally:
        result['finished_at'] = now_iso()
        save()
        if output is not None:
            print('Summary:', output / 'summary.json', flush=True)
        print(f"status: {result['status']}; reset_attempted: {result['reset_attempted']}", flush=True)
    print('zero_verified:', str(result['zero_verified']).lower(), flush=True)
    return 0 if result['zero_verified'] else 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__ +
        ' Running this command connects and zeros the bare sensor; keep the arm at zero, stationary and contact-free.')
    parser.add_argument('--sdk-root', default=str(Path.home() / 'TJ_FX_ROBOT_CONTRL_SDK'))
    parser.add_argument('--robot-ip', default='192.168.1.190')
    parser.add_argument('--connect', action='store_true', default=True, help='compatibility option; connection is enabled by default')
    parser.add_argument('--no-tool', action='store_true', default=True, help='compatibility option; this command is for a bare sensor only')
    parser.add_argument('--output', help='optional new directory for raw samples and summary; default: terminal report only')
    args = parser.parse_args(argv)
    def terminate(signum, _):
        raise KeyboardInterrupt(f'Signal {signum}')
    old_term = signal.signal(signal.SIGTERM, terminate)
    try:
        return run_zero(args)
    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as error:
        print('ERROR:', error, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('Interrupted; check the reported reset_attempted state before retrying.', file=sys.stderr)
        return 130
    finally:
        signal.signal(signal.SIGTERM, old_term)


if __name__ == '__main__':
    sys.exit(main())
