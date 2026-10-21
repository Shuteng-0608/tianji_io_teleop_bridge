"""Automatic calibration state machine; no connection or motion at import time."""
import time

import numpy as np

from ft_calibration_core import SampleWindow, finite_array


def validate_auto(config, plan, model):
    c = config['automatic']
    if c['velocity_percent'] != 10 or c['acceleration_percent'] != 10:
        raise ValueError('Automatic calibration requires velocity=10%, acceleration=10%')
    for key in ('preflight_s', 'mode_timeout_s', 'move_timeout_s', 'stop_timeout_s',
                'command_timeout_s', 'sensor_stale_s'):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'Invalid automatic.{key}')
    if c['preflight_s'] >= c['sensor_stale_s']:
        raise ValueError('preflight_s must be shorter than sensor_stale_s')
    if c['mode_timeout_s'] <= config['sampling']['settle_s']:
        raise ValueError('mode_timeout_s must exceed settle_s for the pre-move stationary check')
    # Check the zero start and every complete seven-axis target, not only J6/J7.
    previous = np.zeros(7)
    for pose in plan['poses']:
        target = np.array([0.] * 5 + [pose['j6_deg'], pose['j7_deg']])
        for q in np.linspace(previous, target, 101):
            if (np.any(q < model.limits[:, 0]) or np.any(q > model.limits[:, 1]) or
                    not model.wrist_check(q[5:], plan['wrist_margin_deg'])['valid']):
                raise ValueError(f"Invalid zero-start joint segment to {pose['id']}")
        previous = target


class AutoSequence:
    def __init__(self, feedback, config, plan, model):
        self.fb, self.config, self.plan, self.model = feedback, config, plan, model
        self.s, self.a = config['sampling'], config['automatic']
        self.last_frame = None
        self.last_fresh = time.monotonic()
        self.last_wrench = None
        self.last_change = self.last_fresh
        self.latest = None
        self.expected_target = None
        self.control_attempted = False

    def next_row(self, trace, *, position=False):
        now = time.monotonic()
        if now - self.last_fresh > self.s['fresh_timeout_s']:
            raise RuntimeError('Right-arm feedback stopped or frame serial froze')
        row = self.fb.poll()
        if row is None or row['frame'] == self.last_frame:
            time.sleep(1. / self.s['poll_hz'])
            return None
        t = float(row['monotonic_s'])
        if not np.isfinite(t) or abs(now - t) > self.s['fresh_timeout_s']:
            raise RuntimeError('Invalid / stale feedback timestamp')
        if self.latest is not None and t <= self.latest['monotonic_s']:
            raise RuntimeError('Feedback time did not increase')
        self.last_frame, self.last_fresh, self.latest = row['frame'], now, row
        trace(row)  # Persist even the row that explains an abort.
        q = finite_array(row['joint_deg'], (7,), 'joint positions')
        finite_array(row['velocity_deg_s'], (7,), 'joint velocities')
        raw = finite_array(row['wrench_raw'], (6,), 'six-axis raw data')
        finite_array(row['wrench_si'], (6,), 'six-axis SI data')
        finite_array(row['command_deg'], (7,), 'echoed command')
        if row['channel'] != 216:
            raise RuntimeError('Right force telemetry channel changed')
        if (np.max(np.abs(q[:5])) > self.s['base_tolerance_deg'] or
                np.any(q < self.model.limits[:, 0]) or np.any(q > self.model.limits[:, 1]) or
                not self.model.wrist_check(q[5:], self.plan['wrist_margin_deg'])['valid']):
            raise RuntimeError('J1-J5 left zero or wrist / joint limits violated')
        if self.last_wrench is None or not np.array_equal(raw, self.last_wrench):
            self.last_wrench, self.last_change = raw.copy(), now
        if now - self.last_change > self.a['sensor_stale_s']:
            raise RuntimeError('All six sensor channels stayed identical; check sensor signal')
        if position and (row['mode'] != 1 or row['velocity_ratio'] != 10 or
                         row['acceleration_ratio'] != 10):
            raise RuntimeError('Position mode or 10% velocity / acceleration changed')
        time.sleep(1. / self.s['poll_hz'])
        return row

    def stationary(self, row):
        return (row['low_speed'] == 1 and
                max(abs(v) for v in row['velocity_deg_s']) <= self.s['max_velocity_deg_s'])

    def preflight(self, trace):
        start = time.monotonic()
        rows = []
        while time.monotonic() - start < self.a['preflight_s']:
            row = self.next_row(trace)
            if row is None:
                continue
            if (row['mode'] not in (0, 1) or not self.stationary(row) or
                    max(abs(q) for q in row['joint_deg']) > self.s['base_tolerance_deg']):
                raise RuntimeError('Start at seven-axis zero, stationary, in IDLE or Position')
            rows.append(row)
        if len(rows) < 2:
            raise RuntimeError('Insufficient fresh preflight feedback')
        q = np.array([r['joint_deg'] for r in rows])
        raw = np.array([r['wrench_raw'] for r in rows])
        if np.max(np.ptp(q, axis=0)) > self.s['max_position_span_deg']:
            raise RuntimeError('Zero posture drifted during preflight')
        if not np.any(raw) or not np.any(np.ptp(raw, axis=0)):
            raise RuntimeError('Sensor data is all zero or frozen; no motion has been commanded')
        return rows[-1]

    @staticmethod
    def echoed(row, target):
        return np.max(np.abs(np.array(row['command_deg']) - target)) <= .001

    def enable(self, trace):
        target = self.latest['joint_deg']
        self.control_attempted = True  # Includes a failed / partially sent packet.
        self.fb.position(target, enter=True)
        deadline = time.monotonic() + self.a['mode_timeout_s']
        while time.monotonic() < deadline:
            row = self.next_row(trace)
            if row is None:
                continue
            if row['mode'] not in (0, 1, 101):
                raise RuntimeError('Unexpected mode during Position transition')
            if (max(abs(q) for q in row['joint_deg']) > self.s['base_tolerance_deg'] or
                    np.max(np.abs(np.array(row['joint_deg']) - target)) > self.s['max_position_span_deg']):
                raise RuntimeError('Robot moved during Position transition')
            if (row['mode'] == 1 and row['velocity_ratio'] == 10 and
                    row['acceleration_ratio'] == 10 and self.echoed(row, target) and
                    self.stationary(row)):
                self.expected_target = list(target)
                return
        raise RuntimeError('Position mode / 10% limits / latched target not confirmed')

    def wait_previous_target(self, trace):
        """Require a fresh continuous stationary hold before any next target.

        A brief velocity transient after enabling Position restarts the dwell;
        it never authorizes dispatch while moving. Position drift, changed
        commands, mode/ratio changes and stale telemetry still abort immediately.
        """
        deadline = time.monotonic() + self.a['mode_timeout_s']
        stable_since = None
        qmin = qmax = np.array(self.latest['joint_deg'], dtype=float)
        while time.monotonic() < deadline:
            row = self.next_row(trace, position=True)
            if row is None:
                continue
            q = np.array(row['joint_deg'])
            if (not self.echoed(row, self.expected_target) or
                    np.max(np.abs(q - self.expected_target)) > self.s['target_tolerance_deg']):
                raise RuntimeError('Previous Position target was changed or left between points')
            qmin, qmax = np.minimum(qmin, q), np.maximum(qmax, q)
            if np.max(qmax - qmin) > self.s['max_position_span_deg']:
                raise RuntimeError('Previous Position target drifted while waiting for a stationary hold')
            if not self.stationary(row):
                stable_since = None
                continue
            if stable_since is None:
                stable_since = row['monotonic_s']
            if row['monotonic_s'] - stable_since >= self.s['settle_s']:
                return
        raise RuntimeError(
            f"Previous target did not remain stationary for {self.s['settle_s']:g} s "
            f"within {self.a['mode_timeout_s']:g} s; no next target was sent")

    def capture(self, pose, trace):
        self.wait_previous_target(trace)
        target = [0.] * 5 + [pose['j6_deg'], pose['j7_deg']]
        self.fb.position(target)
        self.expected_target = list(target)
        start = time.monotonic()
        arrived = None
        acknowledged = False
        window = SampleWindow(pose, [0.] * 5, self.model, self.s, self.plan['wrist_margin_deg'])
        while True:
            now = time.monotonic()
            if not acknowledged and now - start > self.a['command_timeout_s']:
                raise RuntimeError(f"{pose['id']}: target echo timeout")
            if arrived is None and now - start > self.a['move_timeout_s']:
                raise RuntimeError(f"{pose['id']}: movement timeout")
            if arrived is not None and now - arrived > self.s['timeout_s']:
                raise RuntimeError(f"{pose['id']}: sampling timeout: {window.reason}")
            row = self.next_row(trace, position=True)
            if row is None:
                continue
            if self.echoed(row, target):
                acknowledged = True
            elif acknowledged:
                raise RuntimeError('Position target was changed externally')
            if not acknowledged:
                continue
            at_target = (self.stationary(row) and
                         np.max(np.abs(np.array(row['joint_deg']) - target)) <= self.s['target_tolerance_deg'])
            if arrived is None and at_target:
                arrived = now
                print(f"  {pose['id']} 已到位，等待稳定并采样。", flush=True)
            if arrived is not None and window.feed(row):
                return window.rows

    def stop(self):
        """Best effort, with independent stationary feedback verification."""
        self.fb.stop()
        deadline = time.monotonic() + self.a['stop_timeout_s']
        previous, stable_since, reference = None, None, None
        last_fresh = time.monotonic()
        while time.monotonic() < deadline:
            row = self.fb.poll()
            now = time.monotonic()
            if now - last_fresh > self.s['fresh_timeout_s']:
                raise RuntimeError('Stop requested; feedback unavailable to verify the stop')
            if row is not None and row['frame'] != previous:
                previous, last_fresh = row['frame'], now
                q = finite_array(row['joint_deg'], (7,), 'stop feedback')
                if self.stationary(row):
                    if stable_since is None:
                        stable_since, reference = now, q
                    elif np.max(np.abs(q - reference)) > self.s['max_position_span_deg']:
                        stable_since, reference = now, q
                    elif now - stable_since >= self.s['fresh_timeout_s']:
                        return
                else:
                    stable_since = None
            time.sleep(1. / self.s['poll_hz'])
        raise RuntimeError('Stop requested but stationary state was not verified')
