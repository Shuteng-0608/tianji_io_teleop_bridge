"""Offline end-sensor calibration math. No ROS, networking or control SDK imports."""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


MODEL_SHA256 = 'ce20b1a80974c3ed58cdac4973152625096c8ad93a779e5fd4108c26ce19bd52'


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite_array(value, shape, name):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{name}: expected finite array of shape {shape}')
    return result


def rotation(value, name='rotation'):
    result = finite_array(value, (3, 3), name)
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-6) or not np.isclose(
            np.linalg.det(result), 1., atol=1e-6):
        raise ValueError(f'{name}: expected a proper rotation matrix')
    return result


def read_yaml(path):
    result = yaml.safe_load(Path(path).read_text())
    if not isinstance(result, dict):
        raise ValueError(f'{path}: expected YAML mapping')
    return result


def write_yaml(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False))
    tmp.replace(path)


def write_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


@dataclass
class Model:
    path: str
    digest: str
    robot_type: int
    gravity: np.ndarray
    dh: np.ndarray
    limits: np.ndarray
    coupled: np.ndarray

    @classmethod
    def load(cls, path):
        # Same CSV layout as SDK LOADMvCfg. Read only the right-arm block.
        path = Path(path).resolve()
        digest = sha256(path)
        if digest != MODEL_SHA256:
            raise ValueError('Unverified kinematics model hash; review model/limits before use')
        rows = [[float(x) for x in line.rstrip(',').split(',')]
                for line in path.read_text().splitlines() if line.strip()]
        if len(rows) != 26 or len(rows[13]) != 4 or int(rows[13][0]) != 1017:
            raise ValueError('Expected the validated two-arm CCS model')
        joints = finite_array(rows[14:21], (7, 18), 'model joints')
        dh = np.vstack([joints[:, :4], finite_array(rows[21], (4,), 'flange')])
        return cls(str(path), digest, 1017, np.array(rows[13][1:]), dh,
                   joints[:, 4:6], finite_array(rows[22:26], (4, 3), 'coupled'))

    def wrist_bounds(self, j6):
        """Mirror SDK FxRobot.cpp limit branches (J6 > 1, J6 < -1). Degrees."""
        if not np.isfinite(j6):
            raise ValueError('J6 must be finite')
        lo, hi = self.limits[6]
        if j6 > 1:
            hi = min(hi, np.polyval(self.coupled[0], j6))
            lo = max(lo, np.polyval(self.coupled[3], j6))
        elif j6 < -1:
            hi = min(hi, np.polyval(self.coupled[1], j6))
            lo = max(lo, np.polyval(self.coupled[2], j6))
        return float(lo), float(hi)

    def wrist_check(self, pair, margin=10.):
        j6, j7 = finite_array(pair, (2,), 'J6/J7')
        if not np.isfinite(margin) or margin < 0:
            raise ValueError('Invalid wrist margin')
        lo, hi = self.wrist_bounds(j6)
        margins = [j6 - self.limits[5, 0], self.limits[5, 1] - j6,
                   j7 - lo, hi - j7]
        return {'valid': bool(min(margins) >= margin - 1e-9),
                'j7_allowed_deg': [lo, hi],
                'j6_margin_deg': float(min(margins[:2])),
                'j7_margin_deg_at_fixed_j6': float(min(margins[2:]))}


def validate_plan(plan, model):
    if plan.get('angle_convention') != 'sdk_absolute_degrees':
        raise ValueError('Pose angles must be absolute SDK joint degrees')
    if plan.get('fixed_joints') != [1, 2, 3, 4, 5]:
        raise ValueError('Only J6/J7 may be designed in this plan')
    margin = float(plan['wrist_margin_deg'])
    if not np.isfinite(margin) or margin < 5:
        raise ValueError('Use at least 5 degrees of wrist-limit margin')
    ids, pairs, counts = set(), set(), {'calibration': 0, 'validation': 0}
    for pose in plan['poses']:
        if set(pose) != {'id', 'role', 'j6_deg', 'j7_deg'}:
            raise ValueError('Each pose must specify only id, role, J6 and J7')
        if pose['id'] in ids or pose['role'] not in counts:
            raise ValueError('Duplicate pose id or invalid role')
        if not isinstance(pose['id'], str) or not pose['id'].isalnum():
            raise ValueError('Pose ids must be alphanumeric')
        pair = (pose['j6_deg'], pose['j7_deg'])
        if pair in pairs or not model.wrist_check(pair, margin)['valid']:
            raise ValueError(f"{pose['id']}: duplicate/invalid wrist pose or insufficient margin")
        ids.add(pose['id'])
        pairs.add(pair)
        counts[pose['role']] += 1
    if counts['calibration'] < 10 or counts['validation'] < 4:
        raise ValueError('Plan requires at least 10 calibration and 4 held-out validation poses')
    return counts


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def design_matrices(gravity):
    g = np.asarray(gravity, dtype=float)
    if g.ndim != 2 or g.shape[1] != 3 or not np.isfinite(g).all():
        raise ValueError('Invalid gravity samples')
    # Normalized gravity makes conditioning independent of the units of g.
    g0 = float(np.mean(np.linalg.norm(g, axis=1)))
    if g0 < 1e-6:
        raise ValueError('Gravity cannot be zero')
    u = g / g0
    force = np.vstack([np.column_stack([v, np.eye(3)]) for v in u])
    torque = np.vstack([np.column_stack([-skew(v), np.eye(3)]) for v in u])
    return force, torque, g0


def coverage(gravity):
    force, torque, _ = design_matrices(gravity)
    result = {}
    for name, matrix in [('force', force), ('torque', torque)]:
        singular = np.linalg.svd(matrix, compute_uv=False)
        rank = int(np.linalg.matrix_rank(matrix))
        result[name + '_rank'] = rank
        result[name + '_condition'] = (float(singular[0] / singular[-1])
                                      if rank == matrix.shape[1] else None)
    u = np.asarray(gravity) / np.linalg.norm(gravity, axis=1)[:, None]
    result['max_gravity_angle_deg'] = float(np.degrees(np.arccos(
        np.clip(np.min(u @ u.T), -1., 1.))))
    return result


def coverage_ok(quality, max_condition):
    return all(quality[name + '_rank'] == rank and
               quality[name + '_condition'] is not None and
               quality[name + '_condition'] <= max_condition
               for name, rank in [('force', 4), ('torque', 6)])


def solve(gravity, wrench, max_condition=30.):
    gravity = np.asarray(gravity, dtype=float)
    wrench = finite_array(wrench, (len(gravity), 6), 'wrench')
    if len(gravity) < 10:
        raise ValueError('At least 10 independent calibration poses are required')
    quality = coverage(gravity)
    if not coverage_ok(quality, max_condition):
        raise ValueError(f'Insufficient gravity-direction coverage: {quality}')
    a_f, a_t, g0 = design_matrices(gravity)
    p_f = np.linalg.lstsq(a_f, wrench[:, :3].reshape(-1), rcond=None)[0]
    p_t = np.linalg.lstsq(a_t, wrench[:, 3:].reshape(-1), rcond=None)[0]
    mass = float(p_f[0] / g0)
    if mass <= 1e-6:
        raise ValueError('Non-positive/negligible fitted mass: check signs, axes and samples')
    com = p_t[:3] / (g0 * mass)
    return {'mass_kg': mass, 'com_sensor_m': com.tolist(),
            'force_bias_N': p_f[1:].tolist(), 'torque_bias_Nm': p_t[3:].tolist(),
            'coverage': quality}


def predict(parameters, gravity):
    f = parameters['mass_kg'] * np.asarray(gravity)
    t = np.cross(parameters['com_sensor_m'], f)
    return np.concatenate([f + parameters['force_bias_N'],
                           t + parameters['torque_bias_Nm']], axis=-1)


def residual_report(parameters, gravity, wrench):
    residual = np.asarray(wrench) - predict(parameters, gravity)
    return {'per_axis_rms_N_Nm': np.sqrt(np.mean(residual ** 2, axis=0)).tolist(),
            'per_axis_max_abs_N_Nm': np.max(np.abs(residual), axis=0).tolist(),
            'force_vector_rms_N': float(np.sqrt(np.mean(np.sum(residual[:, :3] ** 2, axis=1)))),
            'torque_vector_rms_Nm': float(np.sqrt(np.mean(np.sum(residual[:, 3:] ** 2, axis=1)))),
            'per_pose_residual_N_Nm': residual.tolist()}


def validate_settings(config):
    c = config['sampling']
    for key in ['duration_s', 'settle_s', 'timeout_s', 'fresh_timeout_s',
                'max_velocity_deg_s', 'max_position_span_deg', 'base_tolerance_deg',
                'target_tolerance_deg', 'max_force_std_N', 'max_torque_std_Nm',
                'min_samples', 'poll_hz']:
        value = float(c[key])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f'Invalid sampling.{key}')
    if c['timeout_s'] < c['duration_s'] + c['settle_s']:
        raise ValueError('Sampling timeout shorter than settle + duration')
    if int(c['min_samples']) != c['min_samples'] or c['min_samples'] < 2:
        raise ValueError('min_samples must be an integer >= 2')
    if c['min_samples'] > c['poll_hz'] * c['duration_s']:
        raise ValueError('min_samples cannot be reached at requested poll rate')
    f = config['fitting']
    for key in ['max_condition', 'max_validation_force_rms_N',
                'max_validation_torque_rms_Nm']:
        if not np.isfinite(f[key]) or f[key] <= 0:
            raise ValueError(f'Invalid fitting.{key}')


def coordinates(config, model):
    s = config['sensor']
    if s.get('axes_confirmed') is not True:
        raise ValueError('Confirm sensor axes/signs and R_flange_sensor before fitting')
    if config['gravity'].get('base_direction_confirmed') is not True:
        raise ValueError('Confirm gravity in the actual right-arm base before fitting')
    r_fs = rotation(s['R_flange_sensor'], 'R_flange_sensor')
    if s.get('force_sign') not in (-1, 1) or s.get('torque_sign') not in (-1, 1):
        raise ValueError('force_sign and torque_sign must be +1 or -1')
    if not isinstance(s.get('frame_id'), str) or not s['frame_id'].strip():
        raise ValueError('Specify the sensor measurement frame name')
    g = config['gravity'].get('base_m_s2')
    g = model.gravity if g is None else finite_array(g, (3,), 'base gravity')
    if not 9. < np.linalg.norm(g) < 10.:
        raise ValueError('Unexpected gravity magnitude')
    return r_fs, np.array(g), np.array([s['force_sign']] * 3 + [s['torque_sign']] * 3)


class SampleWindow:
    """A continuous settled window. Timestamps use a monotonic host clock."""
    def __init__(self, pose, base5, model, settings, margin):
        self.target = np.array([pose['j6_deg'], pose['j7_deg']])
        self.base = finite_array(base5, (5,), 'fixed J1-J5')
        self.model, self.c, self.margin = model, settings, margin
        self.rows = []
        self.stable_since = None
        self.last_seen = None
        self.last_frame = None
        self.reason = 'waiting for fresh feedback'

    def reset(self, reason):
        self.rows = []
        self.stable_since = None
        self.reason = reason

    def feed(self, row):
        now = float(row['monotonic_s'])
        if not np.isfinite(now):
            raise ValueError('Invalid monotonic timestamp')
        if self.last_seen is not None and now <= self.last_seen:
            raise ValueError('Non-increasing feedback time')
        if row['frame'] == self.last_frame:
            if self.last_seen is not None and now - self.last_seen > self.c['fresh_timeout_s']:
                self.reset('feedback stopped')
            return False
        if self.last_seen is not None and now - self.last_seen > self.c['fresh_timeout_s']:
            self.reset('feedback gap')
        self.last_seen, self.last_frame = now, row['frame']
        q = finite_array(row['joint_deg'], (7,), 'joint angles')
        v = finite_array(row['velocity_deg_s'], (7,), 'joint velocity')
        finite_array(row['wrench_si'], (6,), 'six-axis sensor')
        if row['channel'] != 216:
            raise ValueError('Telemetry channel changed; expected 216')
        if np.max(np.abs(q[:5] - self.base)) > self.c['base_tolerance_deg']:
            self.reset('J1-J5 changed; restore the user-selected fixed posture')
        elif not self.model.wrist_check(q[5:], self.margin)['valid']:
            self.reset('wrist limit margin not satisfied')
        elif np.max(np.abs(q[5:] - self.target)) > self.c['target_tolerance_deg']:
            self.reset('J6/J7 have not reached the selected pose')
        elif np.max(np.abs(v)) > self.c['max_velocity_deg_s']:
            self.reset('robot is moving')
        else:
            if self.stable_since is None:
                self.stable_since = now
                self.settle_q = q.copy()
            if np.max(np.abs(q - self.settle_q)) > self.c['max_position_span_deg']:
                self.reset('joint drift during settling/sampling')
                return False
            if now - self.stable_since < self.c['settle_s']:
                self.reason = 'settling'
                return False
            self.rows.append(row)
            self.reason = 'sampling'
            positions = np.array([r['joint_deg'] for r in self.rows])
            if np.max(np.ptp(positions, axis=0)) > self.c['max_position_span_deg']:
                self.reset('joint drift during sampling')
                return False
            if now - self.rows[0]['monotonic_s'] >= self.c['duration_s']:
                if len(self.rows) < self.c['min_samples']:
                    self.reset('too few fresh samples')
                    return False
                std = np.std([r['wrench_si'] for r in self.rows], axis=0)
                if max(std[:3]) > self.c['max_force_std_N'] or max(std[3:]) > self.c['max_torque_std_Nm']:
                    self.reset('force/torque not stable')
                    return False
                return True
        return False
