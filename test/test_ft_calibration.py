"""Offline tests: real control-library loading and all sockets are forbidden."""
import copy
import ctypes
import json
from pathlib import Path
import socket
import signal
from types import SimpleNamespace

import numpy as np
import pytest

import ft_calibration_core as core
import ft_calibration_sdk as sdk
import right_ft_calibration as cli
import ft_calibration_auto as auto


ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / 'config/right_ft_calibration.yaml'
POSES = ROOT / 'config/right_ft_calibration_poses.yaml'


@pytest.fixture(autouse=True)
def offline_only(monkeypatch):
    original = ctypes.CDLL

    def local_kine_only(path, *args, **kwargs):
        assert Path(path).name == 'libKine.so', 'Real controller library must never be loaded'
        return original(path, *args, **kwargs)

    def no_socket(*args, **kwargs):
        raise AssertionError('Network access is forbidden in calibration development tests')

    monkeypatch.setattr(ctypes, 'CDLL', local_kine_only)
    monkeypatch.setattr(socket, 'socket', no_socket)


@pytest.fixture
def config():
    return core.read_yaml(CFG)


@pytest.fixture
def model(config):
    return core.Model.load(config['model_file'])


@pytest.fixture
def plan():
    return core.read_yaml(POSES)


@pytest.fixture
def truth():
    return {'mass_kg': .63, 'com_sensor_m': [.015, -.025, .074],
            'force_bias_N': [.025, -.04, .01], 'torque_bias_Nm': [.001, -.003, .002]}


def random_gravity(n=18):
    g = np.random.default_rng(314).normal(size=(n, 3))
    return g / np.linalg.norm(g, axis=1)[:, None] * 9.81


@pytest.mark.parametrize('j6,expected', [(0, 90), (1, 90), (-1, 90), (20, 90),
                                      (25, 84.875), (-25, 84.875),
                                      (40, 69.5), (-40, 69.5), (60, 49), (-60, 49)])
def test_sdk_coupled_limit_values(model, j6, expected):
    assert model.wrist_bounds(j6) == pytest.approx((-expected, expected))


@pytest.mark.parametrize('q', [[50, 75], [50, -75], [-50, 75], [-50, -75],
                             [61, 0], [0, 91], [float('nan'), 0]])
def test_illegal_wrist_combinations(model, q):
    if not np.isfinite(q).all():
        with pytest.raises(ValueError):
            model.wrist_check(q)
    else:
        assert not model.wrist_check(q, 0)['valid']


def test_all_designed_poses_leave_margin_and_only_define_wrist(model, plan):
    assert core.validate_plan(plan, model) == {'calibration': 12, 'validation': 6}
    checks = [model.wrist_check([p['j6_deg'], p['j7_deg']], 10) for p in plan['poses']]
    assert min(c['j6_margin_deg'] for c in checks) == 20
    assert min(c['j7_margin_deg_at_fixed_j6'] for c in checks) == 15
    assert all(c['valid'] for c in checks)
    changed = copy.deepcopy(plan)
    changed['poses'][0]['j1_deg'] = 0
    with pytest.raises(ValueError, match='only'):
        core.validate_plan(changed, model)


def test_plan_rejects_duplicates_and_bad_angles(model, plan):
    bad = copy.deepcopy(plan)
    bad['poses'][0]['j7_deg'] = 85
    with pytest.raises(ValueError):
        core.validate_plan(bad, model)
    bad = copy.deepcopy(plan)
    bad['poses'][1] = dict(bad['poses'][0])
    with pytest.raises(ValueError):
        core.validate_plan(bad, model)
    bad = copy.deepcopy(plan)
    bad['angle_convention'] = 'relative'
    with pytest.raises(ValueError):
        core.validate_plan(bad, model)


def test_modified_model_rejected(config, tmp_path):
    p = tmp_path / 'model'
    p.write_text(Path(config['model_file']).read_text() + '\n')
    with pytest.raises(ValueError, match='hash'):
        core.Model.load(p)


def test_recover_mass_com_and_nonzero_bias_with_noise(truth):
    g = random_gravity()
    w = core.predict(truth, g)
    rng = np.random.default_rng(12)
    w += rng.normal(size=w.shape) * np.array([1e-4] * 3 + [1e-6] * 3)
    result = core.solve(g[:12], w[:12])
    assert result['mass_kg'] == pytest.approx(truth['mass_kg'], abs=1e-5)
    assert result['com_sensor_m'] == pytest.approx(truth['com_sensor_m'], abs=1e-6)
    assert result['force_bias_N'] == pytest.approx(truth['force_bias_N'], abs=1e-4)
    assert result['torque_bias_Nm'] == pytest.approx(truth['torque_bias_Nm'], abs=2e-6)
    report = core.residual_report(result, g[12:], w[12:])
    assert report['force_vector_rms_N'] < .001
    assert report['torque_vector_rms_Nm'] < .00001


def test_degenerate_small_angle_and_negative_mass_rejected(truth):
    repeated = np.tile([0, 0, -9.81], (12, 1))
    with pytest.raises(ValueError, match='coverage'):
        core.solve(repeated, core.predict(truth, repeated))
    g = random_gravity(12)
    negative = dict(truth, mass_kg=-.63)
    with pytest.raises(ValueError, match='mass'):
        core.solve(g, core.predict(negative, g))
    almost_same = repeated + np.linspace(-.0001, .0001, 12)[:, None] * np.array([1., 1., 0.])
    with pytest.raises(ValueError, match='coverage'):
        core.solve(almost_same, core.predict(truth, almost_same))


def test_coverage_invariant_to_fixed_sensor_mount_rotation():
    g = random_gravity()
    a = core.coverage(g)
    b = core.coverage(g @ np.array([[0., 1, 0], [0, 0, 1], [1, 0, 0]]))
    for k in a:
        assert a[k] == pytest.approx(b[k])


def test_coordinates_require_explicit_confirmation(config, model):
    with pytest.raises(ValueError, match='Confirm sensor'):
        core.coordinates(config, model)
    config['sensor'].update(axes_confirmed=True, R_flange_sensor=np.eye(3).tolist(),
                            force_sign=1, torque_sign=1, frame_id='right_ft')
    with pytest.raises(ValueError, match='gravity'):
        core.coordinates(config, model)
    config['gravity']['base_direction_confirmed'] = True
    r, g, s = core.coordinates(config, model)
    assert g.tolist() == [0., -9.81, 0.]
    assert s.tolist() == [1] * 6
    config['sensor']['R_flange_sensor'][0][0] = -1
    with pytest.raises(ValueError, match='proper rotation'):
        core.coordinates(config, model)


def row(i, t=None, q=None, wrench=None):
    w = np.zeros(6) if wrench is None else np.array(wrench)
    return {'frame': i, 'monotonic_s': i * .01 if t is None else t,
            'host_time_ns': i * 10000000, 'channel': 216,
            'joint_deg': [0] * 5 + [-40, -50] if q is None else q,
            'velocity_deg_s': [0] * 7, 'wrench_si': w.tolist(),
            'wrench_raw': (w * 10000).tolist()}


def test_sample_window_waits_for_settle_and_unique_fresh_frames(model, plan, config):
    window = core.SampleWindow(plan['poses'][0], [0] * 5, model, config['sampling'], 10)
    accepted = [i for i in range(302) if window.feed(row(i))]
    assert accepted and accepted[0] >= 300
    assert len(window.rows) >= 200
    window.feed(row(301, t=3.5))  # duplicate old frame is not a fresh measurement
    assert window.rows == []


@pytest.mark.parametrize('fault', ['moving', 'base_moved', 'wrong_target', 'drift', 'channel', 'nan', 'gap'])
def test_bad_samples_reset_or_reject_window(model, plan, config, fault):
    window = core.SampleWindow(plan['poses'][0], [0] * 5, model, config['sampling'], 10)
    for i in range(150):
        window.feed(row(i))
    r = row(150)
    if fault == 'moving':
        r['velocity_deg_s'][6] = 1
    elif fault == 'base_moved':
        r['joint_deg'][0] = 1
    elif fault == 'wrong_target':
        r['joint_deg'][6] += 5
    elif fault == 'drift':
        r['joint_deg'][6] += .2
    elif fault == 'channel':
        r['channel'] = 217
    elif fault == 'nan':
        r['wrench_si'][0] = float('nan')
    else:
        r['monotonic_s'] += .5
    if fault in ['channel', 'nan']:
        with pytest.raises(ValueError):
            window.feed(r)
    else:
        assert not window.feed(r)
        assert window.rows == []


def test_force_noise_rejected(model, plan, config):
    window = core.SampleWindow(plan['poses'][0], [0] * 5, model, config['sampling'], 10)
    for i in range(302):
        assert not window.feed(row(i, wrench=[(-1.) ** i, 0, 0, 0, 0, 0]))


def test_cli_default_plan_and_unapproved_collect_never_connect(tmp_path, capsys):
    assert cli.main([]) == 0
    assert cli.main(['plan']) == 0
    assert cli.main(['collect', '--zero-note', 'not used', '--output', str(tmp_path / 'unused')]) == 2
    assert not (tmp_path / 'unused').exists()
    assert 'requires --connect' in capsys.readouterr().err
    with pytest.raises(ValueError):
        sdk.Feedback('/unused', '192.168.1.190')


@pytest.fixture
def synthetic_session(tmp_path, config, model, plan, truth):
    # Synthetic numerical test posture, not a proposed J1-J5 robot posture.
    base = [-20., -30., 50., -75., 10.]
    kine = sdk.Kinematics(config['sdk_root'], model)
    r_fs = np.array([[1., 0, 0], [0, 0, -1.], [0, 1., 0]])
    config['sensor'].update(axes_confirmed=True, frame_id='synthetic_sensor',
                            R_flange_sensor=r_fs.tolist(), force_sign=-1, torque_sign=1)
    config['gravity']['base_direction_confirmed'] = True
    signs = np.array([-1] * 3 + [1] * 3)
    folder = tmp_path / 'session'
    folder.mkdir()
    session = {'schema_version': 1, 'kind': 'synthetic_test', 'model_sha256': model.digest,
               'base5_deg': base, 'config': config, 'plan': plan,
               'zero_note': 'SYNTHETIC DATA ONLY', 'captures': []}
    rng = np.random.default_rng(22)
    for index, p in enumerate(plan['poses']):
        q = base + [p['j6_deg'], p['j7_deg']]
        g = r_fs.T @ kine.rotation(q).T @ model.gravity
        w = core.predict(truth, g) * signs
        rows = [row(i + index * 1000, t=100 + index * 5 + i * .01, q=q,
                    wrench=w + rng.normal(size=6) * [1e-3, 1e-3, 1e-3, 1e-5, 1e-5, 1e-5])
                for i in range(201)]
        file = folder / (p['id'] + '.jsonl')
        file.write_text(''.join(json.dumps(r) + '\n' for r in rows))
        session['captures'].append({'id': p['id'], 'raw_file': file.name, 'raw_sha256': core.sha256(file)})
    core.write_json(folder / 'session.json', session)
    config_path = tmp_path / 'config.yaml'
    core.write_yaml(config_path, config)
    return folder, config_path, session


def test_offline_sdk_fk_and_full_replay_fit(synthetic_session, tmp_path, truth):
    folder, cfg, session = synthetic_session
    out = tmp_path / 'fit'
    current = tmp_path / 'current.yaml'
    current.write_text('previous real calibration')
    assert cli.main(['--config', str(cfg), 'fit', str(folder), '--output', str(out),
                     '--current-file', str(current)]) == 0
    assert current.read_text() == 'previous real calibration'
    result = core.read_yaml(out / 'synthetic_calibration.yaml')
    p = result['parameters']
    assert p['mass_kg'] == pytest.approx(truth['mass_kg'], abs=2e-5)
    assert p['com_sensor_m'] == pytest.approx(truth['com_sensor_m'], abs=2e-5)
    assert result['validation_passed']
    assert len(result['residuals']['calibration']['pose_ids']) == 12
    assert len(result['residuals']['validation']['pose_ids']) == 6
    assert not (out / 'calibration.yaml').exists()  # synthetic cannot masquerade as real


def test_independent_validation_failure_is_not_fitted_away(synthetic_session, tmp_path):
    folder, cfg, session = synthetic_session
    for capture in session['captures'][12:]:
        file = folder / capture['raw_file']
        rows = [json.loads(s) for s in file.read_text().splitlines()]
        for r in rows:
            r['wrench_si'][0] += 3
            r['wrench_raw'][0] += 30000
        file.write_text(''.join(json.dumps(r) + '\n' for r in rows))
        capture['raw_sha256'] = core.sha256(file)
    core.write_json(folder / 'session.json', session)
    out = tmp_path / 'failed'
    current = tmp_path / 'current.yaml'
    current.write_text('previous real calibration')
    assert cli.main(['--config', str(cfg), 'fit', str(folder), '--output', str(out),
                     '--current-file', str(current)]) == 2
    assert current.read_text() == 'previous real calibration'
    report = json.loads((out / 'report.json').read_text())
    assert report['residuals']['calibration']['force_vector_rms_N'] < .001
    assert report['residuals']['validation']['force_vector_rms_N'] > 2.9
    assert not (out / 'synthetic_calibration.yaml').exists()
    assert not (out / 'calibration.yaml').exists()


@pytest.mark.parametrize('publish', [False, True])
def test_real_fit_updates_current_only_when_requested(synthetic_session, tmp_path, publish):
    # Exercise the real-file publishing branch using generated data only in tmp_path.
    folder, cfg, session = synthetic_session
    session['kind'] = 'real_telemetry'
    core.write_json(folder / 'session.json', session)
    current = tmp_path / 'current.yaml'
    current.write_text('previous calibration')
    out = tmp_path / 'fit'
    args = ['--config', str(cfg), 'fit', str(folder), '--output', str(out)]
    if publish:
        args += ['--current-file', str(current)]
    assert cli.main(args) == 0
    if publish:
        assert current.read_bytes() == (out / 'calibration.yaml').read_bytes()
    else:
        assert current.read_text() == 'previous calibration'


def test_current_calibration_replaces_previous_without_changing_archive(tmp_path):
    source = ROOT / 'test/fixtures/right_ft_calibration_20261016_03.yaml'
    original = source.read_bytes()
    current = tmp_path / 'current/calibration.yaml'
    assert cli.publish_current_calibration(source, current) == current
    assert current.read_bytes() == original
    next_result = core.read_yaml(source)
    next_result['parameters']['mass_kg'] = .51
    next_source = tmp_path / 'next.yaml'
    core.write_yaml(next_source, next_result)
    cli.publish_current_calibration(next_source, current)
    assert current.read_bytes() == next_source.read_bytes()
    assert source.read_bytes() == original


@pytest.mark.parametrize('change', [{'validation_passed': False}, {'kind': 'synthetic_test'},
                                   {'schema_version': 2}])
def test_current_calibration_rejects_unusable_results(tmp_path, change):
    data = core.read_yaml(ROOT / 'test/fixtures/right_ft_calibration_20261016_03.yaml')
    data.update(change)
    source, current = tmp_path / 'result.yaml', tmp_path / 'current.yaml'
    core.write_yaml(source, data)
    current.write_text('previous calibration')
    with pytest.raises(ValueError, match='validated real'):
        cli.publish_current_calibration(source, current)
    assert current.read_text() == 'previous calibration'


def test_current_calibration_atomic_replace_failure_preserves_previous(tmp_path, monkeypatch):
    source = ROOT / 'test/fixtures/right_ft_calibration_20261016_03.yaml'
    current = tmp_path / 'current.yaml'
    current.write_text('previous calibration')
    def fail_replace(*args):
        raise OSError('simulated replace failure')
    monkeypatch.setattr(cli.os, 'replace', fail_replace)
    with pytest.raises(OSError, match='replace failure'):
        cli.publish_current_calibration(source, current)
    assert current.read_text() == 'previous calibration'
    assert list(tmp_path.iterdir()) == [current]


def test_replay_rejects_tampered_samples(synthetic_session, tmp_path):
    folder, cfg, session = synthetic_session
    file = folder / session['captures'][0]['raw_file']
    file.write_text(file.read_text() + '\n')
    assert cli.main(['--config', str(cfg), 'fit', str(folder), '--output', str(tmp_path / 'fit')]) == 2


def test_telemetry_decoder_uses_only_right_six_channels():
    arm = SimpleNamespace(m_OutFrameSerial=42, m_FB_Joint_Pos=list(range(7)),
                          m_LowSpdFlag=1,
                          m_FB_Joint_Vel=[0.] * 7, m_EST_Joint_Firc=[216] * 7,
                          m_EST_Joint_Firc_Dot=[10000, -20000, 30000, 4000, -5000, 6000, 999])
    d = SimpleNamespace(m_Out=[None, arm], m_State=[None, SimpleNamespace(m_ERRCode=0, m_CurState=1)],
                        m_In=[None, SimpleNamespace(m_Joint_Vel_Ratio=10, m_Joint_Acc_Ratio=10,
                                                    m_Joint_CMD_Pos=list(range(7)))])
    result = sdk.feedback_row(d, 123, 12.3)
    assert result['wrench_si'] == pytest.approx([1, -2, 3, .4, -.5, .6])
    assert result['mode'] == 1 and result['velocity_ratio'] == result['acceleration_ratio'] == 10
    d.m_State[1].m_ERRCode = 1
    with pytest.raises(RuntimeError):
        sdk.feedback_row(d, 123, 12.3)


class FakeFunction:
    def __init__(self, name, calls, callback):
        self.name, self.calls, self.callback = name, calls, callback

    def __call__(self, *args):
        self.calls.append((self.name, args))
        return self.callback(*args)


@pytest.mark.parametrize('fault', [None, 'select', 'link', 'body'])
def test_feedback_adapter_allowlist_and_release_without_real_sdk(monkeypatch, config, fault):
    calls, frame = [], [0]

    def buf(ptr):
        frame[0] += 1
        ptr._obj.m_Out[1].m_OutFrameSerial = frame[0]
        ptr._obj.m_Out[1].m_EST_Joint_Firc[0] = 216
        return True

    callbacks = {'CheckSDKTypeCompat': lambda _: 0, 'OnGetSDKVersion': lambda: 100343014,
                 'OnLinkTo': lambda *args: fault != 'link', 'OnRelease': lambda: True,
                 'OnGetBuf': buf, 'OnSetUserSpcfData_B': lambda _: fault != 'select'}
    fake = SimpleNamespace(**{n: FakeFunction(n, calls, f) for n, f in callbacks.items()})
    monkeypatch.setattr(ctypes, 'CDLL', lambda _: fake)
    client = sdk.Feedback(config['sdk_root'], '192.168.1.190', connect=True)
    if fault:
        with pytest.raises(RuntimeError):
            with client:
                if fault == 'body':
                    raise RuntimeError('injected failure')
    else:
        with client:
            assert client.poll()['channel'] == 216
    names = [n for n, _ in calls]
    assert names[-1] == 'OnRelease'
    assert names.count('OnRelease') == 1
    assert set(names) <= set(callbacks)
    for n, args in calls:
        if n == 'OnSetUserSpcfData_B':
            assert args == (216,)


@pytest.mark.parametrize('ending', ['complete', 'quit', 'exception'])
def test_interactive_collection_with_simulated_feedback_only(
        monkeypatch, tmp_path, config, plan, model, truth, ending):
    config['sampling'].update(settle_s=.02, duration_s=.06, min_samples=5, poll_hz=100)
    config['sensor'].update(axes_confirmed=True, R_flange_sensor=np.eye(3).tolist(),
                            force_sign=1, torque_sign=1, frame_id='synthetic_sensor')
    config['gravity']['base_direction_confirmed'] = True
    cfg = tmp_path / 'config.yaml'
    core.write_yaml(cfg, config)
    base = [-20., -30., 50., -75., 10.]  # numerical fixture, not a robot command
    state = {'t': 100., 'index': -1, 'frame': 0, 'closed': False}
    kine = sdk.Kinematics(config['sdk_root'], model)

    def fake_input(prompt):
        state['index'] += 1
        if ending == 'quit' and state['index'] == 1:
            return 'q'
        return ''

    class SimulatedFeedback:
        def __init__(self, *args, **kwargs):
            assert kwargs['connect']
            self.initial = {'joint_deg': base + [-40, -50]}

        def __enter__(self):
            return self

        def poll(self):
            if ending == 'exception' and state['index'] == 1:
                raise RuntimeError('simulated read failure')
            pose = plan['poses'][state['index']]
            q = base + [pose['j6_deg'], pose['j7_deg']]
            w = core.predict(truth, kine.rotation(q).T @ model.gravity)
            state['frame'] += 1
            return row(state['frame'], t=state['t'], q=q, wrench=w)

        def __exit__(self, *_):
            state['closed'] = True

    monkeypatch.setattr(sdk, 'Feedback', SimulatedFeedback)
    monkeypatch.setattr(cli.sys, 'stdin', SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr('builtins.input', fake_input)
    monkeypatch.setattr(cli.time, 'monotonic', lambda: state['t'])
    monkeypatch.setattr(cli.time, 'sleep', lambda dt: state.update(t=state['t'] + dt))
    out = tmp_path / 'collected'
    code = cli.main(['--config', str(cfg), 'collect', '--connect', '--poses-reviewed',
                     '--zero-note', 'TEST WITH SIMULATED SDK', '--output', str(out)])
    assert code == (2 if ending == 'exception' else 0)
    assert state['closed']
    data = json.loads((out / 'session.json').read_text())
    assert len(data['captures']) == (18 if ending == 'complete' else 1)
    assert data['status'] == {'complete': 'complete', 'quit': 'operator_stopped',
                              'exception': 'failed'}[ending]
    for c in data['captures']:
        assert c['raw_sha256'] == core.sha256(out / c['raw_file'])
    if ending == 'complete':
        # Mark simulated input explicitly before testing the real-data replay path.
        data['kind'] = 'synthetic_test'
        core.write_json(out / 'session.json', data)
        assert cli.main(['--config', str(cfg), 'fit', str(out), '--output', str(tmp_path / 'fitted')]) == 0


def test_position_adapter_only_stages_right_targets_and_ten_percent(monkeypatch, config):
    calls, packets, staged = [], [], {}

    def buf(ptr):
        arm = ptr._obj.m_Out[1]
        arm.m_OutFrameSerial += 1
        arm.m_EST_Joint_Firc[0] = 216
        arm.m_LowSpdFlag = b'\x01'
        return True

    def clear():
        staged.clear()
        return True

    def limits(velocity, acceleration):
        staged['limits'] = (velocity, acceleration)
        return True

    def target(q):
        staged['target'] = list(q)
        return True

    def mode(state):
        staged['mode'] = state
        return True

    def send(timeout):
        assert timeout == 100
        packets.append(copy.deepcopy(staged))
        return 0  # Real SDK can return 0; runner must confirm actual feedback separately.

    def stop(name, value):
        assert name.raw == b'RSTA1' + b'\0' * 25
        assert value == 0
        return 0

    callbacks = {'CheckSDKTypeCompat': lambda _: 0, 'OnGetSDKVersion': lambda: 100343014,
                 'OnLinkTo': lambda *args: True, 'OnRelease': lambda: True,
                 'OnGetBuf': buf, 'OnSetUserSpcfData_B': lambda _: True,
                 'OnClearSet': clear, 'OnSetJointLmt_B': limits,
                 'OnSetJointCmdPos_B': target, 'OnSetTargetState_B': mode,
                 'OnSetSendWaitResponse': send, 'OnSetIntPara': stop}
    fake = SimpleNamespace(**{n: FakeFunction(n, calls, f) for n, f in callbacks.items()})
    monkeypatch.setattr(ctypes, 'CDLL', lambda _: fake)
    with sdk.PositionFeedback(config['sdk_root'], config['robot_ip'], connect=True) as client:
        assert client.initial['low_speed'] == 1  # Vendor ctypes uses c_char, not c_ubyte.
        client.position([0.] * 7, enter=True)
        client.position([0.] * 5 + [-40., -50.])
        client.stop()
    assert packets == [{'limits': (10, 10), 'target': [0.] * 7, 'mode': 1},
                       {'limits': (10, 10), 'target': [0.] * 5 + [-40., -50.]}]
    names = [n for n, _ in calls]
    assert set(names) <= set(callbacks)
    assert names[-1] == 'OnRelease' and names.count('OnRelease') == 1


@pytest.mark.parametrize('fault', [None, 'automatic_fit', 'all_zero', 'frozen_sensor', 'nonzero_start',
                                  'moving_start', 'initial_mode', 'mode_changed',
                                  'ratio_changed', 'target_changed', 'no_feedback',
                                  'frozen_frame', 'read_failure', 'command_failure',
                                  'interrupt', 'move_timeout', 'echo_timeout',
                                  'base_moved', 'wrist_violation', 'sample_timeout',
                                  'stop_failure', 'stop_feedback_lost', 'logging_failure',
                                  'mode_timeout', 'sigterm', 'target_changed_between',
                                  'enable_velocity_spike', 'startup_velocity_spike',
                                  'startup_never_stationary'])
def test_automatic_sequence_with_simulated_motion_only(
        monkeypatch, tmp_path, config, plan, model, truth, fault):
    config['sampling'].update(settle_s=.02, duration_s=.06, min_samples=5, poll_hz=100,
                              timeout_s=.5)
    config['automatic'].update(preflight_s=.06, mode_timeout_s=.2, command_timeout_s=.1,
                               move_timeout_s=.3, stop_timeout_s=.5, sensor_stale_s=.4)
    config['sensor'].update(axes_confirmed=True, force_sign=1, torque_sign=1,
                            frame_id='synthetic_sensor')
    config['gravity']['base_direction_confirmed'] = True
    config['current_calibration_file'] = str(tmp_path / 'current.yaml')
    cfg = tmp_path / 'config.yaml'
    core.write_yaml(cfg, config)
    kine = sdk.Kinematics(config['sdk_root'], model)
    r_fs = np.array(config['sensor']['R_flange_sensor'])
    state = {'t': 100., 'frame': 0, 'mode': 0, 'q': np.zeros(7), 'target': np.zeros(7),
             'remaining': 0, 'commands': [], 'stops': 0, 'closed': False, 'stopping': False,
             'polls_at_target': 0, 'hold_polls': 0, 'last_hold_spike': None}

    class SimulatedPosition:
        def __init__(self, *args, **kwargs):
            assert kwargs['connect']

        def __enter__(self):
            return self

        def position(self, target, *, enter=False):
            target = np.array(target)
            assert np.all(target[:5] == 0)
            if len(state['commands']) == 1 and state['last_hold_spike'] is not None:
                assert state['t'] - state['last_hold_spike'] >= config['sampling']['settle_s']
            state['commands'].append({'target': target.tolist(), 'enter': enter})
            if fault in ('command_failure', 'stop_failure', 'stop_feedback_lost') and len(state['commands']) == 3:
                raise RuntimeError('simulated send failure')
            state['mode'] = 1
            if fault == 'mode_timeout' and enter:
                state['mode'] = 101
            state['target'] = target
            state['remaining'] = 0 if enter else 5
            state['polls_at_target'] = 0

        def poll(self):
            if fault == 'stop_feedback_lost' and state['stopping']:
                return None
            second = len(state['commands']) >= 3 and not state['stopping']
            if second and fault == 'read_failure':
                raise RuntimeError('simulated read failure')
            if second and fault == 'interrupt':
                raise KeyboardInterrupt('simulated Ctrl+C')
            if second and fault == 'sigterm':
                signal.getsignal(signal.SIGTERM)(signal.SIGTERM, None)
            if second and fault == 'no_feedback':
                return None
            if not (second and fault == 'frozen_frame'):
                state['frame'] += 1
            previous = state['q'].copy()
            if state['remaining'] and not (second and fault == 'move_timeout'):
                state['q'] += (state['target'] - state['q']) / state['remaining']
                state['remaining'] -= 1
            q = state['q'].copy()
            w = core.predict(truth, r_fs.T @ kine.rotation(q).T @ model.gravity)
            w += np.sin(state['frame']) * 1e-6  # Sub-resolution synthetic noise, not hardware.
            if not state['commands']:
                if fault == 'all_zero':
                    w[:] = 0
                if fault == 'frozen_sensor':
                    w[:] = .01
                if fault == 'nonzero_start':
                    q[6] = 2
            result = row(state['frame'], t=state['t'], q=q.tolist(), wrench=w)
            result.update(mode=state['mode'], low_speed=int(state['remaining'] == 0),
                          velocity_ratio=10, acceleration_ratio=10,
                          command_deg=state['target'].tolist())
            result['velocity_deg_s'] = ((q - previous) / .01).tolist()
            if len(state['commands']) == 1 and not state['stopping']:
                state['hold_polls'] += 1
                spike = ((fault == 'enable_velocity_spike' and state['hold_polls'] == 1) or
                         (fault == 'startup_velocity_spike' and state['hold_polls'] == 2) or
                         (fault == 'startup_never_stationary' and state['hold_polls'] >= 2))
                if spike:
                    # Actual failure: the first pre-dispatch frame after Position
                    # confirmation had J2=-0.300107 deg/s, with tiny pose change.
                    result['velocity_deg_s'][1] = -.3001070022583008
                    result['joint_deg'][1] -= .000103
                    state['last_hold_spike'] = state['t']
            if not state['commands'] and fault == 'moving_start':
                result['velocity_deg_s'][6] = .3
            if not state['commands'] and fault == 'initial_mode':
                result['mode'] = 4
            if second:
                state['polls_at_target'] += 1
                if fault == 'mode_changed':
                    result['mode'] = 2
                if fault == 'ratio_changed':
                    result['acceleration_ratio'] = 100
                if fault == 'target_changed' and state['polls_at_target'] > 2:
                    result['command_deg'][6] += 1
                if fault == 'echo_timeout':
                    result['command_deg'][6] += 1
                if fault == 'base_moved':
                    result['joint_deg'][0] = 1
                if fault == 'wrist_violation':
                    result['joint_deg'][5:] = [60, 80]
                if fault == 'sample_timeout' and state['remaining'] == 0:
                    result['wrench_si'][0] += 10 * (state['frame'] % 2)
                    result['wrench_raw'][0] = result['wrench_si'][0] * 10000
            return result

        def stop(self):
            state['stops'] += 1
            state['stopping'] = True
            state['remaining'] = 0
            state['target'] = state['q'].copy()
            if fault == 'stop_failure':
                raise RuntimeError('simulated stop acknowledgement failure')

        def __exit__(self, *_):
            state['closed'] = True

    monkeypatch.setattr(sdk, 'PositionFeedback', SimulatedPosition)
    if fault == 'target_changed_between':
        actual_summary = cli.summarize_rows

        def summary_and_external_target(*args, **kwargs):
            summary = actual_summary(*args, **kwargs)
            state['target'][6] += 1
            return summary

        monkeypatch.setattr(cli, 'summarize_rows', summary_and_external_target)
    monkeypatch.setattr(auto.time, 'monotonic', lambda: state['t'])
    monkeypatch.setattr(auto.time, 'sleep', lambda dt: state.update(t=state['t'] + dt))
    if fault == 'logging_failure':
        actual_open = Path.open

        class FailingLog:
            def __init__(self, stream):
                self.stream, self.writes = stream, 0

            def __enter__(self):
                return self

            def __exit__(self, *_):
                self.stream.close()

            def write(self, value):
                self.writes += 1
                if self.writes >= 2:
                    raise OSError('simulated raw log write failure')
                return self.stream.write(value)

            def flush(self):
                self.stream.flush()

        def failing_open(path, *args, **kwargs):
            stream = actual_open(path, *args, **kwargs)
            return FailingLog(stream) if path.name == 'C02_attempt002.jsonl' else stream

        monkeypatch.setattr(Path, 'open', failing_open)
    out = tmp_path / 'automatic'
    if fault == 'automatic_fit':
        actual_fit = cli.fit_session

        def fit_labeled_synthetic(args, config, model):
            assert args.current_file == str(tmp_path / 'current.yaml')
            data = json.loads((out / 'session.json').read_text())
            data['kind'] = 'synthetic_test'
            core.write_json(out / 'session.json', data)
            return actual_fit(args, config, model)

        monkeypatch.setattr(cli, 'fit_session', fit_labeled_synthetic)
    args = ['--config', str(cfg), 'auto', '--connect', '--poses-reviewed',
            '--zero-note', 'SIMULATED DATA ONLY', '--output', str(out)]
    if fault != 'automatic_fit':
        args += ['--collect-only', '--current-file', str(tmp_path / 'current.yaml')]
    old_term = signal.getsignal(signal.SIGTERM)
    code = cli.main(args)
    assert not (tmp_path / 'current.yaml').exists()
    assert signal.getsignal(signal.SIGTERM) is old_term
    success = fault in (None, 'automatic_fit', 'enable_velocity_spike', 'startup_velocity_spike')
    assert code == (0 if success else 130 if fault in ('interrupt', 'sigterm') else 2)
    assert state['closed']
    data = json.loads((out / 'session.json').read_text())
    preflight_faults = ('all_zero', 'frozen_sensor', 'nonzero_start', 'moving_start', 'initial_mode')
    if fault in preflight_faults:
        assert state['commands'] == [] and state['stops'] == 0
        assert data['captures'] == []
    elif fault in ('mode_timeout', 'startup_never_stationary'):
        assert len(state['commands']) == 1 and state['stops'] == 1
        assert data['captures'] == [] and data['stop']['verified']
        if fault == 'startup_never_stationary':
            assert 'did not remain stationary' in data['error']
            assert len(data['attempts']) == 1 and not data['attempts'][0]['accepted']
    elif not success:
        assert len(state['commands']) == (2 if fault in ('target_changed_between', 'logging_failure') else 3)
        # At most hold + C01 + C02, never a later point.
        assert state['stops'] == 1
        assert data['stop']['requested']
        assert data['stop']['verified'] == (fault not in ('stop_failure', 'stop_feedback_lost'))
        assert len(data['captures']) == 1 and len(data['attempts']) == 2
        assert not data['attempts'][-1]['accepted']
    else:
        expected = [{'target': [0.] * 7, 'enter': True}]
        expected += [{'target': [0.] * 5 + [p['j6_deg'], p['j7_deg']], 'enter': False}
                     for p in plan['poses']]
        assert state['commands'] == expected and state['stops'] == 0
        assert len(data['captures']) == 18 and data['status'] == 'complete'
        assert data['control']['velocity_percent'] == data['control']['acceleration_percent'] == 10
        assert data['base5_deg'] == [0.] * 5
        # Real automatic collection defaults to fit. Explicitly label synthetic
        # observations before calling the actual offline fit in this test.
        if fault == 'automatic_fit':
            result_dir = out / 'result'
        else:
            data['kind'] = 'synthetic_test'
            core.write_json(out / 'session.json', data)
            result_dir = tmp_path / 'fitted'
            assert cli.main(['--config', str(cfg), 'fit', str(out), '--output', str(result_dir)]) == 0
        fitted = core.read_yaml(result_dir / 'synthetic_calibration.yaml')
        assert fitted['parameters']['mass_kg'] == pytest.approx(truth['mass_kg'], abs=1e-5)
        assert fitted['parameters']['com_sensor_m'] == pytest.approx(truth['com_sensor_m'], abs=1e-5)
    for capture in data['captures']:
        assert capture['raw_sha256'] == core.sha256(out / capture['raw_file'])


@pytest.mark.parametrize('at_first_pose', [False, True])
@pytest.mark.parametrize('fault,match', [
    ('transient_speed', None), ('transient_low_speed', None),
    ('persistent_speed', 'did not remain stationary'),
    ('persistent_low_speed', 'did not remain stationary'),
    ('target_changed', 'target was changed'), ('drift', 'drifted'),
    ('mode_changed', 'Position mode'), ('ratio_changed', 'Position mode'),
    ('feedback_lost', 'feedback stopped'),
])
def test_pre_dispatch_hold_requires_continuous_stability_and_keeps_guards(
        monkeypatch, config, plan, model, at_first_pose, fault, match):
    config['sampling'].update(settle_s=.04, poll_hz=100)
    config['automatic']['mode_timeout_s'] = .3
    target = [0.] * 5 + ([-40., -50.] if at_first_pose else [0., 0.])
    state = {'t': 100., 'frame': 0, 'last_unstable': None, 'first_stable_after': None}

    def poll():
        if fault == 'feedback_lost':
            return None
        state['frame'] += 1
        r = row(state['frame'], t=state['t'], q=list(target),
                wrench=[state['frame'] * 1e-5] * 6)
        r.update(mode=1, low_speed=1, velocity_ratio=10, acceleration_ratio=10,
                 command_deg=list(target))
        transient = state['frame'] in (1, 4)
        if fault == 'persistent_speed' or (fault == 'transient_speed' and transient):
            r['velocity_deg_s'][1] = -.3001070022583008
        if fault == 'persistent_low_speed' or (fault == 'transient_low_speed' and transient):
            r['low_speed'] = 0
        if fault == 'target_changed':
            r['command_deg'][6] += .01
        if fault == 'drift':
            r['joint_deg'][5] += state['frame'] * .04
            r['velocity_deg_s'][5] = 4.
        if fault == 'mode_changed':
            r['mode'] = 2
        if fault == 'ratio_changed':
            r['velocity_ratio'] = 100
        if not r['low_speed'] or max(abs(v) for v in r['velocity_deg_s']) > .2:
            state['last_unstable'] = state['t']
            state['first_stable_after'] = None
        elif state['first_stable_after'] is None:
            state['first_stable_after'] = state['t']
        return r

    monkeypatch.setattr(auto.time, 'monotonic', lambda: state['t'])
    monkeypatch.setattr(auto.time, 'sleep', lambda dt: state.update(t=state['t'] + dt))
    # No position/stop methods are available here; waiting cannot send commands.
    runner = auto.AutoSequence(SimpleNamespace(poll=poll), config, plan, model)
    runner.expected_target = list(target)
    runner.latest = row(0, t=99.99, q=list(target))
    observed = []
    if match:
        with pytest.raises(RuntimeError, match=match):
            runner.wait_previous_target(observed.append)
    else:
        runner.wait_previous_target(observed.append)
        assert observed[-1]['monotonic_s'] - state['first_stable_after'] >= .04
        assert state['last_unstable'] is not None
        assert runner.stationary(observed[-1])
    assert state['t'] - 100. <= .31


@pytest.mark.parametrize('fault', ['ratios', 'coordinates'])
def test_automatic_preconnect_checks_never_load_control_library(config, tmp_path, fault):
    cfg = tmp_path / 'config.yaml'
    if fault == 'ratios':
        config['automatic']['velocity_percent'] = 100
    core.write_yaml(cfg, config)
    args = ['--config', str(cfg), 'auto', '--zero-note', 'TEST', '--output', str(tmp_path / 'run')]
    if fault != 'coordinates':
        args += ['--collect-only']
    assert cli.main(args) == 2
    assert not (tmp_path / 'run').exists()


def test_simple_auto_defaults_and_unique_output_without_preparation(monkeypatch, tmp_path):
    monkeypatch.setattr(Path, 'home', lambda: tmp_path)
    calls = []
    def fake_automatic(args, config, plan, model):
        assert args.connect and args.poses_reviewed
        assert Path(args.config).name == 'right_ft_calibration_robot.yaml'
        assert config['automatic']['velocity_percent'] == 10
        assert config['automatic']['acceleration_percent'] == 10
        assert config['sensor']['axes_confirmed'] is True
        assert 'hardware_zero_reference' not in config['sensor']
        assert len(plan['poses']) == 18
        assert args.zero_note
        assert args.current_file == config['current_calibration_file']
        assert Path(args.output).parent == tmp_path / 'ft_calibration_runs'
        calls.append(args.output)
        return 0
    monkeypatch.setattr(cli, 'automatic', fake_automatic)
    assert cli.main(['auto']) == 0
    assert cli.main(['auto']) == 0
    assert calls[0] != calls[1]
    assert list(tmp_path.iterdir()) == []


def test_auto_still_accepts_explicit_output_and_note(monkeypatch, tmp_path):
    def fake_automatic(args, config, plan, model):
        assert args.output == str(tmp_path / 'custom')
        assert args.zero_note == 'custom note'
        assert args.current_file == str(tmp_path / 'current.yaml')
        return 0
    monkeypatch.setattr(cli, 'automatic', fake_automatic)
    assert cli.main(['auto', '--output', str(tmp_path / 'custom'),
                     '--zero-note', 'custom note', '--current-file', str(tmp_path / 'current.yaml')]) == 0


def test_config_paths_follow_home_and_yaml_directory(monkeypatch, tmp_path):
    home = tmp_path / 'new-user'
    monkeypatch.setenv('HOME', str(home))
    folder = tmp_path / 'different-checkout' / 'config'
    folder.mkdir(parents=True)
    path = folder / 'robot.yaml'
    core.write_yaml(path, {'sdk_root': '~/TJ_FX_ROBOT_CONTRL_SDK',
                          'model_file': '~/TJ_FX_ROBOT_CONTRL_SDK/CommonConfig/model.cfg',
                          'current_calibration_file': 'right_ft_calibration_current.yaml'})
    monkeypatch.chdir(tmp_path)
    config = core.read_yaml(path)
    assert config['sdk_root'] == str(home / 'TJ_FX_ROBOT_CONTRL_SDK')
    assert config['model_file'] == str(home / 'TJ_FX_ROBOT_CONTRL_SDK/CommonConfig/model.cfg')
    assert config['current_calibration_file'] == str(folder / 'right_ft_calibration_current.yaml')
    assert core.read_yaml(POSES)['poses'][0]['id'] == 'C01'


@pytest.mark.parametrize('kind', ['control', 'kinematics'])
def test_reviewed_sdk_builds_accepted_unknown_build_rejected(tmp_path, monkeypatch, kind):
    allowed = sdk.CONTROL_HASHES if kind == 'control' else sdk.KINEMATICS_HASHES
    relative = 'SDK_PYTHON/libMarvinSDK.so' if kind == 'control' else 'DEMO_C++/libKine.so'
    path = tmp_path / relative
    for digest in allowed:
        monkeypatch.setattr(sdk, 'sha256', lambda _, value=digest: value)
        assert sdk.checked_file(tmp_path, relative, allowed) == path
    monkeypatch.setattr(sdk, 'sha256', lambda _: '0' * 64)
    with pytest.raises(ValueError, match='compatibility review'):
        sdk.checked_file(tmp_path, relative, allowed)
    with pytest.raises(ValueError):
        sdk.checked_file(tmp_path, relative, sdk.WRAPPER_HASH)
