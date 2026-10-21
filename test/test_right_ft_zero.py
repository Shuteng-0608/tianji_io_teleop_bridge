"""No robot/library connections: exercise the bare-sensor zero command with a fake client."""
import ctypes
import json
import socket
from types import SimpleNamespace as Obj

import numpy as np
import pytest
import yaml
from pathlib import Path

import right_ft_zero as zero
from ft_calibration_core import Model, coordinates, validate_settings


@pytest.fixture(autouse=True)
def offline_only(monkeypatch):
    def forbid(*args, **kwargs):
        raise AssertionError('No hardware connection or library loading in these tests')
    monkeypatch.setattr(ctypes, 'CDLL', forbid)
    monkeypatch.setattr(socket, 'socket', forbid)


def exercise(tmp_path, monkeypatch, fault=None):
    clock = [10.]
    monkeypatch.setattr(zero, 'time', Obj(monotonic=lambda: clock[0],
                                        sleep=lambda dt: clock.__setitem__(0, clock[0] + dt)))
    state = {'reset': False, 'calls': [], 'frame': 0}
    class Reset:
        def __call__(self, arm):
            assert arm == b'B'
            state['calls'].append('zero_B')
            state['reset'] = True
            if fault == 'reset_exception':
                raise RuntimeError('reset outcome unknown')
            return fault != 'reset_false'
    class Feedback:
        def __init__(self, root, ip, *, connect):
            assert connect
            self.initial = {'mode': 0, 'frame': 0}
            self.lib = Obj(OnReset6DofForceSensor=Reset())
        def __enter__(self):
            state['calls'].append('connect')
            return self
        def __exit__(self, *_):
            state['calls'].append('release')
        def poll(self):
            state['frame'] += 1
            if fault == 'gap':
                return None
            if fault == 'interrupt_after' and state['reset']:
                raise KeyboardInterrupt()
            raw = (np.zeros(6) if state['reset'] else np.array([1, 2, 3, .03, .04, .05])) * 10000
            raw += (state['frame'] % 2) * np.array([10, 10, 10, 1, 1, 1])
            row = {'frame': state['frame'], 'monotonic_s': clock[0], 'channel': 216,
                   'mode': 0, 'low_speed': 1, 'joint_deg': [0.] * 7, 'velocity_deg_s': [0.] * 7,
                   'wrench_raw': raw.tolist(), 'wrench_si': (raw * .0001).tolist()}
            if fault == 'frozen_frame':
                row['frame'] = 1
            elif fault == 'zero_signal':
                row['wrench_raw'] = row['wrench_si'] = [0.] * 6
            elif fault == 'frozen_signal':
                row['wrench_raw'], row['wrench_si'] = [100.] * 6, [.01] * 6
            elif fault == 'channel':
                row['channel'] = 200
            elif fault == 'mode':
                row['mode'] = 2
            elif fault == 'velocity':
                row['velocity_deg_s'][0] = .3
            elif fault == 'position':
                row['joint_deg'][0] = 1.
            elif fault == 'low_speed':
                row['low_speed'] = 0
            elif fault == 'nan':
                row['wrench_si'][0] = float('nan')
            elif fault == 'unstable':
                row['wrench_si'][0] += state['frame'] % 2
            elif fault == 'residual' and state['reset']:
                row['wrench_si'][2] += 1.
            elif fault == 'pose_changed' and state['reset']:
                row['joint_deg'][0] = .3
            elif fault == 'after_mode' and state['reset']:
                row['mode'] = 1
            return row
    args = Obj(connect=True, no_tool=True, output=str(tmp_path / 'zero'), sdk_root='unused', robot_ip='unused')
    return args, Feedback, state


def test_success_verifies_before_after_and_only_calls_right_reset_once(tmp_path, monkeypatch):
    args, feedback, state = exercise(tmp_path, monkeypatch)
    assert zero.run_zero(args, feedback) == 0
    result = json.loads((Path(args.output) / 'summary.json').read_text())
    assert result['zero_verified'] and result['reset_return_is_not_acknowledgement']
    assert result['before']['mean_si'][0] > .9
    assert result['after_settled']['mean_si'][0] < .01
    assert result['after_settled']['duration_s'] > 4.9
    assert result['reset_api_invocations'] == 1
    assert state['calls'] == ['connect', 'zero_B', 'release']
    assert not result['motion_commanded'] and not result['mode_commanded']


@pytest.mark.parametrize('fault', ['gap', 'frozen_frame', 'zero_signal', 'frozen_signal', 'channel',
                                  'mode', 'velocity', 'position', 'low_speed', 'nan', 'unstable'])
def test_bad_preflight_never_resets(tmp_path, monkeypatch, fault):
    args, feedback, state = exercise(tmp_path, monkeypatch, fault)
    with pytest.raises((RuntimeError, ValueError)):
        zero.run_zero(args, feedback)
    result = json.loads((Path(args.output) / 'summary.json').read_text())
    assert not result['zero_verified'] and not result['reset_attempted']
    assert state['calls'] == ['connect', 'release']


@pytest.mark.parametrize('fault', ['residual', 'reset_false'])
def test_return_value_alone_does_not_pass_verification(tmp_path, monkeypatch, fault):
    args, feedback, state = exercise(tmp_path, monkeypatch, fault)
    assert zero.run_zero(args, feedback) == 2
    result = json.loads((Path(args.output) / 'summary.json').read_text())
    assert not result['zero_verified']
    assert state['calls'] == ['connect', 'zero_B', 'release']


@pytest.mark.parametrize('fault', ['reset_exception', 'interrupt_after', 'pose_changed', 'after_mode'])
def test_post_reset_fault_keeps_attempt_record_and_does_not_retry(tmp_path, monkeypatch, fault):
    args, feedback, state = exercise(tmp_path, monkeypatch, fault)
    with pytest.raises((RuntimeError, KeyboardInterrupt)):
        zero.run_zero(args, feedback)
    result = json.loads((Path(args.output) / 'summary.json').read_text())
    assert result['reset_attempted'] and not result['zero_verified']
    assert result['reset_api_invocations'] == 1
    assert state['calls'] == ['connect', 'zero_B', 'release']


@pytest.mark.parametrize('flag', ['connect', 'no_tool'])
def test_missing_operator_flags_block_before_connect(tmp_path, monkeypatch, flag):
    args, feedback, state = exercise(tmp_path, monkeypatch)
    setattr(args, flag, False)
    with pytest.raises(ValueError):
        zero.run_zero(args, feedback)
    assert not state['calls'] and not Path(args.output).exists()


def test_existing_output_is_preserved_without_connection(tmp_path, monkeypatch):
    args, feedback, state = exercise(tmp_path, monkeypatch)
    Path(args.output).mkdir()
    sentinel = Path(args.output) / 'summary.json'
    sentinel.write_text('preserve')
    with pytest.raises(FileExistsError):
        zero.run_zero(args, feedback)
    assert sentinel.read_text() == 'preserve' and not state['calls']


def test_reusable_template_has_current_axes_but_no_historical_bias():
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / 'config/right_ft_calibration_robot.yaml').read_text())
    validate_settings(config)
    r, gravity, signs = coordinates(config, Model.load(config['model_file']))
    np.testing.assert_array_equal(r, np.eye(3))
    np.testing.assert_array_equal(gravity, [0, -9.81, 0])
    np.testing.assert_array_equal(signs, np.ones(6))
    assert not config['sensor']['installation_bias_reference_applied']
    assert 'hardware_zero_reference' not in config['sensor']
    assert 'parameters' not in config and 'calibration_context' not in config


@pytest.mark.parametrize('fault,expected', [(None, 0), ('residual', 2)])
def test_terminal_only_zero_preserves_verification_without_files(tmp_path, monkeypatch, capsys, fault, expected):
    args, feedback, state = exercise(tmp_path, monkeypatch, fault)
    args.output = None
    assert zero.run_zero(args, feedback) == expected
    output = capsys.readouterr().out
    assert 'BEFORE' in output and 'AFTER' in output
    assert f"zero_verified: {'true' if expected == 0 else 'false'}" in output
    assert 'Summary:' not in output
    assert list(tmp_path.iterdir()) == []
    assert state['calls'] == ['connect', 'zero_B', 'release']


def test_simple_cli_runs_terminal_only_zero(tmp_path, monkeypatch):
    _, feedback, state = exercise(tmp_path, monkeypatch)
    import ft_calibration_sdk
    monkeypatch.setattr(ft_calibration_sdk, 'Feedback', feedback)
    assert zero.main([]) == 0
    assert state['calls'] == ['connect', 'zero_B', 'release']
    assert list(tmp_path.iterdir()) == []


def test_terminal_only_interruption_reports_reset_and_releases(tmp_path, monkeypatch, capsys):
    args, feedback, state = exercise(tmp_path, monkeypatch, 'interrupt_after')
    args.output = None
    with pytest.raises(KeyboardInterrupt):
        zero.run_zero(args, feedback)
    assert 'reset_attempted: True' in capsys.readouterr().out
    assert state['calls'] == ['connect', 'zero_B', 'release']
    assert list(tmp_path.iterdir()) == []
