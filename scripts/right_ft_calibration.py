#!/usr/bin/python3
"""Standalone right end-sensor calibration: offline, manual or automatic Position."""
import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

import numpy as np
import yaml

from ft_calibration_core import (
    Model, SampleWindow, coordinates, coverage, coverage_ok, finite_array, read_yaml,
    residual_report, sha256, solve, validate_plan, validate_settings,
    write_json, write_yaml,
)


def default_config_dir():
    script = Path(__file__).resolve()
    source = script.parents[1] / 'config'
    return source if source.exists() else script.parents[2] / 'share/tianji_io_teleop_bridge/config'


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def publish_current_calibration(source, destination):
    """Atomically copy a validated real result; leave the previous file on failure."""
    source = Path(source).expanduser()
    destination = Path(destination).expanduser().absolute()
    payload = source.read_bytes()
    result = yaml.safe_load(payload)
    if (not isinstance(result, dict) or result.get('schema_version') != 1 or
            result.get('kind') != 'real_telemetry' or result.get('validation_passed') is not True):
        raise ValueError('Only a validated real calibration can replace the current file')
    if source.resolve() == destination.resolve():
        raise ValueError('Current calibration destination must differ from the archived result')
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='wb', dir=destination.parent,
                                         prefix='.' + destination.name + '.', delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return destination


def plan_table(plan, model):
    lines = ['| ID | 用途 | J6 (°) | J7 (°) | 此 J6 下 J7 允许范围 (°) | J6 边界余量 (°) | J7 边界余量 (°) |',
             '|---|---|---:|---:|---|---:|---:|']
    for pose in plan['poses']:
        check = model.wrist_check([pose['j6_deg'], pose['j7_deg']], plan['wrist_margin_deg'])
        lo, hi = check['j7_allowed_deg']
        role = '标定' if pose['role'] == 'calibration' else '验证'
        lines.append(f"| {pose['id']} | {role} | {pose['j6_deg']:+g} | {pose['j7_deg']:+g} | "
                     f"[{lo:g}, {hi:g}] | {check['j6_margin_deg']:g} | "
                     f"{check['j7_margin_deg_at_fixed_j6']:g} |")
    return '\n'.join(lines)


def export_plan(plan, model, out):
    out.mkdir(parents=True, exist_ok=True)
    write_yaml(out / 'poses.yaml', plan)
    rows = []
    for pose in plan['poses']:
        check = model.wrist_check([pose['j6_deg'], pose['j7_deg']], plan['wrist_margin_deg'])
        rows.append({**{f'j{i}_deg': 0. for i in range(1, 6)}, **pose,
                     'j7_lower_deg': check['j7_allowed_deg'][0],
                     'j7_upper_deg': check['j7_allowed_deg'][1],
                     'j6_margin_deg': check['j6_margin_deg'],
                     'j7_margin_deg': check['j7_margin_deg_at_fixed_j6']})
    with (out / 'poses.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (out / 'REVIEW.md').write_text(
        '# 右臂末端六维力标定：J6/J7 姿态审核\n\n'
        '按操作者确认，自动流程的 J1～J5 全部保持 0°，只改变 J6/J7。表内为 SDK **绝对关节角（度）**，'
        '不是角度增量或末端欧拉角。12 个标定姿态，6 个独立验证姿态。\n\n'
        '当前配置约束：`|J6| <= 60°`，`|J7| <= min(90°, 110.5° - 1.025*|J6|)`。'
        'SDK 在 |J6|≤1° 时仅取单轴 J7 限位；当前系数下与上式的交集相同。\n\n'
        '按当前 SDK 模型已核查单轴和 J6/J7 组合限位；程序要求至少 10° 边界余量。'
        'J7 余量是固定 J6 时与上下边界的角度差，**不是几何距离或实际碰撞间隙**。\n\n'
        '操作者已逐点手动验证这 18 个姿态。自动程序以 Position 模式、速度/加速度 10%/10%，'
        '从七轴零位按表中顺序走点；安装最终工具后，应按现场装配确认工具、线缆及点间路径。\n\n'
        + plan_table(plan, model) + '\n\n'
        '固定 J1～J5 为零的完整七轴目标已列入 CSV。自动程序检查零位起点、目标及相邻目标的'
        '关节角直线插值的腕部约束；实际运动中也持续检查反馈关节约束。'
        '这不是几何碰撞检测，也不保证控制器的实际插值路径。拟合仍检查覆盖的秩和条件数。\n\n'
        f'模型：`{model.path}`\n\nSHA-256：`{model.digest}`\n\n'
        '限位依据：SDK `kinematicsSDK/FxRobot.cpp` 的 `lmtj67_pp/np/nn/pn` 分支，'
        '`CommonConfig/ccs_m6_40.MvKDCfg` 右臂配置。\n')
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib unavailable; table and CSV exported without a plot')
        return
    x = np.linspace(-60., 60., 481)
    b = np.array([model.wrist_bounds(v) for v in x])
    fig, ax = plt.subplots(figsize=(9, 7))
    ax.fill_between(x, b[:, 0], b[:, 1], color='#e3e9ef', label='SDK wrist allowed region')
    margin = plan['wrist_margin_deg']
    inside = (x >= -60 + margin) & (x <= 60 - margin)
    ax.fill_between(x[inside], b[inside, 0] + margin, b[inside, 1] - margin,
                    color='#cee7d5', label=f'{margin:g} deg coordinate margin')
    for role, color, marker in [('calibration', '#1565c0', 'o'), ('validation', '#bc4c00', '^')]:
        points = [p for p in plan['poses'] if p['role'] == role]
        ax.scatter([p['j6_deg'] for p in points], [p['j7_deg'] for p in points],
                   c=color, marker=marker, s=45, label=role.title(), zorder=3)
        for p in points:
            ax.annotate(p['id'], (p['j6_deg'], p['j7_deg']), xytext=(6, 3),
                        textcoords='offset points', fontsize=9, color=color)
    ax.set(xlabel='J6 absolute SDK angle (deg)', ylabel='J7 absolute SDK angle (deg)',
           xlim=(-70, 70), ylim=(-100, 100), title='Right wrist calibration pose review')
    ax.grid(alpha=.2)
    ax.legend(loc='upper center', ncol=2, fontsize=8)
    fig.text(.5, .012, 'Wrist limits only. Full-arm / tool / environment collision review is required.',
             ha='center', fontsize=9)
    fig.tight_layout(rect=(0, .03, 1, 1))
    fig.savefig(out / 'poses.png', dpi=160)
    plt.close(fig)


def summarize_rows(rows, pose, base5, model, settings, margin, kine, *, position=False):
    """Recheck a stored sampling window; reject malformed or edited observations."""
    if len(rows) < settings['min_samples']:
        raise ValueError('Too few samples')
    stamps = finite_array([r['monotonic_s'] for r in rows], (len(rows),), 'sample times')
    dt = np.diff(stamps)
    if (np.any(dt <= 0) or np.max(dt) > settings['fresh_timeout_s'] or
            stamps[-1] - stamps[0] < settings['duration_s']):
        raise ValueError('Sample time window is short, stale or unordered')
    if any(a['frame'] == b['frame'] for a, b in zip(rows, rows[1:])):
        raise ValueError('Repeated controller frames')
    q = finite_array([r['joint_deg'] for r in rows], (len(rows), 7), 'recorded joints')
    v = finite_array([r['velocity_deg_s'] for r in rows], (len(rows), 7), 'recorded velocity')
    w = finite_array([r['wrench_si'] for r in rows], (len(rows), 6), 'recorded wrench')
    raw = finite_array([r['wrench_raw'] for r in rows], (len(rows), 6), 'recorded raw data')
    if not np.allclose(w, raw * .0001, rtol=1e-9, atol=1e-12):
        raise ValueError('Raw sensor scaling mismatch')
    if any(r['channel'] != 216 for r in rows):
        raise ValueError('Wrong telemetry channel')
    target = [pose['j6_deg'], pose['j7_deg']]
    if position:
        goal = np.array([0.] * 5 + target)
        if any(r['mode'] != 1 or r['velocity_ratio'] != 10 or r['acceleration_ratio'] != 10 or
               r['low_speed'] != 1 or
               np.max(np.abs(finite_array(r['command_deg'], (7,), 'stored command') - goal)) > .001
               for r in rows):
            raise ValueError('Recorded Position mode, 10% ratios or target mismatch')
    if (np.max(np.abs(q[:, :5] - base5)) > settings['base_tolerance_deg'] or
            np.max(np.abs(q[:, 5:] - target)) > settings['target_tolerance_deg'] or
            np.max(np.abs(v)) > settings['max_velocity_deg_s'] or
            np.max(np.ptp(q, axis=0)) > settings['max_position_span_deg']):
        raise ValueError('Recorded pose is moving or does not match the fixed base/target')
    if not all(model.wrist_check(p[5:], margin)['valid'] for p in q):
        raise ValueError('Recorded wrist limit margin failed')
    std = np.std(w, axis=0)
    if max(std[:3]) > settings['max_force_std_N'] or max(std[3:]) > settings['max_torque_std_Nm']:
        raise ValueError('Recorded wrench is unstable')
    # Average rotations, not Euler angles, so mean gravity is exact for the sample window.
    mean_r = np.mean([kine.rotation(p) for p in q], axis=0)
    return {'id': pose['id'], 'role': pose['role'], 'samples': len(rows),
            'duration_s': float(stamps[-1] - stamps[0]), 'joint_mean_deg': q.mean(axis=0).tolist(),
            'joint_span_deg': np.ptp(q, axis=0).tolist(),
            'mean_R_base_flange': mean_r.tolist(), 'wrench_mean_si': w.mean(axis=0).tolist(),
            'wrench_std_si': std.tolist()}


def analyze_plan(plan, model, kine, base5, gravity):
    base5 = finite_array(base5, (5,), 'operator-selected J1-J5')
    gs = [kine.rotation([*base5, p['j6_deg'], p['j7_deg']]).T @ gravity
          for p in plan['poses'] if p['role'] == 'calibration']
    return coverage(gs)


def collect(args, config, plan, model):
    if not args.connect or not args.poses_reviewed:
        raise ValueError('collect requires --connect --poses-reviewed after your collision review')
    if not args.zero_note.strip():
        raise ValueError('Record the unchanged hardware-zero state with --zero-note')
    if not sys.stdin.isatty():
        raise ValueError('collect requires an interactive terminal for manual pose sampling')
    from ft_calibration_sdk import Feedback, Kinematics
    kine = Kinematics(config['sdk_root'], model)
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    session = {'schema_version': 1, 'kind': 'real_telemetry', 'created_utc': utc_now(),
               'model_sha256': model.digest, 'config': config, 'plan': plan,
               'zero_note': args.zero_note, 'poses_reviewed_by_operator': True,
               'base5_deg': None, 'captures': [], 'attempts': [], 'status': 'created'}
    write_json(out / 'session.json', session)
    try:
        with Feedback(config['sdk_root'], config['robot_ip'], connect=args.connect) as fb:
            base5 = fb.initial['joint_deg'][:5]
            session['base5_deg'] = base5
            session['status'] = 'collecting'
            write_json(out / 'session.json', session)
            print(f'已记录你选定的 J1～J5（度）：{base5}；后续保持不动。')
            g = config['gravity']['base_m_s2']
            g = model.gravity if g is None else finite_array(g, (3,), 'gravity')
            quality = analyze_plan(plan, model, kine, base5, g)
            print('按配置重力方向预测的覆盖情况：', quality)
            if not coverage_ok(quality, config['fitting']['max_condition']):
                raise ValueError('所选 J1～J5 下重力方向覆盖不足；请另选固定姿态后重新启动')
            print('仅采集反馈；手动调姿。工具无接触、手已离开后回车采样；q 结束。')
            for pose in plan['poses']:
                while True:
                    reply = input(f"{pose['id']} {pose['role']}: J6={pose['j6_deg']:+g}°, "
                                  f"J7={pose['j7_deg']:+g}°；到位后回车：").strip().lower()
                    if reply == 'q':
                        session['status'] = 'operator_stopped'
                        return
                    window = SampleWindow(pose, base5, model, config['sampling'], plan['wrist_margin_deg'])
                    name = f"{pose['id']}_attempt{len(session['attempts']) + 1:03}.jsonl"
                    attempt = {'id': pose['id'], 'raw_file': name, 'accepted': False}
                    session['attempts'].append(attempt)
                    write_json(out / 'session.json', session)
                    start, last_frame, complete = time.monotonic(), None, False
                    last_print = start
                    with (out / name).open('x') as stream:
                        while time.monotonic() - start < config['sampling']['timeout_s']:
                            row = fb.poll()
                            if row is not None:
                                if row['frame'] != last_frame:
                                    stream.write(json.dumps(row, allow_nan=False) + '\n')
                                    last_frame = row['frame']
                                if window.feed(row):
                                    complete = True
                                    break
                            if time.monotonic() - last_print >= 2.:
                                print(f'  {window.reason}; {len(window.rows)} samples', flush=True)
                                last_print = time.monotonic()
                            time.sleep(1. / config['sampling']['poll_hz'])
                    if not complete:
                        attempt['reason'] = window.reason
                        write_json(out / 'session.json', session)
                        print(f'本次未接受：{window.reason}；调整后可重试。')
                        continue
                    accepted_name = f"{pose['id']}.jsonl"
                    with (out / accepted_name).open('x') as stream:
                        for row in window.rows:
                            stream.write(json.dumps(row, allow_nan=False) + '\n')
                    summary = summarize_rows(window.rows, pose, base5, model, config['sampling'],
                                             plan['wrist_margin_deg'], kine)
                    summary['raw_file'] = accepted_name
                    summary['raw_sha256'] = sha256(out / accepted_name)
                    session['captures'].append(summary)
                    attempt['accepted'] = True
                    write_json(out / 'session.json', session)
                    print('已保存：均值 N/Nm', summary['wrench_mean_si'])
                    break
            session['status'] = 'complete'
    except BaseException as exc:
        session['status'] = 'interrupted' if isinstance(exc, (KeyboardInterrupt, EOFError)) else 'failed'
        session['error'] = str(exc)
        raise
    finally:
        session['updated_utc'] = utc_now()
        write_json(out / 'session.json', session)
        print(f'采样已保存至 {out}；已执行连接清理。拟合使用独立 fit 命令。')


def automatic(args, config, plan, model):
    if not args.connect or not args.poses_reviewed:
        raise ValueError('auto requires --connect --poses-reviewed')
    if not args.zero_note.strip():
        raise ValueError('Record the unchanged no-tool zero and final tool assembly with --zero-note')
    from ft_calibration_auto import AutoSequence, validate_auto
    from ft_calibration_sdk import Kinematics, PositionFeedback
    validate_auto(config, plan, model)
    if not args.collect_only:
        coordinates(config, model)  # Fail before connecting or moving if fitting is unconfigured.
    kine = Kinematics(config['sdk_root'], model)
    g = config['gravity']['base_m_s2']
    g = model.gravity if g is None else finite_array(g, (3,), 'gravity')
    quality = analyze_plan(plan, model, kine, [0.] * 5, g)
    if not coverage_ok(quality, config['fitting']['max_condition']):
        raise ValueError('Zero-base plan has inadequate gravity coverage')
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    session = {'schema_version': 1, 'kind': 'real_telemetry', 'created_utc': utc_now(),
               'model_sha256': model.digest, 'config': config, 'plan': plan,
               'zero_note': args.zero_note, 'poses_reviewed_by_operator': True,
               'base5_deg': [0.] * 5, 'captures': [], 'attempts': [], 'status': 'created',
               'control': {'arm': 'B', 'mode': 'Position', 'state': 1,
                           'velocity_percent': 10, 'acceleration_percent': 10},
               'predicted_coverage': quality}

    def save():
        session['updated_utc'] = utc_now()
        write_json(out / 'session.json', session)

    def trace_to(stream):
        def trace(row):
            stream.write(json.dumps(row, allow_nan=False) + '\n')
            stream.flush()
        return trace

    def terminate(signum, _):
        raise KeyboardInterrupt(f'Signal {signum}')

    save()
    old_term = signal.signal(signal.SIGTERM, terminate)
    try:
        with PositionFeedback(config['sdk_root'], config['robot_ip'], connect=True) as fb:
            runner = AutoSequence(fb, config, plan, model)
            try:
                session['status'] = 'preflight'
                save()
                with (out / 'preflight.jsonl').open('x') as stream:
                    runner.preflight(trace_to(stream))
                    print('零位、静止及力传数据预检查通过；进入 Position，速度/加速度 10%/10%。', flush=True)
                    runner.enable(trace_to(stream))
                session['status'] = 'collecting'
                save()
                for index, pose in enumerate(plan['poses'], 1):
                    target = [0.] * 5 + [pose['j6_deg'], pose['j7_deg']]
                    print(f"[{index}/{len(plan['poses'])}] {pose['id']} → {target} deg", flush=True)
                    name = f"{pose['id']}_attempt{index:03}.jsonl"
                    attempt = {'id': pose['id'], 'raw_file': name, 'accepted': False,
                               'target_deg': target}
                    session['attempts'].append(attempt)
                    save()
                    with (out / name).open('x') as stream:
                        rows = runner.capture(pose, trace_to(stream))
                    accepted_name = f"{pose['id']}.jsonl"
                    with (out / accepted_name).open('x') as stream:
                        for row in rows:
                            stream.write(json.dumps(row, allow_nan=False) + '\n')
                    summary = summarize_rows(rows, pose, [0.] * 5, model, config['sampling'],
                                             plan['wrist_margin_deg'], kine, position=True)
                    summary.update(raw_file=accepted_name, raw_sha256=sha256(out / accepted_name))
                    session['captures'].append(summary)
                    attempt['accepted'] = True
                    save()
                    print('  已保存：均值 N/Nm', summary['wrench_mean_si'], flush=True)
                session['status'] = 'complete'
                save()
                print(f"{len(plan['poses'])} 点采集完成；保持最后目标，不自动回零。", flush=True)
            except BaseException:
                if runner.control_attempted:
                    session['stop'] = {'requested': True, 'verified': False}
                    # Stop before any disk operation: a full / failing disk must
                    # not prevent the robot stop after a logging error.
                    try:
                        runner.stop()
                        session['stop']['verified'] = True
                    except BaseException as stop_error:
                        session['stop']['error'] = str(stop_error)
                        print(f'停止未能确认：{stop_error}；请现场确认机械臂状态。', file=sys.stderr, flush=True)
                    save()
                raise
    except BaseException as exc:
        session['status'] = 'interrupted' if isinstance(exc, (KeyboardInterrupt, EOFError)) else 'failed'
        session['error'] = str(exc)
        raise
    finally:
        signal.signal(signal.SIGTERM, old_term)
        save()
        print(f'原始采样保存至 {out}；已执行 SDK 连接清理。', flush=True)
    if args.collect_only:
        print('原始采集完成；补齐坐标/符号配置后，可使用 fit 离线拟合。')
        return 0
    # Fit after releasing the SDK; this part is entirely offline.
    return fit_session(argparse.Namespace(session=str(out), output=str(out / 'result'),
                                         current_file=args.current_file), config, model)


def fit_session(args, config, model):
    r_fs, g_base, signs = coordinates(config, model)
    folder = Path(args.session).resolve()
    session = json.loads((folder / 'session.json').read_text())
    if session.get('schema_version') != 1 or session.get('kind') not in ('real_telemetry', 'synthetic_test'):
        raise ValueError('Unsupported session schema/type')
    if session.get('model_sha256') != model.digest:
        raise ValueError('Session model differs from current verified model')
    plan = session['plan']
    validate_plan(plan, model)
    validate_settings(session['config'])
    base5 = finite_array(session['base5_deg'], (5,), 'session fixed J1-J5')
    from ft_calibration_sdk import Kinematics
    kine = Kinematics(config['sdk_root'], model)
    lookup = {p['id']: p for p in plan['poses']}
    groups = {'calibration': [], 'validation': []}
    used = set()
    for capture in session['captures']:
        ident = capture['id']
        if ident not in lookup or ident in used:
            raise ValueError('Unknown/duplicate captured pose')
        used.add(ident)
        file = (folder / capture['raw_file']).resolve()
        if file.parent != folder or sha256(file) != capture['raw_sha256']:
            raise ValueError('Sample path or hash mismatch')
        rows = [json.loads(line) for line in file.read_text().splitlines()]
        summary = summarize_rows(rows, lookup[ident], base5, model, session['config']['sampling'],
                                 plan['wrist_margin_deg'], kine,
                                 position=session.get('control', {}).get('mode') == 'Position')
        g = r_fs.T @ np.array(summary['mean_R_base_flange']).T @ g_base
        w = np.array(summary['wrench_mean_si']) * signs
        groups[lookup[ident]['role']].append((ident, g, w, summary))
    if len(groups['calibration']) < 10 or len(groups['validation']) < 4:
        raise ValueError('Need at least 10 accepted calibration and 4 independent validation poses')
    train, valid = groups['calibration'], groups['validation']
    parameters = solve([r[1] for r in train], [r[2] for r in train], config['fitting']['max_condition'])
    reports = {}
    for role, rows in groups.items():
        reports[role] = {'pose_ids': [r[0] for r in rows],
                         **residual_report(parameters, [r[1] for r in rows], [r[2] for r in rows])}
    v = reports['validation']
    passed = (v['force_vector_rms_N'] <= config['fitting']['max_validation_force_rms_N'] and
              v['torque_vector_rms_Nm'] <= config['fitting']['max_validation_torque_rms_Nm'])
    synthetic = session['kind'] == 'synthetic_test'
    result = {'schema_version': 1, 'kind': session['kind'], 'created_utc': utc_now(),
              'validation_passed': bool(passed), 'parameters': parameters,
              'sensor': config['sensor'], 'gravity_base_m_s2': g_base.tolist(),
              'model_sha256': model.digest, 'session_sha256': sha256(folder / 'session.json'),
              'zero_note': session['zero_note'], 'thresholds': config['fitting'],
              'residuals': reports,
              'equations': 'w_canonical = signs * (raw / 10000); Fg=m*g_s; tau_g=com_s x Fg; '
                           'w_external=w_canonical-[Fg,tau_g]-bias',
              'com_reference': 'sensor measurement origin, expressed in sensor frame'}
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    write_json(out / 'report.json', result)
    (out / 'REPORT.md').write_text(
        '# 末端六维力工具标定报告\n\n'
        f"数据类型：{session['kind']}；验证：{'通过' if passed else '未通过'}。\n\n"
        f"质量：{parameters['mass_kg']:.8f} kg\n\n"
        f"质心（传感器测量原点，m）：{parameters['com_sensor_m']}\n\n"
        f"验证力矢量 RMS：{v['force_vector_rms_N']:.6f} N\n\n"
        f"验证力矩矢量 RMS：{v['torque_vector_rms_Nm']:.6f} N·m\n\n"
        f"标定点：{len(train)}，独立验证点：{len(valid)}；逐姿态残差见 report.json。\n\n"
        '参数只适用于本工具装配、测量原点、轴向、单位及硬件零点。重新清零后需更新零偏。'
        '这是静态重力补偿，不包含运动惯性补偿。\n')
    if passed:
        name = 'synthetic_calibration.yaml' if synthetic else 'calibration.yaml'
        write_yaml(out / name, result)
        print(f'验证通过；结果：{out / name}')
        if not synthetic and getattr(args, 'current_file', None):
            current = publish_current_calibration(out / name, args.current_file)
            print(f'固定标定文件已更新：{current}（桥接下次启动时加载）')
    else:
        print(f'验证未通过，未生成可用标定配置；诊断：{out / "report.json"}')
    print(f"质量 {parameters['mass_kg']:.6f} kg；质心 {parameters['com_sensor_m']} m")
    return 0 if passed else 2


def main(argv=None):
    defaults = default_config_dir()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='defaults to right_ft_calibration_robot.yaml for auto; generic config for other commands')
    parser.add_argument('--poses', default=str(defaults / 'right_ft_calibration_poses.yaml'))
    commands = parser.add_subparsers(dest='command')
    p = commands.add_parser('plan', help='offline wrist-limit check and pose review export')
    p.add_argument('--output', help='export REVIEW.md, CSV, YAML and optional plot')
    a = commands.add_parser('analyze', help='offline coverage using YOUR chosen J1-J5')
    a.add_argument('--base-deg', type=float, nargs=5, required=True, metavar=('J1', 'J2', 'J3', 'J4', 'J5'))
    c = commands.add_parser('collect', help='explicit live telemetry, manual posing only')
    c.add_argument('--connect', action='store_true')
    c.add_argument('--poses-reviewed', action='store_true')
    c.add_argument('--zero-note', required=True, help='record no-tool zero state; do not zero with tool')
    c.add_argument('--output', required=True, help='new session directory')
    auto = commands.add_parser('auto', help='right Position, zero J1-J5, automatic poses at 10%% / 10%%')
    auto.add_argument('--connect', action='store_true', default=True, help='compatibility option; auto connects by default')
    auto.add_argument('--poses-reviewed', action='store_true', default=True,
                      help='compatibility option; auto uses the operator-reviewed pose plan')
    auto.add_argument('--zero-note', default='无工具清零后安装工具；由操作者启动自动标定',
                      help='optional note; no zero-record file is required')
    auto.add_argument('--output', help='optional new session directory; default: automatically created under ~/ft_calibration_runs')
    auto.add_argument('--collect-only', action='store_true', help='save raw samples; defer configured fitting')
    auto.add_argument('--current-file',
                      help='fixed output file; defaults to current_calibration_file in config or the package config directory')
    f = commands.add_parser('fit', help='offline fit and independent validation from saved raw samples')
    f.add_argument('session')
    f.add_argument('--output', required=True, help='new result directory')
    f.add_argument('--current-file', help='optionally replace the fixed file after a successful real-data fit; omitted for offline reviews')
    args = parser.parse_args(argv)
    if args.command is None:
        parser.print_help()
        return 0
    if args.config is None:
        args.config = str(defaults / ('right_ft_calibration_robot.yaml' if args.command == 'auto'
                                     else 'right_ft_calibration.yaml'))
    if args.command == 'auto' and args.output is None:
        args.output = str(Path.home() / 'ft_calibration_runs' /
                          f'calibration_{datetime.now():%Y%m%d_%H%M%S}_{time.time_ns()}')
    try:
        config = read_yaml(args.config)
        model = Model.load(config['model_file'])
        validate_settings(config)
        if args.command == 'fit':
            return fit_session(args, config, model)
        plan = read_yaml(args.poses)
        validate_plan(plan, model)
        if args.command == 'plan':
            print(plan_table(plan, model))
            if args.output:
                export_plan(plan, model, Path(args.output))
        elif args.command == 'analyze':
            from ft_calibration_sdk import Kinematics
            kine = Kinematics(config['sdk_root'], model)
            g = config['gravity']['base_m_s2']
            g = model.gravity if g is None else finite_array(g, (3,), 'gravity')
            quality = analyze_plan(plan, model, kine, args.base_deg, g)
            adequate = coverage_ok(quality, config['fitting']['max_condition'])
            print(json.dumps({'coverage_adequate': adequate, **quality}, indent=2))
            print('Based on configured base gravity. Fixed sensor mounting rotation does not change rank/conditioning.')
            return 0 if adequate else 2
        elif args.command == 'auto':
            if args.current_file is None:
                args.current_file = config.get('current_calibration_file') or str(
                    defaults / 'right_ft_calibration_current.yaml')
            return automatic(args, config, plan, model)
        else:
            collect(args, config, plan, model)
        return 0
    except (ValueError, RuntimeError, OSError, KeyError, TypeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    except (KeyboardInterrupt, EOFError):
        print('Stopped; accepted samples remain saved.', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
