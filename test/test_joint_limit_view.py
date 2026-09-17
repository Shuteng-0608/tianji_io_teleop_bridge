import math
from types import SimpleNamespace

import pytest

from joint_limit_view import JointView, JOINT_NAMES, decode, demo


def make_status(name, actual):
    fields = dict(joint_name=name, enabled='true', feedback_rad=str(actual), command_rad='0',
                  hard_lower_rad='-2', hard_upper_rad='2',
                  soft_lower_rad='-1.99', soft_upper_rad='1.99',
                  release_lower_rad='-1.98', release_upper_rad='1.98',
                  safe_lower_rad='-1.98', safe_upper_rad='1.98', tolerance_rad='0.1')
    return SimpleNamespace(name='alignment/' + name, message='ALIGNED',
                           values=[SimpleNamespace(key=k, value=v) for k, v in fields.items()])


def test_mapping_is_canonical_not_message_order():
    statuses = [make_status(name, i / 10) for i, name in enumerate(JOINT_NAMES)]
    _, rows = decode(list(reversed(statuses)))
    assert [r.name for r in rows] == JOINT_NAMES
    for i, row in enumerate(rows):
        assert row.actual == pytest.approx(math.degrees(i / 10))
        assert row.ranges_valid()


def test_duplicates_missing_or_unknown_never_reuse_other_arm():
    _, rows = decode([make_status('Joint1_R', 1), make_status('Joint1_R', 2),
                      make_status('lh_FFJ1', 3), make_status('Joint2_L', 0.4)])
    assert math.isnan(rows[0].actual)
    assert math.isnan(rows[7].actual)
    assert rows[1].actual == pytest.approx(math.degrees(0.4))
    assert not rows[7].ranges_valid()


def test_right_joint_two_requires_increasing_into_release_range():
    _, rows = demo()
    advice, severity = rows[8].advice(True, True, 'SOFT_LIMIT_HOLD')
    assert severity == 'adjust'
    assert '增大' in advice
    assert '-95.169' in advice
    assert '+0.578' in advice


def test_upper_limit_and_alignment_are_intersected():
    _, rows = demo()
    row = rows[0]
    row.actual, row.target = 96, 99
    assert '减小' in row.advice(True, True, 'SOFT_LIMIT_HOLD')[0]
    assert '+98.000' in row.advice(True, True, 'SOFT_LIMIT_HOLD')[0]
    row.actual, row.target = 0, 10
    assert '+5.730' in row.advice(True, True, 'COMMAND_WAIT')[0]


def test_satisfied_joint_is_not_claimed_whole_robot_ready():
    row = demo()[1][0]
    assert '等待其余关节' in row.advice(True, True, 'COMMAND_WAIT')[0]


@pytest.mark.parametrize('fb,cmd', [(False, False), (False, True), (True, False)])
def test_stale_data_never_gives_movement_advice(fb, cmd):
    text, severity = demo()[1][8].advice(fb, cmd, 'SOFT_LIMIT_HOLD')
    assert severity == 'stale'
    assert '增大' not in text and '减小' not in text


def test_disabled_joint_stays_display_only():
    row = demo()[1][8]
    row.enabled = False
    assert row.advice(True, True, 'SOFT_LIMIT_HOLD')[1] == 'stale'


def test_legacy_diagnostics_without_new_limits_are_invalid():
    row = JointView.from_values('Joint1_L', dict(enabled='true', feedback_rad='1'))
    assert not row.ranges_valid()
    assert '重启' in row.advice(True, True, 'ACTIVE')[0]


def test_bad_range_or_nonfinite_values_are_not_actionable():
    row = demo()[1][0]
    row.release = (200, -200)
    assert not row.ranges_valid()
    row = demo()[1][0]
    row.target = math.nan
    assert row.advice(True, True, 'ACTIVE')[1] == 'stale'


def test_no_intersection_requires_operator_check_not_impossible_target():
    row = demo()[1][0]
    row.actual = 150
    assert row.advice(True, True, 'COMMAND_WAIT')[1] == 'danger'


class RecordingCanvas:
    def __init__(self):
        self.items = []

    def __getattr__(self, name):
        def record(*args, **kwargs):
            self.items.append((name, args, kwargs))
        return record


def test_bar_only_view_has_no_advice_or_verbose_labels():
    from joint_limit_dashboard import LimitWindow
    window = LimitWindow.__new__(LimitWindow)
    window.canvas = RecordingCanvas()
    window.draw_row(demo()[1][8], 16, 12, 550, True, True, True)
    texts = [options['text'] for name, _, options in window.canvas.items if name == 'create_text']
    assert len(texts) == 4  # Name, actual angle, two endpoints.
    assert texts[0] == 'Joint2_R'
    assert all('增大' not in text and '减小' not in text for text in texts)
    assert sum(name == 'create_oval' for name, _, _ in window.canvas.items) == 1
    assert sum(name == 'create_polygon' for name, _, _ in window.canvas.items) == 1


def test_bar_only_stale_data_hides_both_markers():
    from joint_limit_dashboard import LimitWindow
    window = LimitWindow.__new__(LimitWindow)
    window.canvas = RecordingCanvas()
    window.draw_row(demo()[1][8], 16, 12, 550, False, False, False)
    assert not any(name in ('create_oval', 'create_polygon') for name, _, _ in window.canvas.items)
    assert any(options.get('text') == '—' for _, _, options in window.canvas.items)
