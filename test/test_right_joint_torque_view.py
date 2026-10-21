import math

from right_joint_torque_view import RIGHT_JOINTS, TorqueView, format_torque


def test_right_arm_mapping_ignores_left_and_orders_by_name():
    names = [f'Joint{i}_{side}' for side in ('L', 'R') for i in range(1, 8)]
    model = TorqueView()
    model.accept(names[::-1], list(range(14))[::-1], 10**9, 10**9, 0.0)
    assert model.current(10**9, 0.0)[0] == tuple(range(7, 14))


def test_three_decimals_and_negative_zero():
    assert format_torque(1.23456) == '1.235'
    assert format_torque(-2.1) == '-2.100'
    assert format_torque(-0.0001) == '0.000'
    assert format_torque(0) == '0.000'
    assert format_torque(math.nan) == '—'


def test_invalid_input_hides_old_values_and_fresh_data_recovers():
    model = TorqueView()
    model.accept(RIGHT_JOINTS, range(7), 10**9, 10**9, 0.0)
    cases = [(RIGHT_JOINTS, []), (RIGHT_JOINTS[:-1], range(6)),
             (RIGHT_JOINTS[:-1] + (RIGHT_JOINTS[0],), range(7)),
             (RIGHT_JOINTS, [1, 2, 3, 4, 5, 6, math.inf])]
    for i, (names, values) in enumerate(cases):
        stamp = 10**9 + i * 10**7
        model.accept(names, values, stamp, stamp, 0.0)
        assert model.current(stamp, 0.0)[0] is None
    model.accept(RIGHT_JOINTS, range(7), 2 * 10**9, 2 * 10**9, 1.0)
    assert model.current(2 * 10**9, 1.0)[0] == tuple(range(7))


def test_stale_duplicate_future_and_clock_reset():
    model = TorqueView()
    model.accept(RIGHT_JOINTS, range(7), 10**9, 10**9, 0.0)
    model.accept(RIGHT_JOINTS, range(7), 10**9, 10**9, 0.4)
    assert model.current(10**9, 0.51)[0] is None  # Frozen ROS clock, repeated message.
    assert model.current(2 * 10**9, 0.1)[0] is None  # Old source, recent local arrival.
    model.accept(RIGHT_JOINTS, range(7), 5 * 10**8, 5 * 10**8, 0.6)
    assert model.current(5 * 10**8, 0.6)[0] == tuple(range(7))
    model.accept(RIGHT_JOINTS, range(7), 5 * 10**9, 2 * 10**9, 1.0)
    assert model.current(2 * 10**9, 1.0)[0] is None
    model.accept(RIGHT_JOINTS, range(7), 5 * 10**8, 5 * 10**8, 1.1)
    assert model.current(5 * 10**8, 1.1)[0] == tuple(range(7))
