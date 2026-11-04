"""MAV_CMD_BATTERY_RESET: the battery's charge set, as ArduSub sets it (a test's
stand-in for a battery swap). The front seat runs on while the test reads, so the
charge has drawn down a little by then: "full" is within a percent."""

from .support import M, ready

RESET = M.MAV_CMD_BATTERY_RESET


def _drain(fs, used_frac):
    b = fs.server.vehicle.battery
    b.used_mah = b.capacity_mah * used_frac
    return b


def test_the_reset_sets_the_charge_and_is_accepted(front_seat):
    fs = front_seat()
    c = fs.client()
    ready(c)
    b = _drain(fs, 0.4)
    assert c.command(RESET, 1, 100) == M.MAV_RESULT_ACCEPTED
    assert b.remaining > 0.99
    assert c.command(RESET, 1, 30) == M.MAV_RESULT_ACCEPTED
    assert 0.29 < b.remaining <= 0.30 + 1e-9
    c.interval("BATTERY_STATUS", 5)
    status = c.wait("BATTERY_STATUS", cond=lambda s: s.battery_remaining <= 30)
    assert status.battery_remaining >= 29


def test_a_mask_without_battery_one_fails(front_seat):
    fs = front_seat()
    c = fs.client()
    b = _drain(fs, 0.4)
    assert c.command(RESET, 2, 100) == M.MAV_RESULT_FAILED
    assert b.remaining <= 0.6


def test_a_percentage_over_100_is_denied(front_seat):
    fs = front_seat()
    c = fs.client()
    b = _drain(fs, 0.4)
    assert c.command(RESET, 1, 101) == M.MAV_RESULT_DENIED
    assert c.command(RESET, 1, -1) == M.MAV_RESULT_DENIED
    assert b.remaining <= 0.6


def test_it_resets_while_armed(front_seat):
    fs = front_seat()
    c = fs.client()
    assert c.command(M.MAV_CMD_COMPONENT_ARM_DISARM, 1) == M.MAV_RESULT_ACCEPTED
    b = _drain(fs, 0.5)
    assert c.command(RESET, 1, 100) == M.MAV_RESULT_ACCEPTED
    assert b.remaining > 0.99
    assert fs.server.vehicle.armed
