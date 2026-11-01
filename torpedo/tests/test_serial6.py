"""SERIAL6: the acoustic ocean's truth feed (SIM_STATE), served only when
``SERIAL6_PROTOCOL`` is 2. The client below asks as the ocean does: system 247, no
heartbeat, the ``SIM_OPOS_ALT`` probe, then SIM_STATE and GLOBAL_POSITION_INT at 2 Hz
(``MAV_CMD_SET_MESSAGE_INTERVAL``)."""

import time

import pytest

from torpedo_sitl.server import Config

from .support import HOME, M, ready

ON = "SERIAL6_PROTOCOL 2"
SIM_STATE, GLOBAL_POSITION_INT = 108, 33


def ocean_asks(c):
    """What the ocean sends, in its order: the SITL probe, then the two streams."""
    c.m.mav.param_request_read_send(c.target, 1, b"SIM_OPOS_ALT", -1)
    probe = c.wait("PARAM_VALUE", cond=lambda p: p.param_id == "SIM_OPOS_ALT")
    for msg_id in (SIM_STATE, GLOBAL_POSITION_INT):
        c.m.mav.command_long_send(
            c.target,
            1,
            M.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            1e6 / 2.0,
            0,
            0,
            0,
            0,
            0,
        )
    return probe


def row(c):
    """One row as the ocean builds it: SIM_STATE after a GLOBAL_POSITION_INT."""
    boot = c.wait("GLOBAL_POSITION_INT").time_boot_ms
    sim = c.wait("SIM_STATE")
    return sim.lat_int / 1e7, sim.lon_int / 1e7, -sim.alt, boot


def test_serial6_streams_the_truth_to_a_client_that_asks_as_the_ocean_does(front_seat):
    fs = front_seat(sitl_params=ON)
    assert "SERIAL6" in fs.ports
    c = fs.client("SERIAL6", system=247)
    assert c.hb.type == M.MAV_TYPE_SUBMARINE
    assert ocean_asks(c).param_value == 0.0  # only a simulator answers
    lat, lon, depth, boot = row(c)
    assert (lat, lon) == pytest.approx(HOME, abs=1e-5)
    assert depth == pytest.approx(fs.server.vehicle.hull.depth, abs=0.5)
    assert boot >= 0
    # two rows a second of vehicle time: the next arrive without asking again
    assert row(c)[3] > boot


def test_serial6_carries_the_truth_not_the_estimate(front_seat):
    fs = front_seat(sitl_params=ON, speedup=30.0)
    c2 = fs.client("SERIAL2")
    ready(c2)
    c2.m.mav.param_set_send(
        c2.target, 1, b"SIM_GPS1_GLTCH_X", 0.0002, M.MAV_PARAM_TYPE_REAL32
    )
    c6 = fs.client("SERIAL6", system=247)
    ocean_asks(c6)
    row(c6)
    time.sleep(0.5)  # the glitch reaches the estimate
    estimate = c2.fresh("GLOBAL_POSITION_INT")
    lat, _, _, _ = row(c6)
    assert estimate.lat / 1e7 - lat == pytest.approx(0.0002, abs=2e-6)
    assert lat == pytest.approx(HOME[0], abs=1e-5)


def test_serial6_is_off_unless_the_protocol_is_2(front_seat):
    assert "SERIAL6" not in front_seat().ports
    for text in ("SERIAL6_PROTOCOL -1", "SERIAL6_PROTOCOL 1", "SERIAL6_PROTOCOL 5"):
        assert "SERIAL6" not in front_seat(sitl_params=text).ports


def test_a_reboot_reads_serial6_protocol_again(front_seat):
    fs = front_seat()
    c = fs.client("SERIAL0")
    assert "SERIAL6" not in fs.ports
    c.m.mav.param_set_send(
        c.target, 1, b"SERIAL6_PROTOCOL", 2.0, M.MAV_PARAM_TYPE_REAL32
    )
    assert c.param("SERIAL6_PROTOCOL") == 2.0
    assert "SERIAL6" not in fs.ports  # ArduSub reads it at boot
    c.m.mav.command_long_send(
        c.target, 1, M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0
    )
    end = time.time() + 10
    while "SERIAL6" not in fs.ports and time.time() < end:
        time.sleep(0.05)
    assert "SERIAL6" in fs.ports
    again = fs.client("SERIAL6", system=247)
    ocean_asks(again)
    assert row(again)[0] == pytest.approx(HOME[0], abs=1e-5)


def test_a_client_that_disconnects_and_comes_back_gets_rows_again(front_seat):
    fs = front_seat(sitl_params=ON)
    first = fs.client("SERIAL6", system=247)
    ocean_asks(first)
    row(first)
    first.close()
    second = fs.client("SERIAL6", system=247)
    ocean_asks(second)
    assert row(second)[0] == pytest.approx(HOME[0], abs=1e-5)


def test_serial6_port_follows_the_instance():
    assert Config().serial6_port == 5766
    assert Config(instance=2).serial6_port == 5786
    assert Config(base_port=5790).serial6_port == 5796
