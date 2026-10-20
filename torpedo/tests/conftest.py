"""Shared fixtures: the two generic test hulls and the measures flown on them."""

from pathlib import Path

import pytest

from torpedo_sitl import check, model

MODELS = Path(__file__).resolve().parent / "models"
HULLS = ("torpedo-test-small", "torpedo-test-large")


@pytest.fixture(params=HULLS)
def hull_name(request):
    """The id of each test hull in turn."""
    return request.param


@pytest.fixture
def hull_model(hull_name):
    """The model of each test hull in turn."""
    return model.load(MODELS / f"{hull_name}.json")


_measured = {}


@pytest.fixture
def measured(hull_name, hull_model):
    """``check.measure`` of the hull, flown once per hull for the whole run."""
    if hull_name not in _measured:
        _measured[hull_name] = check.measure(hull_model)
    return _measured[hull_name]


@pytest.fixture
def front_seat(tmp_path):
    """Start front seats in threads; all are stopped at the end of the test."""
    from .support import FrontSeat

    started = []

    def start(**kw):
        fs = FrontSeat(tmp_path, **kw)
        started.append(fs)
        return fs

    yield start
    for fs in started:
        fs.close()
