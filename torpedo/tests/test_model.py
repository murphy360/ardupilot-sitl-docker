"""The model file: schema 1 loads, anything else raises ModelError naming the key."""

import json

import pytest

from torpedo_sitl import model, trials

from .conftest import MODELS


def _doc(name="torpedo-test-small"):
    return json.loads((MODELS / f"{name}.json").read_text())


def test_loads_a_schema_1_file(hull_model, hull_name):
    assert hull_model.id == hull_name
    assert hull_model.hydro.xUabsU < 0
    assert hull_model.fin.max_rad == pytest.approx(0.24)
    assert hull_model.turn_radius_m > hull_model.length_m


def test_is_frozen(hull_model):
    with pytest.raises(AttributeError):
        hull_model.mass_kg = 1.0


@pytest.mark.parametrize("key", ["mass_kg", "max_thrust_n", "turn_radius_m"])
def test_a_missing_key_is_named(key, tmp_path):
    doc = _doc()
    del doc[key]
    with pytest.raises(model.ModelError, match=key):
        model.from_dict(doc)


def test_a_missing_nested_key_is_named():
    doc = _doc()
    del doc["hydro"]["nRabsR"]
    with pytest.raises(model.ModelError, match=r"hydro\.nRabsR"):
        model.from_dict(doc)
    doc = _doc()
    del doc["fin"]
    with pytest.raises(model.ModelError, match="fin"):
        model.from_dict(doc)


@pytest.mark.parametrize("schema", [0, 2, "1", None])
def test_another_schema_is_refused(schema):
    doc = _doc()
    doc["schema"] = schema
    with pytest.raises(model.ModelError, match="schema"):
        model.from_dict(doc)


def test_a_value_that_is_not_a_number_is_named():
    doc = _doc()
    doc["length_m"] = "long"
    with pytest.raises(model.ModelError, match="length_m"):
        model.from_dict(doc)
    doc = _doc()
    doc["fin"]["rate_rad_s"] = True
    with pytest.raises(model.ModelError, match=r"fin\.rate_rad_s"):
        model.from_dict(doc)
    doc = _doc()
    doc["mass_kg"] = float("inf")
    with pytest.raises(model.ModelError, match="mass_kg"):
        model.from_dict(doc)


def test_load_reads_a_file_and_refuses_junk(tmp_path):
    path = tmp_path / "bad.json"
    path.write_text("{not json")
    with pytest.raises(model.ModelError, match="not JSON"):
        model.load(path)
    path.write_text("[]")
    with pytest.raises(model.ModelError, match="not an object"):
        model.load(path)
    doc = _doc()
    del doc["id"]
    with pytest.raises(model.ModelError, match="id"):
        model.from_dict(doc)


def test_the_fixtures_are_what_the_script_writes(hull_name):
    """tests/models/make_fixtures.py writes the committed files, turn radius too."""
    from .models import make_fixtures as mf

    length, d, mass = mf.HULLS[hull_name]
    doc = mf.build(hull_name, length, d, mass)
    written = _doc(hull_name)
    radius, _ = trials.steady_turn(model.from_dict(doc))
    doc["turn_radius_m"] = round(radius, 1)
    assert doc == written
