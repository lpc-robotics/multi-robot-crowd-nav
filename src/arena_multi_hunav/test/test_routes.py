import copy
import importlib.util
from pathlib import Path

import pytest
import yaml

from arena_multi_hunav.routes import compile_route
from arena_multi_hunav.route_tracking import RouteTracker
from arena_multi_hunav.multi_adapter import load_people

ROOT = Path(__file__).resolve().parents[3]


def spec(mode="loop", count=4):
    data = {"route_mode": mode, "radius": .4, "goal_radius": .3,
            "init_pose": {"x": 5., "y": 5.}, "goals": [f"g{i}" for i in range(count)]}
    for i in range(count):
        data[f"g{i}"] = {"x": 6. + i * 2, "y": 6.}
    return data


@pytest.mark.parametrize("count,expected", [(2, (0, 1)), (3, (0, 1, 2, 1)), (4, (0, 1, 2, 3, 2, 1))])
def test_folded_routes_do_not_repeat_endpoints(count, expected):
    route = compile_route(spec("ping_pong", count), person="p")
    assert route.names == tuple(f"g{i}" for i in expected)
    assert route.cyclic


@pytest.mark.parametrize("count", [2, 3, 4])
def test_loop_keeps_configured_first_goal_and_order(count):
    route = compile_route(spec(count=count), person="p")
    assert route.names == tuple(f"g{i}" for i in range(count))
    assert route.points[0] == (6., 6.)


@pytest.mark.parametrize("goals,cyclic", [([], False), (["g0"], True), (["g0", "g1"], False), (["g0", "g1"], True)])
def test_legacy_queue_semantics(goals, cyclic):
    data = spec(); del data["route_mode"]
    data.update(goals=goals, cyclic_goals=cyclic)
    route = compile_route(data, person="legacy")
    assert route.names == tuple(goals) and route.cyclic == cyclic


@pytest.mark.parametrize("change,reason", [
    ({"route_mode": "random"}, "route_mode"), ({"route_mode": None}, "route_mode"),
    ({"goals": []}, "at least two"), ({"goals": ["g0"]}, "at least two"),
    ({"goals": ["g0", "g0"]}, "duplicate"), ({"goals": ["g0", "missing"]}, "missing"),
    ({"g0": {"x": float("nan"), "y": 6}}, "g0"),
    ({"g1": {"x": 6., "y": 6.}}, "duplicate goal coordinates"),
    ({"g1": {"x": 6.5, "y": 6.}}, "overlap"),
    ({"g1": {"x": 6.60000001, "y": 6.}}, "overlap"),
    ({"g3": {"x": 6., "y": 6.5}}, "overlap"),
    ({"cyclic_goals": False}, "conflicts"), ({"cyclic_goals": "true"}, "conflicts"),
    ({"goal_radius": 0}, "goal_radius"),
    ({"g0": {"x": 30., "y": 6.}}, "g0.*bounds"),
    ({"init_pose": {"x": .2, "y": 6.}}, "init_pose.*bounds"),
    ({"init_pose": {"x": .4, "y": 6.}}, "init_pose.*bounds"),
    ({"radius": 1e100}, "float32"), ({"goal_radius": 1e-100}, "float32"),
])
def test_bad_routes_have_context(change, reason):
    data = spec(); data.update(change)
    with pytest.raises(ValueError, match="person_1.*" + reason):
        compile_route(data, person="person_1", world_bounds=(30, 23))


def test_world_bounds_apply_to_legacy_without_changing_queue():
    data = spec(); del data["route_mode"]; data["cyclic_goals"] = True
    data["g1"] = {"x": 31., "y": 5.}
    assert len(compile_route(data, person="p").points) == 4
    with pytest.raises(ValueError, match="p.g1.*bounds"):
        compile_route(data, person="p", world_bounds=(30, 23))


def test_new_scene_loads_three_loops_and_three_folds():
    people, models = load_people(ROOT / "config/hunav/multi_regular_6_multigoal.yaml", world_bounds=(30, 23))
    old, old_models = load_people(ROOT / "config/hunav/multi_regular_6.yaml", world_bounds=(30, 23))
    assert models == old_models
    assert [len(a.goals) for a in people.agents] == [4, 4, 4, 6, 6, 6]
    for new, previous in zip(people.agents, old.agents):
        assert new.goals[0] == previous.goals[0]
        new.goals = copy.deepcopy(previous.goals)
        assert new == previous  # All non-route appearance/motion settings preserved.


def test_loader_reports_person_and_goal(tmp_path):
    document = yaml.safe_load((ROOT / "config/hunav/multi_regular_6_multigoal.yaml").read_text())
    del document["hunav_loader"]["ros__parameters"]["person_4"]["g3"]
    path = tmp_path / "bad.yaml"; path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError, match="person_4.g3"):
        load_people(path)


def test_tracker_counts_full_cycles_with_repeated_interior_targets():
    route = compile_route(spec("ping_pong"), person="p")
    tracker = RouteTracker(route, .3)
    tracker.observe(0, 5, 5, route.points)
    for step in range(1, 13):
        x, y = route.points[(step - 1) % 6]
        tracker.observe(step * 25_000_000, x, y, tracker.queue(step % 6))
    assert tracker.report()["completed_cycles"] == 2
    assert [v["goal"] for v in tracker.visits] == list(route.names) * 2


@pytest.mark.parametrize("failure", ["skip", "premature", "missing", "rewind", "changed"])
def test_tracker_rejects_false_route_acceptance(failure):
    route = compile_route(spec(), person="p"); tracker = RouteTracker(route, .3)
    tracker.observe(0, 5, 5, route.points)
    stamp, x, y, goals = 25_000_000, 6., 6., tracker.queue(1)
    if failure == "skip": goals = tracker.queue(2)
    if failure == "premature": x = 5
    if failure == "missing": stamp = 50_000_000
    if failure == "rewind": stamp = -1
    if failure == "changed": goals = ((22., 22.),) + goals[1:]
    with pytest.raises(ValueError): tracker.observe(stamp, x, y, goals)


def test_compatibility_comparison_ignores_cdr_padding_but_detects_motion(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT / "scripts"))
    module_spec = importlib.util.spec_from_file_location(
        "multigoal_cpu_test", ROOT / "scripts/validate_multi_sfm_routes_cpu.py")
    module = importlib.util.module_from_spec(module_spec); module_spec.loader.exec_module(module)
    original, _ = load_people(ROOT / "config/hunav/multi_regular_1.yaml")
    other = copy.deepcopy(original)
    serialize = module.serialize_message

    def padded(message):
        data = bytearray(serialize(message))
        if message is other:
            # With frame_id="map", bytes 30/31 align the first Agent.name
            # after its int32 id and uint8 type/skin. They are not fields.
            data[30] ^= 0x55; data[31] ^= 0xaa
        return bytes(data)

    monkeypatch.setattr(module, "serialize_message", padded)
    assert padded(original) != padded(other)
    assert module.wire_equal(original, other)
    other.agents[0].position.position.x += .001
    assert not module.wire_equal(original, other)
