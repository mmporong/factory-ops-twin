from __future__ import annotations

import copy
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.layout import Layout
from scripts.ros.g1_mission import LifecycleStatePoller, prepare_params


ROOT = Path(__file__).resolve().parents[1]


class _Future:
    def __init__(self, state: int, *, done: bool) -> None:
        self.state = state
        self.complete = done

    def done(self) -> bool:
        return self.complete

    def result(self) -> object:
        return SimpleNamespace(current_state=SimpleNamespace(id=self.state))


class _Client:
    def __init__(self, futures: list[_Future]) -> None:
        self.futures = futures
        self.requests: list[object] = []

    @staticmethod
    def service_is_ready() -> bool:
        return True

    def call_async(self, request: object) -> _Future:
        self.requests.append(request)
        return self.futures.pop(0)


class LifecycleStatePollerTests(unittest.TestCase):
    def test_immediate_responses_remain_bounded_to_two_requests_per_second(self) -> None:
        client = _Client([_Future(1, done=True), _Future(1, done=True), _Future(1, done=True)])
        poller = LifecycleStatePoller(["controller_server"])

        for index in range(100):
            poller.poll(index / 100.0, {"controller_server": client}, object)

        self.assertEqual(len(client.requests), 2)

    def test_pending_future_is_not_resent_after_interval(self) -> None:
        pending = _Future(1, done=False)
        client = _Client([pending])
        poller = LifecycleStatePoller(["controller_server"])

        poller.poll(0.0, {"controller_server": client}, object)
        poller.poll(5.0, {"controller_server": client}, object)

        self.assertEqual(len(client.requests), 1)
        self.assertIsNone(poller.states["controller_server"])

    def test_active_state_is_cached_without_further_requests(self) -> None:
        client = _Client([_Future(3, done=True)])
        poller = LifecycleStatePoller(["controller_server"])

        poller.poll(0.0, {"controller_server": client}, object)
        states = poller.poll(0.01, {"controller_server": client}, object)
        poller.poll(10.0, {"controller_server": client}, object)

        self.assertEqual(states, {"controller_server": 3})
        self.assertEqual(len(client.requests), 1)


class PrepareParamsTests(unittest.TestCase):
    def setUp(self) -> None:
        self.layout = Layout.from_file(ROOT / "configs/factory-v3/layout.json")
        self.config = json.loads((ROOT / "configs/factory-v3/g1.json").read_text(encoding="utf-8"))
        self.ready = {
            "footprint": {
                "passed": True,
                "collision_radius_m": 0.637109,
                "clearance_m": 0.2,
                "inflated_radius_m": 0.837109,
            }
        }

    def test_planning_radius_adds_localization_margin_without_changing_evaluator(self) -> None:
        template = {
            "navigation_map_server": {"ros__parameters": {}},
            "map_server": {"ros__parameters": {}},
            "local_costmap": {"local_costmap": {"ros__parameters": {
                "robot_radius": 0.0, "inflation_layer": {"inflation_radius": 0.0},
            }}},
            "global_costmap": {"global_costmap": {"ros__parameters": {
                "robot_radius": 0.0, "inflation_layer": {"inflation_radius": 0.0},
            }}},
        }
        fake_yaml = SimpleNamespace(
            safe_load=lambda _text: copy.deepcopy(template),
            safe_dump=lambda value, sort_keys=False: json.dumps(value),
        )
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            with patch.dict(sys.modules, {"yaml": fake_yaml}):
                params_path = prepare_params(ROOT, output, self.ready, self.layout, self.config)
            params = json.loads(params_path.read_text(encoding="utf-8"))
            provenance = json.loads((output / "maps/provenance.json").read_text(encoding="utf-8"))

        for name in ("local_costmap", "global_costmap"):
            costmap = params[name][name]["ros__parameters"]
            self.assertAlmostEqual(costmap["robot_radius"], 1.137109)
            self.assertAlmostEqual(costmap["inflation_layer"]["inflation_radius"], 1.337109)
        self.assertEqual(provenance["evaluation_clearance_m"], 0.2)
        self.assertAlmostEqual(provenance["evaluation_radius_m"], 0.837109)
        self.assertEqual(provenance["localization_planning_margin_m"], 0.3)
        self.assertAlmostEqual(provenance["nav2_planning_radius_m"], 1.137109)

    def test_invalid_localization_planning_margin_is_rejected(self) -> None:
        for invalid in (0, -0.1, float("nan"), float("inf"), True, "0.3"):
            config = dict(self.config)
            config["localization_planning_margin_m"] = invalid
            with self.subTest(value=invalid), tempfile.TemporaryDirectory() as folder:
                with self.assertRaisesRegex(ValueError, "finite and positive"):
                    prepare_params(ROOT, Path(folder), self.ready, self.layout, config)

if __name__ == "__main__":
    unittest.main()
