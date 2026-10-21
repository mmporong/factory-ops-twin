"""CPU launch-plan checks only; no ROS process, timer execution or native claim."""
from __future__ import annotations

import ast
import importlib.util
import json
import os
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch


PATH = Path(__file__).resolve().parents[1]/"scripts/ros/fleet4.launch.py"
SPEC = importlib.util.spec_from_file_location("fleet4_launch_143", PATH)
assert SPEC is not None and SPEC.loader is not None
LAUNCH = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LAUNCH)
NAMES = ["carter1", "carter2", "carter3", "carter4"]


class Fleet4LaunchTests(unittest.TestCase):
    def plan(self, stagger=None, names=None):
        environment = {"FACTORY_OPS_FLEET_NAMESPACES":json.dumps(NAMES if names is None else names),
                       "FACTORY_OPS_NAV2_PARAMS":"observed-params.yaml"}
        if stagger is not None:
            environment["FACTORY_OPS_NAV2_STARTUP_STAGGER_S"] = stagger
        modules = {
            "launch":NS(LaunchDescription=lambda actions:actions),
            "launch.actions":NS(TimerAction=lambda **kwargs:{"timer":kwargs}),
            "launch_ros":NS(),
            "launch_ros.actions":NS(Node=lambda **kwargs:kwargs,
                                    SetParameter=lambda *args:{"setting":args}),
        }
        with patch.dict(os.environ,environment,clear=True), patch.dict("sys.modules",modules):
            return LAUNCH.generate_launch_description()

    def test_default_and_explicit_zero_preserve_immediate_stack_plan(self):
        actions = self.plan()
        self.assertEqual(actions,self.plan("0"))
        self.assertEqual(actions[0],{"setting":("use_sim_time",True)})
        self.assertEqual(len(actions),37)
        self.assertFalse(any("timer" in row for row in actions))
        for index,name in enumerate(NAMES):
            group = actions[1+index*9:1+(index+1)*9]
            self.assertEqual({name},{row["namespace"] for row in group})
            self.assertEqual(len(group),9)

    def test_opt_in_defers_complete_namespace_groups_without_changing_nodes(self):
        baseline = self.plan()[1:]
        actions = self.plan("2.5")
        self.assertEqual(actions[0],{"setting":("use_sim_time",True)})
        self.assertEqual(actions[1:10],baseline[:9])
        timers = [row["timer"] for row in actions[10:]]
        self.assertEqual([row["period"] for row in timers],[2.5,5.,7.5])
        flattened = actions[1:10]
        for index,timer in enumerate(timers,1):
            self.assertIs(timer["cancel_on_shutdown"],True)
            self.assertEqual(timer["actions"],baseline[index*9:(index+1)*9])
            flattened += timer["actions"]
        self.assertEqual(flattened,baseline)
        for row in flattened:
            if row["executable"] == "lifecycle_manager":
                parameters = row["parameters"][0]
                self.assertIs(parameters["autostart"],True)
                self.assertEqual(parameters["bond_timeout"],20.)
                self.assertEqual(len(parameters["node_names"]),8)
            else:
                self.assertEqual(row["parameters"],["observed-params.yaml"])
                self.assertEqual(row["remappings"],[("/tf","tf"),("/tf_static","tf_static")])

    def test_invalid_spacing_and_unbounded_total_delay_fail_before_launch(self):
        for value in ("-0.01","5.01","NaN","inf","-inf","","true","2 seconds"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.plan(value)
        for value in (None,True,[] ):
            with self.subTest(value=value), self.assertRaises(ValueError):
                LAUNCH.startup_stagger_s({"FACTORY_OPS_NAV2_STARTUP_STAGGER_S":value})
        self.assertEqual(self.plan("5")[-1]["timer"]["period"],15.)
        with self.assertRaisesRegex(ValueError,"15 wall seconds"):
            self.plan("5",names=NAMES+["carter5"])

    def test_python310_syntax_without_importing_ros(self):
        for path in (PATH,Path(__file__)):
            ast.parse(path.read_text(encoding="utf-8"),feature_version=(3,10))


if __name__ == "__main__":
    unittest.main()
