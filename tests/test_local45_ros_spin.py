"""Focused owned-executor and receipt CPU protocols; no SDK/DDS execution."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from scripts.isaac import g2_scene


class Node:
    def __init__(self, name):
        self.name = name
        self.context = SimpleNamespace(shutdown=Mock())
        self._executor = None
        self.destroy_node = Mock()

    @property
    def executor(self):
        return self._executor

    @executor.setter
    def executor(self, value):
        if self._executor is not None and self._executor is not value:
            self._executor.remove_node(self)
        self._executor = value


class Executor:
    def __init__(self, *, context, events, fail_add=False, fail_shutdown=False):
        self.context = context
        self.events = events
        self.fail_add, self.fail_shutdown = fail_add, fail_shutdown
        self.node = None
        self.add_calls = self.remove_calls = self.shutdown_calls = 0

    def add_node(self, node):
        self.add_calls += 1
        if self.fail_add:
            return False
        self.node = node
        node.executor = self
        return True

    def remove_node(self, node):
        self.remove_calls += 1
        self.events.append(("detach", node.name))

    def spin_once(self, *, timeout_sec):
        self.events.append(("spin", self.node.name, timeout_sec))

    def shutdown(self, *, timeout_sec):
        self.shutdown_calls += 1
        self.events.append(("shutdown", self.node.name if self.node else None, timeout_sec))
        if self.fail_shutdown:
            raise RuntimeError("shutdown failed")
        return True


class RosSpinTests(unittest.TestCase):
    def create(self, nodes, factory):
        with patch.object(g2_scene.importlib, "import_module",
                          return_value=SimpleNamespace(SingleThreadedExecutor=factory)):
            return g2_scene._create_node_executors(nodes)

    def test_nodes_register_once_with_actual_context_and_spin_four_fair_rounds(self):
        events, created = [], []
        nodes = [Node(name) for name in ("clock", "carter1", "carter2", "carter3", "carter4")]

        def factory(*, context):
            executor = Executor(context=context, events=events)
            created.append(executor)
            return executor

        owned = self.create(nodes, factory)
        rclpy = SimpleNamespace(spin_once=Mock(side_effect=AssertionError("global executor used")))
        for _ in range(2):
            g2_scene._spin_nodes(rclpy, nodes)
        self.assertEqual([("spin", node.name, 0.) for node in nodes]*8, events)
        self.assertEqual([1]*5, [executor.add_calls for executor in created])
        for node, executor in zip(nodes, created):
            self.assertIs(node.context, executor.context)
            self.assertIs(executor, node.executor)
        rclpy.spin_once.assert_not_called()
        nodes[0].executor = None
        with self.assertRaisesRegex(RuntimeError, "registration changed"):
            g2_scene._spin_nodes(rclpy, nodes)
        rclpy.spin_once.assert_not_called()
        g2_scene._shutdown_node_executors(owned)

    def test_legacy_nodes_keep_existing_global_spin_budget(self):
        nodes = [Node("one"), Node("two")]
        rclpy = SimpleNamespace(spin_once=Mock())
        g2_scene._spin_nodes(rclpy, nodes)
        self.assertEqual([(node,) for node in nodes]*4,
                         [call.args for call in rclpy.spin_once.call_args_list])
        self.assertTrue(all(call.kwargs == {"timeout_sec": 0.} for call in rclpy.spin_once.call_args_list))

    def test_shutdown_detaches_owned_executors_without_destroying_nodes_or_contexts(self):
        events = []
        nodes = [Node("one"), Node("two")]
        owned = self.create(nodes, lambda **kwargs: Executor(**kwargs, events=events))
        g2_scene._shutdown_node_executors(owned)
        self.assertEqual([("detach", "two"), ("shutdown", "two", 0.),
                          ("detach", "one"), ("shutdown", "one", 0.)], events)
        for node, executor in owned:
            self.assertIsNone(node.executor)
            self.assertFalse(hasattr(node, "_g2_owned_executor"))
            self.assertEqual(1, executor.shutdown_calls)
            node.destroy_node.assert_not_called()
            node.context.shutdown.assert_not_called()

    def test_partial_constructor_or_registration_failure_cleans_every_created_executor(self):
        for failure in ("constructor", "registration", "cleanup"):
            with self.subTest(failure=failure):
                events, created = [], []
                nodes = [Node("one"), Node("two")]
                original = ValueError("constructor failed")

                def factory(*, context):
                    if created and failure in ("constructor", "cleanup"):
                        raise original
                    executor = Executor(context=context, events=events,
                        fail_add=bool(created) and failure == "registration",
                        fail_shutdown=failure == "cleanup")
                    created.append(executor)
                    return executor

                with self.assertRaises((ValueError, RuntimeError)) as raised:
                    self.create(nodes, factory)
                if failure != "registration":
                    self.assertIs(original, raised.exception)
                self.assertTrue(all(executor.shutdown_calls == 1 for executor in created))
                for node in nodes:
                    self.assertIsNone(node.executor)
                    node.destroy_node.assert_not_called()
                    node.context.shutdown.assert_not_called()
        foreign = object()
        node = Node("foreign")
        node.executor = foreign
        factory = Mock(side_effect=AssertionError("foreign executor must not be replaced"))
        with self.assertRaisesRegex(RuntimeError, "not registered"):
            self.create([node], factory)
        self.assertIs(foreign, node.executor)
        factory.assert_not_called()

    def test_receipt_preserves_zero_counters_actual_topic_and_unknown_match(self):
        robots = {}
        for name, count, graph_count in (("carter1", 0, 0), ("carter2", 2, 3),
                                        ("carter3", None, 1), ("carter4", None, None)):
            node = Node(name)
            node._g2_cmd_subscription = SimpleNamespace(topic_name=f"/remapped/{name}/command")
            if count is not None:
                node._g2_cmd_subscription.get_publisher_count = lambda count=count: count
            if graph_count is not None:
                node.count_publishers = Mock(return_value=graph_count)
            robots[name] = {"node": node, "ros": {"command_sequence": 0, "cmd_vel": 0,
                "cmd_rejected": 0, "command_sim_time_s": None, "last_command_rejection": None}}
        robots["carter2"]["ros"].update(command_sequence=3, cmd_vel=3, cmd_rejected=1,
            command_sim_time_s=12.5, last_command_rejection={"type": "ValueError", "message": "bad", "count": 1})
        original = {key: deepcopy(value["ros"]) for key, value in robots.items()}
        result = g2_scene._command_receipt_state(robots, run_id="run071", physics_frame=800,
            sim_time_s=13., executor_mode="dedicated_single_threaded_executor_per_node")
        json.dumps(result, allow_nan=False)
        rows = result["robots"]
        self.assertEqual(0, rows["carter1"]["matched_publisher_count"])
        self.assertEqual(0, rows["carter1"]["graph_publisher_count"])
        self.assertEqual(0, rows["carter1"]["command_sequence"])
        self.assertIsNone(rows["carter1"]["command_sim_time_s"])
        self.assertEqual(2, rows["carter2"]["matched_publisher_count"])
        self.assertEqual(3, rows["carter2"]["graph_publisher_count"])
        self.assertEqual("/remapped/carter2/command", rows["carter2"]["resolved_cmd_topic"])
        self.assertEqual(original["carter2"]["last_command_rejection"], rows["carter2"]["last_command_rejection"])
        self.assertIsNone(rows["carter3"]["matched_publisher_count"])
        self.assertEqual(1, rows["carter3"]["graph_publisher_count"])
        self.assertEqual("matched_publisher_count", rows["carter3"]["diagnostic_errors"][0]["field"])
        self.assertIn("unavailable", rows["carter3"]["diagnostic_errors"][0]["message"])
        self.assertIsNone(rows["carter4"]["graph_publisher_count"])
        self.assertEqual("graph_publisher_count", rows["carter4"]["diagnostic_errors"][1]["field"])
        for name in ("carter1", "carter2", "carter3"):
            robots[name]["node"].count_publishers.assert_called_once_with(f"/remapped/{name}/command")
        self.assertEqual(original, {key: value["ros"] for key, value in robots.items()})

    def test_bad_count_and_topic_readbacks_are_unknown_not_zero(self):
        class Subscription:
            @property
            def topic_name(self):
                raise RuntimeError("topic readback failed")

            def get_publisher_count(self):
                return True

        node = Node("carter1")
        node._g2_cmd_subscription = Subscription()
        node.count_publishers = Mock(return_value=True)
        row = g2_scene._command_receipt_state({"carter1": {"node": node, "ros": {}}},
            run_id="run071", physics_frame=1, sim_time_s=1., executor_mode="test")["robots"]["carter1"]
        self.assertIsNone(row["resolved_cmd_topic"])
        self.assertIsNone(row["matched_publisher_count"])
        self.assertIsNone(row["graph_publisher_count"])
        self.assertEqual(3, len(row["diagnostic_errors"]))
        node.count_publishers.assert_not_called()
        node._g2_cmd_subscription = SimpleNamespace(topic_name="/carter1/cmd_vel", get_publisher_count=lambda: 0)
        row = g2_scene._command_receipt_state({"carter1": {"node": node, "ros": {}}},
            run_id="run071", physics_frame=1, sim_time_s=1., executor_mode="test")["robots"]["carter1"]
        self.assertEqual(0, row["matched_publisher_count"])
        self.assertIsNone(row["graph_publisher_count"])
        self.assertEqual("graph_publisher_count", row["diagnostic_errors"][0]["field"])

    def test_snapshot_write_failure_keeps_existing_primary_error(self):
        primary = {"type": "ValueError", "message": "existing primary"}
        result = {"error": primary, "status": "failed"}
        with patch.object(g2_scene, "_atomic_json", side_effect=OSError("snapshot write failed")):
            g2_scene._save_command_receipt_state(Path("unused"), {}, result,
                run_id="run071", physics_frame=1, sim_time_s=1., executor_mode="test")
        self.assertIs(primary, result["error"])
        self.assertEqual("failed", result["status"])
        self.assertIn("snapshot write failed", result["command_receipt_checkpoint_error"])

    def test_existing_callback_retains_last_type_value_error_without_changing_counts_or_qos(self):
        import builtins
        original_import = builtins.__import__
        modules = {"geometry_msgs.msg": SimpleNamespace(TransformStamped=object, Twist=object),
            "nav_msgs.msg": SimpleNamespace(Odometry=object), "sensor_msgs.msg": SimpleNamespace(LaserScan=object),
            "rclpy.qos": SimpleNamespace(QoSProfile=lambda **kwargs: kwargs,
                                          ReliabilityPolicy=SimpleNamespace(BEST_EFFORT="best_effort"))}

        def imported(name, *args, **kwargs):
            return modules[name] if name in modules else original_import(name, *args, **kwargs)

        node = Node("carter1")
        subscriptions = []

        def subscribe(msg_type, topic, callback, qos):
            subscriptions.append((topic, callback, qos))
            return SimpleNamespace(topic_name="/carter1/"+topic)

        node.create_subscription = subscribe
        node.create_publisher = Mock()
        watchdog = SimpleNamespace(receive=Mock())
        broadcaster = lambda node: SimpleNamespace(sendTransform=Mock())
        with patch("builtins.__import__", side_effect=imported):
            _, state = g2_scene._setup_robot_ros(SimpleNamespace(create_node=lambda *a, **kw: node),
                {"namespace": "carter1"}, [], None, watchdog, broadcaster_types=(broadcaster, broadcaster))
        callback = subscriptions[0][1]
        self.assertEqual(("cmd_vel", 50), (subscriptions[0][0], subscriptions[0][2]))
        state["latest_sim_time"] = 12.5
        callback(SimpleNamespace(linear=SimpleNamespace(x="bad"), angular=SimpleNamespace(z=0.)))
        self.assertEqual("ValueError", state["last_command_rejection"]["type"])
        watchdog.receive.side_effect = TypeError("existing watchdog type rejection")
        message = SimpleNamespace(linear=SimpleNamespace(x=.4), angular=SimpleNamespace(z=0.))
        callback(message)
        self.assertEqual({"type": "TypeError", "message": "existing watchdog type rejection", "count": 2},
                         state["last_command_rejection"])
        watchdog.receive.side_effect = None
        callback(message)
        self.assertEqual((1, 1, 2, 12.5), (state["command_sequence"], state["cmd_vel"],
                                          state["cmd_rejected"], state["command_sim_time_s"]))
        self.assertEqual(2, state["last_command_rejection"]["count"])


if __name__ == "__main__":
    unittest.main()
