"""CPU World/ROS protocol fixtures; no SDK, DDS delivery, or runtime proof."""
import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from factory_ops.world_clock import WorldClockPublisher, create_ros_world_clock


def clock_owner(world, callback=None):
    callback = Mock() if callback is None else callback
    return WorldClockPublisher(callback, lambda: world.time, lambda: world.frame), callback


class WireClock:
    def __init__(self):
        self.clock = SimpleNamespace(sec=0, nanosec=0)


def ros_protocol():
    publisher, subscription = Mock(), object()
    node = SimpleNamespace(create_publisher=Mock(return_value=publisher),
        create_subscription=Mock(return_value=subscription), destroy_node=Mock())
    ros = SimpleNamespace(init=Mock(), create_node=Mock(return_value=node), shutdown=Mock())
    extensions = SimpleNamespace(set_extension_enabled_immediate=Mock())
    kit = SimpleNamespace(get_app=lambda: SimpleNamespace(get_extension_manager=lambda: extensions))
    qos = SimpleNamespace(QoSProfile=lambda **kw: SimpleNamespace(**kw),
        ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable", BEST_EFFORT="best_effort"),
        HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
        DurabilityPolicy=SimpleNamespace(VOLATILE="volatile"))
    modules = {"omni.kit.app": kit, "rclpy": ros, "rclpy.qos": qos,
               "rosgraph_msgs.msg": SimpleNamespace(Clock=WireClock)}
    return modules, ros, node, publisher, subscription, extensions


class WorldClockTests(unittest.TestCase):
    def test_warmup_paused_duplicate_then_next_actual_frame_exact_time(self):
        world = SimpleNamespace(time=0., frame=0)
        owner, callback = clock_owner(world)
        self.assertEqual("warmup", owner.publish()["status"])
        callback.assert_not_called()
        world.time, world.frame = 14.933334112, 896
        self.assertTrue(owner.publish()["published"])
        callback.assert_called_once_with(14, 933334112)
        self.assertEqual("duplicate", owner.publish()["status"])
        self.assertEqual("duplicate", owner.publish()["status"])
        self.assertEqual(1, callback.call_count)
        world.time, world.frame = 14.950000779, 897
        owner.publish()
        self.assertEqual((14, 950000779), callback.call_args.args)
        report = owner.report()
        self.assertEqual((2, 2, 1), (report["published_count"], report["duplicate_count"], report["warmup_count"]))
        self.assertEqual((world.time, world.frame), (report["last_world_time_s"], report["last_physics_frame"]))
        self.assertFalse(report["sensor_acquisition"])
        self.assertFalse(report["native_timestamp_claimed"])
        self.assertFalse(report["hardware_timestamp_claimed"])
        self.assertIsNone(report["qos_policy"])
        json.dumps(report, allow_nan=False)

    def test_nanosecond_carry_keeps_original_world_value(self):
        world = SimpleNamespace(time=1.9999999996, frame=120)
        owner, callback = clock_owner(world)
        owner.publish()
        callback.assert_called_once_with(2, 0)
        self.assertEqual(world.time, owner.report()["last_published_world_time_s"])
        self.assertEqual({"sec": 2, "nanosec": 0}, owner.report()["last_published_stamp"])

    def test_invalid_time_frame_and_ros_seconds_are_rejected(self):
        cases = [(True, 1), ("1", 1), (float("nan"), 1), (float("inf"), 1),
                 (-.1, 1), (2 ** 31, 1), (10 ** 400, 1), (1., True),
                 (1., 1.5), (1., -1), (1., "1")]
        for stamp, frame in cases:
            owner, callback = clock_owner(SimpleNamespace(time=stamp, frame=frame))
            with self.subTest(time=repr(stamp), frame=frame), self.assertRaises(ValueError):
                owner.publish()
            callback.assert_not_called()
            self.assertEqual(0, owner.report()["published_count"])

    def test_resets_and_same_frame_changed_time_fail_even_on_cached_reads(self):
        for stamp, frame in ((1., 76), (1.3, 74), (1.26, 75), (1.25, 76)):
            world = SimpleNamespace(time=1.25, frame=75)
            owner, callback = clock_owner(world)
            owner.publish()
            world.time, world.frame = stamp, frame
            with self.subTest(time=stamp, frame=frame), self.assertRaises(ValueError):
                owner.publish()
            self.assertEqual(1, callback.call_count)
            self.assertEqual((1.25, 75), (owner.report()["last_world_time_s"], owner.report()["last_physics_frame"]))

    def test_failed_callback_retries_same_pair_without_consuming_publication(self):
        callback = Mock(side_effect=[OSError("publish failed"), None])
        world = SimpleNamespace(time=1.25, frame=75)
        owner, _ = clock_owner(world, callback)
        with self.assertRaisesRegex(OSError, "publish failed"):
            owner.publish()
        self.assertEqual(0, owner.report()["published_count"])
        self.assertIsNone(owner.report()["last_published_stamp"])
        world.time = 1.3
        with self.assertRaisesRegex(ValueError, "same World physics frame"):
            owner.publish()
        world.time = 1.25
        self.assertEqual("published", owner.publish()["status"])
        self.assertEqual((1, 0, 2), (owner.published_count, owner.duplicate_count, callback.call_count))

    def test_factory_creates_one_wire_owner_with_explicit_qos_and_received_values(self):
        modules, ros, node, publisher, subscription, extensions = ros_protocol()
        app = SimpleNamespace(update=Mock())
        manager = SimpleNamespace(get_simulation_time=Mock(return_value=4.125),
                                  get_num_physics_steps=Mock(return_value=248))
        with patch("factory_ops.world_clock.importlib.import_module", side_effect=modules.__getitem__):
            returned_ros, returned_node, stamps, owner = create_ros_world_clock(app, manager)
        self.assertIs(ros, returned_ros)
        self.assertIs(node, returned_node)
        extensions.set_extension_enabled_immediate.assert_called_once_with("isaacsim.ros2.bridge", True)
        app.update.assert_called_once_with()
        ros.init.assert_called_once_with()
        ros.create_node.assert_called_once_with("factory_local45_world_clock_owner")
        node.create_publisher.assert_called_once()
        node.create_subscription.assert_called_once()
        pub_type, pub_topic, pub_qos = node.create_publisher.call_args.args
        sub_type, sub_topic, received, sub_qos = node.create_subscription.call_args.args
        self.assertIs(WireClock, pub_type)
        self.assertIs(WireClock, sub_type)
        self.assertEqual(("/clock", "/clock"), (pub_topic, sub_topic))
        self.assertEqual((10, "reliable", "keep_last", "volatile"),
                         (pub_qos.depth, pub_qos.reliability, pub_qos.history, pub_qos.durability))
        self.assertEqual((10, "best_effort", "keep_last", "volatile"),
                         (sub_qos.depth, sub_qos.reliability, sub_qos.history, sub_qos.durability))
        self.assertIs(node._factory_clock_publisher, publisher)
        self.assertIs(node._factory_clock_subscription, subscription)
        self.assertIs(node._factory_world_clock_owner, owner)
        owner.publish()
        message = publisher.publish.call_args.args[0]
        self.assertEqual((4, 125000000), (message.clock.sec, message.clock.nanosec))
        self.assertEqual([], stamps)  # Sending is not receiving.
        incoming = WireClock()
        incoming.clock.sec, incoming.clock.nanosec = 9, 250000001
        received(incoming)
        self.assertEqual([9.250000001], stamps)
        self.assertEqual({"sec": 9, "nanosec": 250000001}, owner.report()["last_received_clock_stamp"])
        self.assertEqual(1, owner.report()["received_clock_count"])
        self.assertEqual(4.125, owner.report()["last_world_time_s"])
        self.assertEqual("RELIABLE", owner.report()["qos_policy"]["publisher"]["reliability"])
        manager.get_simulation_time.assert_called_once_with()
        manager.get_num_physics_steps.assert_called_once_with()
        node.destroy_node.assert_not_called()
        ros.shutdown.assert_not_called()

    def test_factory_partial_failure_cleans_only_its_initialized_context(self):
        for init_failure in (False, True):
            modules, ros, node, _, _, _ = ros_protocol()
            if init_failure:
                ros.init.side_effect = RuntimeError("foreign context")
            else:
                node.create_subscription.side_effect = RuntimeError("subscription failed")
            with patch("factory_ops.world_clock.importlib.import_module", side_effect=modules.__getitem__):
                with self.assertRaises(RuntimeError):
                    create_ros_world_clock(SimpleNamespace(update=Mock()), SimpleNamespace(
                        get_simulation_time=lambda: 1., get_num_physics_steps=lambda: 60))
            if init_failure:
                node.destroy_node.assert_not_called()
                ros.shutdown.assert_not_called()
            else:
                node.destroy_node.assert_called_once_with()
                ros.shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
