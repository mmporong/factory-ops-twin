"""ROS/Jazzy-only regression: delivery history must cover a callback pause.

Run explicitly in the ROS environment; this file is outside CPU test discovery.
"""
from pathlib import Path
import json
import time
import unittest
from uuid import uuid4

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry

from factory_ops.energy_runtime import EnergyRuntime
from scripts.ros.g3_mission import mission_odometry_qos


class OdometryBacklogTests(unittest.TestCase):
    def test_reliable_history_preserves_energy_samples_across_callback_pause(self):
        profile = json.loads((Path(__file__).resolve().parents[2] /
            'configs/factory-fleet4/energy-profile.json').read_text())
        runtimes = {name: EnergyRuntime('backlog', profile, {'robot': .8})
                    for name in ('sensor', 'accounting')}
        counts = {name: 0 for name in runtimes}
        faults = {}
        rclpy.init()
        node = rclpy.create_node('odometry_backlog_' + uuid4().hex)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        topic = 'odometry_' + uuid4().hex
        publisher = node.create_publisher(Odometry, topic, mission_odometry_qos())

        def receive(name, message):
            counts[name] += 1
            try:
                runtimes[name].observe('backlog', 'robot',
                    message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                    message.twist.twist.linear.x, message.twist.twist.angular.z,
                    loaded=False)
            except ValueError as error:
                faults[name] = str(error)

        subscriptions = [node.create_subscription(Odometry, topic,
            lambda message, key=name: receive(key, message), qos)
            for name, qos in (('sensor', qos_profile_sensor_data),
                              ('accounting', mission_odometry_qos()))]

        def drain_until(condition):
            deadline = time.monotonic() + 5.
            while not condition() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.02)
            self.assertTrue(condition(), str(counts))

        def publish(index):
            message = Odometry()
            stamp_ns = 1_000_000_000 + index * 50_000_000
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            message.twist.twist.linear.x = .1
            publisher.publish(message)

        try:
            drain_until(lambda: publisher.get_subscription_count() == 2)
            publish(0)
            drain_until(lambda: counts == {'sensor': 1, 'accounting': 1})
            # DDS receives 12 samples at 20 Hz while application callbacks pause.
            # No timestamp is synthesized in either accounting runtime.
            for index in range(1, 13):
                publish(index)
                time.sleep(.02)
            time.sleep(.1)
            drain_until(lambda: counts['accounting'] == 13)
            for _ in range(10):
                executor.spin_once(timeout_sec=.01)
            self.assertIn('sensor', faults)
            self.assertNotIn('accounting', faults)
            self.assertLess(counts['sensor'], counts['accounting'])
            state = runtimes['accounting'].projection('backlog', 'robot', 1.6)
            self.assertEqual('fresh', state['freshness'])
            self.assertEqual(1.6, state['observed_sim_time_s'])
            expected_wh = 45. * .6 / 3600 + .05 * .1 * .6
            self.assertAlmostEqual(expected_wh,
                runtimes['accounting'].snapshot()['robots']['robot']['consumed_wh'])
            print(json.dumps({'callbacks': counts, 'old_profile_faulted': 'sensor' in faults,
                              'new_profile_faulted': 'accounting' in faults,
                              'energy_max_gap_s': profile['max_observation_gap_s']}))
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
