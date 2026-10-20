import unittest
import yaml
from scripts.ros.g3_mission import _recording_topics, _scene_runtime, bag_qos_overrides


class Local45RecordingTests(unittest.TestCase):
    def test_local_sensor_recording_does_not_depend_on_discovery_qos(self):
        ids = ('carter1', 'carter2', 'carter3', 'carter4')
        qos = yaml.safe_load(bag_qos_overrides(ids, local45=True))
        for robot in ids:
            for topic in ('scan_front_raw', 'scan_rear_raw', 'odom', 'tf'):
                self.assertEqual('best_effort' if topic.endswith('_raw') else 'reliable',
                    qos[f'/{robot}/{topic}']['reliability'])
                self.assertEqual('volatile', qos[f'/{robot}/{topic}']['durability'])
            self.assertEqual('transient_local', qos[f'/{robot}/tf_static']['durability'])
            self.assertEqual('reliable', qos[f'/{robot}/controller_cycle_trace']['reliability'])
        canonical = yaml.safe_load(bag_qos_overrides(ids))
        self.assertNotIn('/carter1/scan_front_raw', canonical)
        self.assertNotIn('/carter1/odom', canonical)

    def test_development_keeps_all_eight_raw_sensors_and_control_without_filtered_copies(self):
        raw = [f"/carter{robot}/scan_{side}_raw" for robot in range(1, 5) for side in ("front", "rear")]
        copies = [f"/carter{robot}/{name}" for robot in range(1, 5)
                  for name in ("scan_front", "scan_rear", "scan_localization")]
        control = ["/clock", "/carter1/odom", "/carter1/tf", "/carter1/tf_static", "/carter1/cmd_vel",
                   "/carter1/follow_path/_action/status", "/factory/g3/authorizations"]
        self.assertEqual(raw + control, _recording_topics(raw + copies + control, "local45_development"))
        _file, fields = _scene_runtime(True, "local45_development")
        self.assertIn("not canonical qualification", fields["recording_policy"])

    def test_default_canonical_recording_and_runtime_fields_remain_unchanged(self):
        topics = ["/carter1/scan_front_raw", "/carter1/scan_front", "/carter1/scan_localization"]
        self.assertEqual(topics, _recording_topics(topics, "canonical62"))
        self.assertEqual(("fleet4-transport-scene.json", {}), _scene_runtime(True, "canonical62"))
        with self.assertRaises(ValueError):
            _scene_runtime(False, "local45_development")


if __name__ == "__main__":
    unittest.main()
