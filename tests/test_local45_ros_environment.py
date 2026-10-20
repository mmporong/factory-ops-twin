import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from scripts.isaac.run_local45_fleet_pair import prepare_ros_transport
from scripts.isaac.g2_scene import _ros_context_details


class RosEnvironmentTests(unittest.TestCase):
    def test_run_owned_udp_profile_and_explicit_environment(self):
        with tempfile.TemporaryDirectory() as folder:
            profile, environment = prepare_ros_transport(Path(folder), 73)
            namespace = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
            tree = ET.parse(profile)
            participant = tree.find('d:participant', namespace)
            self.assertEqual('true', participant.attrib['is_default_profile'])
            self.assertEqual('false', participant.find('d:rtps/d:useBuiltinTransports', namespace).text)
            descriptor = tree.find('d:transport_descriptors/d:transport_descriptor', namespace)
            self.assertEqual('UDPv4', descriptor.find('d:type', namespace).text)
            self.assertEqual(descriptor.find('d:transport_id', namespace).text,
                participant.find('d:rtps/d:userTransports/d:transport_id', namespace).text)
            self.assertEqual('73', environment['ROS_DOMAIN_ID'])
            self.assertEqual('rmw_fastrtps_cpp', environment['RMW_IMPLEMENTATION'])
            self.assertEqual('0', environment['ROS_LOCALHOST_ONLY'])
            for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
                self.assertEqual(str(profile), environment[key])
            original = profile.read_bytes()
            with self.assertRaises(FileExistsError):
                prepare_ros_transport(Path(folder), 73)
            self.assertEqual(original, profile.read_bytes())

    def test_invalid_domain_does_not_create_profile(self):
        for domain in (True, '73', -1, 233):
            with self.subTest(domain=domain), tempfile.TemporaryDirectory() as folder:
                with self.assertRaises(ValueError):
                    prepare_ros_transport(Path(folder), domain)
                self.assertEqual([], list(Path(folder).iterdir()))

    def context(self, domain=73, rmw='rmw_fastrtps_cpp'):
        module = SimpleNamespace(__file__='bundled-humble/rclpy/__init__.py',
            utilities=SimpleNamespace(get_rmw_implementation_identifier=lambda: rmw))
        node = SimpleNamespace(context=SimpleNamespace(get_domain_id=lambda: domain),
            get_namespace=lambda: '/carter2')
        return module, node

    def test_observation_uses_actual_context_instead_of_environment(self):
        module, node = self.context()
        with patch.dict('os.environ', {'ROS_DOMAIN_ID': '0'}):
            observation = _ros_context_details(module, node, 73)
        self.assertEqual(73, observation['domain_id'])
        self.assertEqual('/carter2', observation['namespace'])
        self.assertEqual(module.__file__, observation['rclpy_file'])
        json.dumps(observation, allow_nan=False)

    def test_domain_or_middleware_mismatch_fails_with_actual_values(self):
        for domain, rmw in ((0, 'rmw_fastrtps_cpp'), (73, 'rmw_cyclonedds_cpp')):
            with self.subTest(domain=domain, rmw=rmw):
                module, node = self.context(domain, rmw)
                with self.assertRaisesRegex(RuntimeError, 'context mismatch') as caught:
                    _ros_context_details(module, node, 73)
                self.assertIn(str(domain), str(caught.exception))
                self.assertIn(rmw, str(caught.exception))


if __name__ == '__main__':
    unittest.main()
