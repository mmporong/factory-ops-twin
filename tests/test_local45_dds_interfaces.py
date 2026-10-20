"""Run-local profile/launch wiring only; all external process calls are mocked."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

from scripts.isaac import run_local45_fleet_pair as pair


NS = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}


class DdsInterfaceTests(unittest.TestCase):
    def test_default_profile_and_environment_remain_unscoped(self):
        with TemporaryDirectory() as temporary:
            stage = Path(temporary)
            profile, environment = pair.prepare_ros_transport(stage, 17)
            self.assertEqual(stage / "fastdds-udp.xml", profile)
            self.assertEqual({"ROS_DOMAIN_ID": "17", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                "ROS_LOCALHOST_ONLY": "0", "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile),
                "FASTDDS_DEFAULT_PROFILES_FILE": str(profile)}, environment)
            root = ET.parse(profile).getroot()
            self.assertEqual("UDPv4", root.findtext("dds:transport_descriptors/dds:transport_descriptor/dds:type", namespaces=NS))
            self.assertEqual("false", root.findtext("dds:participant/dds:rtps/dds:useBuiltinTransports", namespaces=NS))
            self.assertEqual([], root.findall(".//dds:interfaceWhiteList", NS))
            original = profile.read_bytes()
            with self.assertRaises(FileExistsError):
                pair.prepare_ros_transport(stage, 17)
            self.assertEqual(original, profile.read_bytes())

    def test_paired_profiles_use_platform_addresses_environments_hashes_and_inputs(self):
        with TemporaryDirectory() as temporary:
            stage, sdk = Path(temporary) / "stage", Path(temporary) / "sdk"
            output = stage / "results/run"
            for directory in (output / "provenance", stage / "source", stage / "inputs", sdk):
                directory.mkdir(parents=True)

            def write(path, data):
                path.write_text(json.dumps(data), encoding="utf-8")

            run_id = "a" * 32
            metadata = output / "provenance/fleet-source.json"
            write(metadata, {"source_files": {}})
            (stage / "source.tar").write_bytes(b"CPU fixture only")
            write(stage / "build-attestation.json", {"actual_install_directory": str(sdk)})
            write(stage / "inputs/config.json", {"ros_domain_id": 17})
            write(stage / "launch.json", {"run_id": run_id, "source_commit": "fixture",
                "provenance_sha256": pair.sha(metadata), "source_archive_sha256": pair.sha(stage / "source.tar"),
                "build_attestation_sha256": pair.sha(stage / "build-attestation.json")})
            write(output / "fleet-transport-verified.json", {"fixture": True})
            write(output / "mission-result.json", {"status": "passed"})
            write(output / "local45-fleet4-transport-scene.json", {"status": "validated"})
            write(stage / "job.json", {"run_id": run_id, "closed": True,
                "remaining_owned_pids": [], "status": "passed"})
            owner = Mock(unit_name="factory-ops-fixture")
            owner.start.return_value = {"fixture": True}
            owner.stop.return_value = {"closure_verified": True}
            worker = Mock(pid=123, returncode=0)
            worker.poll.return_value = 0
            with patch.object(pair, "WslProcessSupervisor", return_value=owner), \
                 patch.object(pair.subprocess, "Popen", return_value=worker) as popen, \
                 patch.object(pair, "monitor_pair", return_value=None), \
                 patch.object(pair, "wait_for_bootstrap_marker"):
                self.assertEqual(0, pair.run(stage, sdk, 450, lidar_backend="physx_planar",
                    windows_dds_interface="192.0.2.10", linux_dds_interface="192.0.2.11"))
            report = json.loads((stage / "pair-host.json").read_text(encoding="utf-8"))
            expected = json.loads((stage / "job-spec.json").read_text(encoding="utf-8"))["expected_inputs"]
            profiles = report["ros_transport"]["interface_whitelist"]
            for platform, address in (("windows", "192.0.2.10"), ("linux", "192.0.2.11")):
                profile = stage / f"fastdds-udp-{platform}.xml"
                root = ET.parse(profile).getroot()
                self.assertEqual([address], [element.text for element in root.findall(".//dds:interfaceWhiteList/dds:address", NS)])
                self.assertEqual("UDPv4", root.findtext("dds:transport_descriptors/dds:transport_descriptor/dds:type", namespaces=NS))
                self.assertEqual("false", root.findtext("dds:participant/dds:rtps/dds:useBuiltinTransports", namespaces=NS))
                self.assertEqual({"address": address, "profile": str(profile), "sha256": pair.sha(profile)}, profiles[platform])
                self.assertEqual(pair.sha(profile), expected[str(profile)])
            windows_profile = str(stage / "fastdds-udp-windows.xml")
            linux_profile = pair.linux(stage / "fastdds-udp-linux.xml")
            for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
                self.assertEqual(windows_profile, popen.call_args.kwargs["env"][key])
                self.assertEqual(windows_profile, report["ros_transport"]["windows_environment"][key])
                self.assertEqual(linux_profile, report["ros_transport"]["linux_environment"][key])
                self.assertIn(f"export {key}={linux_profile}", (stage / "nav-command.sh").read_text(encoding="utf-8"))
            command = json.loads((stage / "job-spec.json").read_text(encoding="utf-8"))["command"]
            self.assertIn("--lidar-backend physx_planar --physics-cadence owned_step6", command)
            self.assertTrue(report["windows_closed"] and report["linux_closed"])
            self.assertFalse((stage / "fastdds-udp.xml").exists())

    def test_partial_or_invalid_interface_pair_rejects_before_files_or_processes(self):
        for windows, linux_address in (("192.0.2.10", None), (None, "192.0.2.11"),
                ("invalid", "192.0.2.11"), ("192.0.2.10", "::1"),
                ("192.0.2.10", "192.0.2.999"), (True, "192.0.2.11"),
                ("192.0.2.10</address>", "192.0.2.11")):
            with self.subTest(windows=windows, linux=linux_address), TemporaryDirectory() as temporary:
                stage = Path(temporary)
                with patch.object(pair, "save") as save, patch.object(pair, "WslProcessSupervisor") as owner, \
                     patch.object(pair.subprocess, "Popen") as popen:
                    with self.assertRaises(ValueError):
                        pair.run(stage, stage, 450, windows_dds_interface=windows, linux_dds_interface=linux_address)
                self.assertEqual([], list(stage.iterdir()))
                save.assert_not_called()
                owner.assert_not_called()
                popen.assert_not_called()


if __name__ == "__main__":
    unittest.main()
