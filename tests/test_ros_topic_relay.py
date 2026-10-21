import base64
import hashlib
import struct
import unittest
from pathlib import Path
import tempfile
from factory_ops.ros_topic_relay import Framer,Receiver,catalog


class RelayTests(unittest.TestCase):
    def test_checkpoint_retries_transient_windows_sharing_conflict(self):
        from unittest.mock import patch
        import os,json
        from factory_ops.ros_topic_relay import write_checkpoint
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/"relay.json";target.write_text("old",encoding="utf-8")
            replace=os.replace;attempts=[]
            def locked_once(source,destination):
                attempts.append(1)
                if len(attempts)==1:raise PermissionError("observed sharing violation")
                replace(source,destination)
            with patch("factory_ops.compat.os.replace",side_effect=locked_once):
                write_checkpoint(target,{"status":"ready"})
            self.assertEqual(len(attempts),2)
            self.assertEqual(json.loads(target.read_text(encoding="utf-8")),{"status":"ready"})
    def packet(self,seq=1,topic="/carter1/cmd_vel",stamp=10,raw=b"unchanged CDR"):
        return {"version":1,"run_id":"r","sequence":seq,"topic":topic,"type":"geometry_msgs/msg/Twist",
                "acquired_clock_ns":stamp,"payload_b64":base64.b64encode(raw).decode(),"payload_sha256":hashlib.sha256(raw).hexdigest()}

    def test_partial_and_multiple_packets_preserve_payload(self):
        data=Framer.encode(self.packet())+Framer.encode(self.packet(2))
        f=Framer();result=[]
        for i in range(0,len(data),7):result.extend(f.feed(data[i:i+7]))
        self.assertEqual(len(result),2)
        receiver=Receiver("r","windows",catalog())
        self.assertEqual(receiver.take(result[0],local_clock_ns=20)[1],b"unchanged CDR")
        self.assertEqual(receiver.take(result[1],local_clock_ns=20)[1],b"unchanged CDR")

    def test_wrong_identity_direction_hash_and_replay_fail(self):
        for change in ({"run_id":"other"},{"topic":"ros_discovery_info"},{"payload_sha256":"0"*64},{"sequence":2},{"type":"std_msgs/msg/String"}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                Receiver("r","windows",catalog()).take({**self.packet(),**change},local_clock_ns=20)
        with self.assertRaises(ValueError):Receiver("r","linux",catalog()).take(self.packet(),local_clock_ns=20)
        with self.assertRaises(ValueError):Framer().feed(struct.pack("!I",2_000_000))

    def test_old_or_future_command_is_dropped_without_restamping(self):
        r=Receiver("r","windows",catalog())
        self.assertIsNone(r.take(self.packet(stamp=0),local_clock_ns=300_000_001)[1])
        self.assertIsNone(r.take(self.packet(seq=2,stamp=100),local_clock_ns=90)[1])
        self.assertEqual(r.take(self.packet(seq=3,stamp=100),local_clock_ns=110)[1],b"unchanged CDR")

    def test_run_profile_keeps_dds_on_each_os_loopback(self):
        from scripts.isaac.run_local45_fleet_pair import prepare_ros_transport
        with tempfile.TemporaryDirectory() as directory:
            path,env=prepare_ros_transport(Path(directory),73,interface="127.0.0.1",isolated_loopback=True)
            import xml.etree.ElementTree as ET
            root=ET.parse(path).getroot();ns={"d":"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
            self.assertEqual([x.text for x in root.findall(".//d:address",ns)],["127.0.0.1","127.0.0.1"])
            self.assertEqual(root.find(".//d:maxInitialPeersRange",ns).text,"120")
            self.assertEqual(env["ROS_DOMAIN_ID"],"73")
            with self.assertRaises(ValueError):
                prepare_ros_transport(Path(directory),73,interface="192.168.96.1",isolated_loopback=True)


if __name__=="__main__":unittest.main()
