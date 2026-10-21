import base64
import hashlib
import struct
import unittest
from pathlib import Path
import tempfile
from factory_ops.ros_topic_relay import Framer,Receiver,catalog


class RelayTests(unittest.TestCase):
    def test_connecting_heartbeat_stays_pending_until_authenticated_ready(self):
        from factory_ops.ros_topic_relay import write_heartbeat, read_heartbeat
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"relay.json"
            report={"run_id":"a"*32,"side":"linux","status":"connecting","sent":{},"received":{},"stale_commands_dropped":0}
            self.assertFalse(write_heartbeat(path,report))
            self.assertIsNone(read_heartbeat(path))
            self.assertNotIn("heartbeat_sequence",report)
            report['status']='ready'
            self.assertTrue(write_heartbeat(path,report))
            self.assertEqual(read_heartbeat(path)['sequence'],1)

    def test_pair_observes_heartbeat_and_rejects_foreign_run(self):
        from factory_ops.ros_topic_relay import write_checkpoint, write_heartbeat
        from factory_ops.ros_topic_relay_pair import RelayPair
        from unittest.mock import patch
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"windows-relay.json"
            report={"run_id":"a"*32,"side":"windows","status":"ready","sent":{},"received":{},"stale_commands_dropped":0}
            write_checkpoint(path,report);write_heartbeat(path,report)
            pair=RelayPair(Path(folder),Path(folder),"a"*32,60,{}, {},"192.168.0.1","192.168.0.2",28473)
            with patch.object(pair.owner,"client_state",return_value={"started":False}):
                pair.assert_healthy()
                self.assertEqual(pair.report["last_heartbeats"]["windows"]["sequence"],1)
                write_heartbeat(path,dict(report,run_id="b"*32))
                with self.assertRaises(RuntimeError):pair.assert_healthy()

    def test_periodic_heartbeat_appends_while_checkpoint_and_journal_readers_are_held(self):
        from factory_ops.ros_topic_relay import write_checkpoint, write_heartbeat, read_heartbeat
        import json,os
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"relay.json"
            report={"run_id":"a"*32,"side":"windows","status":"ready","sent":{},"received":{},
                    "stale_commands_dropped":0,"last_source_clock_ns":0}
            write_checkpoint(path,report)
            write_heartbeat(path,report)
            with path.open("rb") as checkpoint_reader, path.with_suffix(".heartbeat.jsonl").open("rb"):
                if os.name=="nt":
                    probe=Path(folder)/"probe.tmp";probe.write_text("replacement")
                    with self.assertRaises(PermissionError):os.replace(probe,path)
                for stamp in range(1,21):
                    report.update(last_source_clock_ns=stamp, sent={"/clock":stamp})
                    write_heartbeat(path,report)
                    self.assertEqual(read_heartbeat(path)["last_source_clock_ns"],stamp)
                self.assertEqual(json.loads(checkpoint_reader.read())["last_source_clock_ns"],0)
            self.assertEqual(read_heartbeat(path)["sequence"],21)
            self.assertEqual(read_heartbeat(path)["sent_total"],20)

    def test_heartbeat_reader_keeps_inflight_partial_row_pending_and_rejects_complete_corruption(self):
        from factory_ops.ros_topic_relay import read_heartbeat, write_heartbeat
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/"relay.json";journal=path.with_suffix(".heartbeat.jsonl")
            self.assertIsNone(read_heartbeat(path))
            report={"run_id":"a"*32,"side":"windows","status":"ready","sent":{},"received":{},"stale_commands_dropped":0}
            write_heartbeat(path,report)
            with journal.open("ab") as stream:stream.write(b'{"sequence":')
            self.assertEqual(read_heartbeat(path)["sequence"],1)
            with journal.open("ab") as stream:stream.write(b'bad}\n')
            with self.assertRaises(ValueError):read_heartbeat(path)

    def test_heartbeat_append_permanent_error_is_not_retried_or_marked_published(self):
        from factory_ops.ros_topic_relay import write_heartbeat
        from unittest.mock import patch
        import errno
        report={"run_id":"a"*32,"side":"windows","status":"ready","sent":{},"received":{},"stale_commands_dropped":0}
        with patch.object(Path,"open",side_effect=OSError(errno.ENOSPC,"full storage")) as opened:
            with self.assertRaises(OSError):write_heartbeat(Path("relay.json"),report)
        self.assertEqual(opened.call_count,1)
        self.assertNotIn("heartbeat_sequence",report)

    def test_service_writer_profiles_cover_exact_linux_lifecycle_requests_and_replies(self):
        from scripts.isaac.run_local45_fleet_pair import prepare_ros_transport
        import xml.etree.ElementTree as ET
        with tempfile.TemporaryDirectory() as directory:
            path,environment=prepare_ros_transport(Path(directory),73,platform="linux",interface="127.0.0.1",
                isolated_loopback=True,startup_service_match_wait=True)
            ns={"d":"http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
            publishers=ET.parse(path).getroot().findall("d:publisher",ns)
            expected={f"{prefix}/{robot}/{component}/{service}{suffix}"
                for prefix,suffix in (("rq","Request"),("rr","Reply"))
                for robot in ("carter1","carter2","carter3","carter4")
                for component in ("map_server","navigation_map_server","amcl","planner_server",
                    "controller_server","smoother_server","behavior_server","bt_navigator")
                for service in ("change_state","get_state")}
            self.assertEqual(len(publishers),128)
            self.assertEqual({row.get("profile_name") for row in publishers},expected)
            self.assertNotIn("RMW_FASTRTPS_USE_QOS_FROM_XML",environment)
            for row in publishers:
                self.assertEqual(row.find(".//d:max_blocking_time/d:sec",ns).text,"1")
                self.assertEqual(row.find(".//d:max_blocking_time/d:nanosec",ns).text,"0")
                self.assertEqual(row.find(".//d:reliability/d:kind",ns).text,"RELIABLE")
            with self.assertRaises(ValueError):
                prepare_ros_transport(Path(directory),73,platform="windows",interface="127.0.0.1",
                    isolated_loopback=True,startup_service_match_wait=True)
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
