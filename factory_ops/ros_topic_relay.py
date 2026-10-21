"""Run-owned, allowlisted ROS topic transport across isolated ROS networks.

ROS discovery, services and actions stay inside each OS. Only explicitly
declared application messages cross the socket. This transport never creates
motion commands, changes a message field, offsets a clock, or reads GT.
"""
from __future__ import annotations

import argparse
import base64
from collections import deque
import hashlib
import importlib
import json
import os
from pathlib import Path
import re
import select
import socket
import struct
import sys
import time
import traceback

from factory_ops.compat import replace_shared_file, write_shared_text

MAX_PACKET_BYTES = 1_048_576
MAX_QUEUED_BYTES = 4_194_304
COMMAND_AGE_NS = 300_000_000
MAX_HEARTBEAT_BYTES = 4096


def write_checkpoint(path, report):
    temporary=path.with_suffix(".tmp")
    write_shared_text(temporary,json.dumps(report,indent=2,allow_nan=False)+"\n")
    replace_shared_file(temporary,path)


def write_heartbeat(path, report):
    """Append one small observation; never retry a possibly partial append."""
    if report["status"] in {"starting", "connecting"}:
        return False
    if report["status"]!="ready":
        raise ValueError("periodic relay heartbeat requires an authenticated ready connection")
    row = {"run_id": report["run_id"], "side": report["side"], "status": report["status"],
           "sequence": report.get("heartbeat_sequence", 0)+1,
           "last_source_clock_ns": report.get("last_source_clock_ns", 0),
           "queued_bytes": report.get("queued_bytes", 0),
           "sent_total": sum(report["sent"].values()),
           "received_total": sum(report["received"].values()),
           "stale_commands_dropped": report["stale_commands_dropped"]}
    raw = (json.dumps(row, separators=(",", ":"), allow_nan=False)+"\n").encode("utf-8")
    if len(raw)>MAX_HEARTBEAT_BYTES:
        raise ValueError("relay heartbeat outside size bound")
    with path.with_suffix(".heartbeat.jsonl").open("ab", buffering=0) as stream:
        if stream.write(raw)!=len(raw):
            raise OSError("incomplete relay heartbeat append")
    report["heartbeat_sequence"] = row["sequence"]
    return True


def read_heartbeat(path):
    """Read the last complete bounded row; leave an in-flight final row pending."""
    try:
        with path.with_suffix(".heartbeat.jsonl").open("rb") as stream:
            size = stream.seek(0, 2)
            stream.seek(max(0, size-2*MAX_HEARTBEAT_BYTES))
            raw = stream.read(2*MAX_HEARTBEAT_BYTES)
    except FileNotFoundError:
        return None
    rows = raw.split(b"\n")[:-1]
    if size>2*MAX_HEARTBEAT_BYTES:
        rows = rows[1:]
    if not rows:
        return None
    if not 1<=len(rows[-1])<=MAX_HEARTBEAT_BYTES:
        raise ValueError("relay heartbeat row outside size bound")
    row = json.loads(rows[-1])
    if not isinstance(row, dict) or type(row.get("sequence")) is not int or row["sequence"]<1:
        raise ValueError("invalid relay heartbeat identity/sequence")
    return row


def catalog(robot_ids=("carter1","carter2","carter3","carter4"), *, probe_prefix=None):
    rows={"/clock": ("rosgraph_msgs/msg/Clock","windows","reliable",False)}
    for robot in robot_ids:
        prefix="/"+robot if probe_prefix is None else probe_prefix+"/"+robot
        for suffix,typ,qos,latched in (
            ("odom","nav_msgs/msg/Odometry","reliable",False),
            ("tf","tf2_msgs/msg/TFMessage","reliable",False),
            ("tf_static","tf2_msgs/msg/TFMessage","reliable",True),
            ("scan_front_raw","sensor_msgs/msg/LaserScan","best_effort",False),
            ("scan_rear_raw","sensor_msgs/msg/LaserScan","best_effort",False),
        ):
            rows[prefix+"/"+suffix]=(typ,"windows",qos,latched)
        rows[prefix+"/cmd_vel"]=("geometry_msgs/msg/Twist","linux","reliable",False)
    return rows


class Framer:
    def __init__(self): self.buffer=bytearray()

    @staticmethod
    def encode(document):
        raw=json.dumps(document,separators=(",", ":"),allow_nan=False).encode("utf-8")
        if not 1 <= len(raw) <= MAX_PACKET_BYTES: raise ValueError("relay packet size outside bound")
        return struct.pack("!I",len(raw))+raw

    def feed(self,data):
        self.buffer.extend(data)
        if len(self.buffer)>2*MAX_PACKET_BYTES+8: raise ValueError("relay input buffer outside bound")
        documents=[]
        while len(self.buffer)>=4:
            size=struct.unpack("!I",self.buffer[:4])[0]
            if not 1<=size<=MAX_PACKET_BYTES: raise ValueError("relay declared packet size outside bound")
            if len(self.buffer)<size+4:break
            raw=bytes(self.buffer[4:size+4]);del self.buffer[:size+4]
            document=json.loads(raw)
            if not isinstance(document,dict):raise ValueError("relay packet must be an object")
            documents.append(document)
        return documents


class Receiver:
    def __init__(self, run_id, side, topics):
        self.run_id,self.side,self.topics=run_id,side,topics
        self.sequence=0

    def take(self,packet, *, local_clock_ns):
        required={"version","run_id","sequence","topic","type","acquired_clock_ns","payload_b64","payload_sha256"}
        if set(packet)!=required or packet["version"]!=1 or packet["run_id"]!=self.run_id:
            raise ValueError("relay packet contract/run identity differs")
        seq=packet["sequence"]
        if type(seq) is not int or seq!=self.sequence+1:raise ValueError("relay sequence is not contiguous")
        self.sequence=seq
        topic=packet["topic"]
        if topic not in self.topics:raise ValueError("relay topic is not allowlisted")
        typ,source,_,_=self.topics[topic]
        if source==self.side or packet["type"]!=typ:raise ValueError("relay topic direction/type differs")
        stamp=packet["acquired_clock_ns"]
        if type(stamp) is not int or stamp<0:raise ValueError("invalid relay acquisition clock")
        raw=base64.b64decode(packet["payload_b64"],validate=True)
        if not raw or hashlib.sha256(raw).hexdigest()!=packet["payload_sha256"]:
            raise ValueError("relay serialized payload integrity differs")
        if source=="linux" and not 0<=local_clock_ns-stamp<=COMMAND_AGE_NS:
            return topic,None,"stale_or_future_command"
        return topic,raw,"observed"


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side",choices=("windows","linux"),required=True)
    parser.add_argument("--run-id",required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--stop-request",type=Path,required=True)
    parser.add_argument("--host",required=True)
    parser.add_argument("--port",type=int,required=True)
    parser.add_argument("--expected-peer")
    parser.add_argument("--sdk",type=Path)
    parser.add_argument("--wall-bound-s",type=float,default=10800.)
    parser.add_argument("--probe-prefix")
    args=parser.parse_args()
    if not re.fullmatch("[0-9a-f]{32}",args.run_id) or not 1024<=args.port<=65535 or not 1<=args.wall_bound_s<=10830:
        parser.error("run identity, port or wall bound outside contract")
    if args.output.exists() or args.output.is_symlink() or not args.output.is_absolute():
        parser.error("new absolute relay output required")
    if args.probe_prefix is not None and args.probe_prefix!="/probe_"+args.run_id:
        parser.error("probe topics must match run identity")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    started=time.monotonic();deadline=started+args.wall_bound_s
    report={"run_id":args.run_id,"side":args.side,"status":"starting","source_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "scope":"allowlisted ROS application relay, each OS DDS graph isolated; no GT/control synthesis or clock offset",
        "sent":{},"received":{},"stale_commands_dropped":0,"last_source_clock_ns":0,
        "cleanup_errors":[],"closed":False,"command_maximum_acquisition_age_ns":COMMAND_AGE_NS,
        "node_acquisition_clock_is_producer_generation_time":False,
        "periodic_observation":"bounded-row append heartbeat; full checkpoint at lifecycle transitions and closure"}
    def save():
        write_checkpoint(args.output,report)
    def stopping():
        if args.stop_request.is_file() and not args.stop_request.is_symlink():
            if args.stop_request.read_text(encoding="ascii")!=args.run_id:raise ValueError("relay stop request identity differs")
            return True
        if time.monotonic()>deadline:raise TimeoutError("relay wall bound exceeded")
        return False
    listener=connection=node=executor=None;rclpy=None;initialized=False;code=1
    save()
    try:
        if args.side=="windows":
            if args.sdk is None:raise ValueError("Windows relay requires existing SDK path")
            os.environ["ISAAC_PATH"]=str(args.sdk)
            from factory_ops.isaac45_runtime import configure_ros_environment
            configure_ros_environment(int(os.environ["ROS_DOMAIN_ID"]))
            bundles=list((args.sdk/"extscache").glob("omni.kit.pip_archive-*/pip_prebundle"))
            if len(bundles)!=1:raise ValueError("SDK Python bundle ambiguous")
            sys.path.insert(0,str(bundles[0]));sys.path.insert(0,str(args.sdk/"exts/isaacsim.ros2.bridge/humble/rclpy"))
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.serialization import serialize_message,deserialize_message
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        rclpy.init();initialized=True
        node=rclpy.create_node("factory_topic_relay_"+args.side)
        executor=SingleThreadedExecutor();executor.add_node(node)
        topics=catalog(probe_prefix=args.probe_prefix)
        types={typ:getattr(importlib.import_module(typ.split("/")[0]+".msg"),typ.split("/")[2]) for typ,_,_,_ in topics.values()}
        queue=deque();queued_bytes=0;sequence=0;clock_ns=0
        publishers={}
        def enqueue(document):
            nonlocal queued_bytes
            data=Framer.encode(document);queued_bytes+=len(data)
            if queued_bytes>MAX_QUEUED_BYTES:raise ValueError("relay output queue outside bound")
            queue.append(memoryview(data))
        for topic,(typ,source,reliability,latched) in topics.items():
            qos=QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE if reliability=="reliable" else ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)
            if source!=args.side:
                publishers[topic]=node.create_publisher(types[typ],topic,qos)
            else:
                def send(message,topic=topic,typ=typ):
                    nonlocal sequence,clock_ns
                    if topic=="/clock":
                        observed=int(message.clock.sec)*1_000_000_000+int(message.clock.nanosec)
                        if observed<clock_ns:raise ValueError("source World clock regressed")
                        clock_ns=observed
                    sequence+=1;raw=bytes(serialize_message(message))
                    enqueue({"version":1,"run_id":args.run_id,"sequence":sequence,"topic":topic,"type":typ,
                        "acquired_clock_ns":clock_ns,"payload_b64":base64.b64encode(raw).decode("ascii"),
                        "payload_sha256":hashlib.sha256(raw).hexdigest()})
                    report["sent"][topic]=report["sent"].get(topic,0)+1
                node.create_subscription(types[typ],topic,send,qos)
        report.update(status="connecting",domain_id=node.context.get_domain_id(),rclpy_file=rclpy.__file__,
            topics={name:{"type":row[0],"source":row[1]} for name,row in topics.items()})
        save()
        if args.side=="windows":
            listener=socket.socket(socket.AF_INET,socket.SOCK_STREAM);listener.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            listener.bind((args.host,args.port));listener.listen(1);listener.settimeout(.2)
            while connection is None:
                if stopping():report["status"]="stopped_before_peer";code=0;return code
                try:
                    connection,peer=listener.accept()
                    if args.expected_peer is not None and peer[0]!=args.expected_peer:raise ValueError("relay TCP peer address differs")
                    report["tcp_peer"]=list(peer)
                except socket.timeout:pass
        else:
            while connection is None:
                if stopping():report["status"]="stopped_before_peer";code=0;return code
                try:connection=socket.create_connection((args.host,args.port),timeout=.5)
                except (ConnectionRefusedError,socket.timeout,OSError):time.sleep(.1)
        connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);connection.setblocking(False)
        enqueue({"hello":1,"run_id":args.run_id,"side":args.side})
        framer=Framer();receiver=Receiver(args.run_id,args.side,topics);authenticated=False;next_save=0.
        while not stopping():
            reads,writes,_=select.select([connection],[connection] if queue else [],[],.002)
            if reads:
                data=connection.recv(65536)
                if not data:
                    if stopping():
                        report["status"]="stopped";code=0;break
                    raise ConnectionError("paired relay socket closed")
                for packet in framer.feed(data):
                    if not authenticated:
                        if packet!={"hello":1,"run_id":args.run_id,"side":"linux" if args.side=="windows" else "windows"}:
                            raise ValueError("relay peer handshake differs")
                        authenticated=True;report["status"]="ready";save();continue
                    topic,raw,status=receiver.take(packet,local_clock_ns=clock_ns)
                    if raw is None:report["stale_commands_dropped"]+=1;continue
                    message=deserialize_message(raw,types[topics[topic][0]])
                    if topic=="/clock":
                        observed=int(message.clock.sec)*1_000_000_000+int(message.clock.nanosec)
                        if observed<clock_ns:raise ValueError("received World clock regressed")
                        clock_ns=observed
                    publishers[topic].publish(message)
                    report["received"][topic]=report["received"].get(topic,0)+1
            if authenticated:
                for _ in range(8):executor.spin_once(timeout_sec=0.)
            if writes and queue:
                count=connection.send(queue[0]);queued_bytes-=count
                queue[0]=queue[0][count:]
                if not queue[0]:queue.popleft()
            if time.monotonic()>=next_save:
                report.update(last_source_clock_ns=clock_ns,queued_bytes=queued_bytes,received_sequence=receiver.sequence)
                write_heartbeat(args.output,report);next_save=time.monotonic()+1.
        report["status"]="stopped";code=0
    except BaseException as error:
        report.update(status="failed",error={"type":type(error).__name__,"message":str(error),"traceback":traceback.format_exc(limit=8)})
    finally:
        for resource in (connection,listener):
            if resource is not None:
                try:resource.close()
                except Exception as error:report["cleanup_errors"].append(str(error))
        if executor is not None:
            try:executor.shutdown()
            except Exception as error:report["cleanup_errors"].append(str(error))
        if node is not None:
            try:node.destroy_node()
            except Exception as error:report["cleanup_errors"].append(str(error))
        if initialized:
            try:rclpy.shutdown()
            except Exception as error:report["cleanup_errors"].append(str(error))
        report.update(closed=not report["cleanup_errors"],wall_s=time.monotonic()-started)
        save()
    return code


if __name__=="__main__":raise SystemExit(main())
