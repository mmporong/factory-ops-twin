"""Run an already prepared SDK4.5/WSL development pair with separate tree owners."""
from __future__ import annotations

import argparse
import hashlib
from ipaddress import IPv4Address
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import tarfile
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.wsl_process_supervisor import WslProcessSupervisor
from factory_ops.compat import replace_shared_file, write_shared_text

PAIR_CLEANUP_RESERVE_S = 30


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    write_shared_text(temporary,json.dumps(value, indent=2, allow_nan=False) + "\n")
    replace_shared_file(temporary,path)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def linux(path: Path) -> str:
    if not path.is_absolute() or not path.drive or any(c in str(path) for c in '\r\n%&|<>^!"'):
        raise ValueError("runtime paths must be absolute drive paths without shell metacharacters")
    return "/mnt/" + path.drive[0].lower() + path.as_posix()[2:]


def _validate_dds_interfaces(windows: str | None, linux_interface: str | None) -> None:
    if (windows is None) != (linux_interface is None):
        raise ValueError("Windows and Linux DDS interfaces must be supplied together")
    for address in (windows, linux_interface):
        if address is not None:
            if not isinstance(address, str):
                raise ValueError("DDS interface must be a numeric IPv4 address string")
            IPv4Address(address)


def prepare_ros_transport(stage: Path, domain_id: int, *, interface: str | None = None,
                          platform: str = "windows", peers: tuple[str, ...] = (),
                          isolated_loopback: bool = False, startup_service_match_wait: bool = False) -> tuple[Path, dict[str, str]]:
    """Create the run-owned UDP participant profile before ROS processes start."""
    if isinstance(domain_id, bool) or not isinstance(domain_id, int) or not 0 <= domain_id <= 232:
        raise ValueError("bundle ROS domain must be an integer in0..232")
    if interface is not None:
        if platform not in {"windows", "linux"} or not isinstance(interface, str):
            raise ValueError("DDS whitelist requires a platform and numeric IPv4 address string")
        interface = str(IPv4Address(interface))
    if isolated_loopback and (interface != "127.0.0.1" or peers):
        raise ValueError("isolated DDS requires only the local loopback interface")
    if startup_service_match_wait and (platform!="linux" or not isolated_loopback):
        raise ValueError("startup service match profile requires isolated Linux DDS")
    if peers:
        if type(peers) is not tuple or len(peers) != 2 or len(set(peers)) != 2 or interface not in peers:
            raise ValueError("explicit DDS discovery requires the two paired IPv4 interfaces")
        for peer in peers:
            if not isinstance(peer, str):
                raise ValueError("DDS peer must be a numeric IPv4 address")
            address = IPv4Address(peer)
            if address.is_multicast or address.is_unspecified or address.is_loopback:
                raise ValueError("DDS peer must be an identified unicast interface")
    profile = stage / ("fastdds-udp.xml" if interface is None else f"fastdds-udp-{platform}.xml")
    text = '''<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor><transport_id>FactoryUdp</transport_id><type>UDPv4</type></transport_descriptor>
  </transport_descriptors>
  <participant profile_name="factory_udp" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>FactoryUdp</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
'''
    if interface is not None:
        text = text.replace("<type>UDPv4</type>", "<type>UDPv4</type>"
            f"<interfaceWhiteList><address>{interface}</address></interfaceWhiteList>")
    if peers:
        # Fast DDS2.6 permits unspecified peer ports; cover the authored local
        # multi-participant topology without depending only on NAT multicast.
        text = text.replace("<type>UDPv4</type>", "<type>UDPv4</type><maxInitialPeersRange>120</maxInitialPeersRange>")
        locators = "".join(f"<locator><udpv4><address>{peer}</address></udpv4></locator>" for peer in peers)
        text = text.replace("<rtps>", f"<rtps><builtin><initialPeersList>{locators}</initialPeersList></builtin>")
    if isolated_loopback:
        text = text.replace("<type>UDPv4</type>", "<type>UDPv4</type><maxInitialPeersRange>120</maxInitialPeersRange>")
        text = text.replace("<rtps>", "<rtps><builtin><initialPeersList><locator><udpv4>"
            "<address>127.0.0.1</address></udpv4></locator></initialPeersList></builtin>")
    if startup_service_match_wait:
        # RMW8.4.4 selects exact DDS topic profiles independently for request and
        # reply writers. This sets writer QoS; it does not call wait_for_service.
        # Keep the existing one-second writer budget on both lifecycle directions.
        components=("map_server","navigation_map_server","amcl","planner_server","controller_server",
                    "smoother_server","behavior_server","bt_navigator")
        profiles="".join('<publisher profile_name="'+prefix+'/'+robot+'/'+component+'/'+service+suffix+'">'
            '<qos><reliability><kind>RELIABLE</kind><max_blocking_time><sec>1</sec><nanosec>0</nanosec>'
            '</max_blocking_time></reliability></qos></publisher>'
            for robot in ("carter1","carter2","carter3","carter4") for component in components
            for service in ("change_state","get_state")
            for prefix,suffix in (("rq","Request"),("rr","Reply")))
        text=text.replace('</profiles>',profiles+'\n</profiles>')
    with profile.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
    return profile, {"ROS_DOMAIN_ID": str(domain_id), "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS_LOCALHOST_ONLY": "0", "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile),
        "FASTDDS_DEFAULT_PROFILES_FILE": str(profile)}


def close_pair(worker, streams, owner, stage: Path, report: dict, started: float, *,
               linux_start_attempted: bool = True, windows_start_attempted: bool = True) -> None:
    errors = []
    if worker is not None:
        try:
            if worker.poll() is None:
                abort = stage / "job-abort-request.txt"
                if not abort.exists():
                    with abort.open("x", encoding="ascii") as stream:
                        stream.write(report["run_id"])
                elif abort.read_text(encoding="ascii") != report["run_id"]:
                    raise RuntimeError("abort request identity differs")
        except Exception as error:
            errors.append("windows abort request: " + str(error))
        try:
            worker.wait(timeout=10)
        except Exception as error:
            errors.append("windows wait: " + str(error))
            try:
                worker.terminate()
            except Exception as terminate_error:
                errors.append("windows terminate: " + str(terminate_error))
    for stream in streams:
        try:
            stream.close()
        except Exception as error:
            errors.append("log close: " + str(error))
    if linux_start_attempted:
        try:
            report["linux_end"] = owner.stop()
            report["linux_closed"] = report["linux_end"]["closure_verified"]
        except Exception as error:
            errors.append("linux stop: " + str(error))
    else:
        report["linux_end"] = {"not_started": True, "closure_verified": True}
        report["linux_closed"] = True
    try:
        job_path = stage / "job.json"
        if not windows_start_attempted and worker is None and not job_path.exists():
            report["windows_end"] = {"not_started": True, "closed": True, "remaining_owned_pids": []}
            report["windows_closed"] = True
            job = None
        else:
            job = json.loads(job_path.read_text(encoding="utf-8"))
        if job is not None:
            report["windows_end"] = job
            report["windows_closed"] = (job["run_id"] == report["run_id"] and job["closed"]
                                         and not job["remaining_owned_pids"])
            if job.get("status") != "passed":
                report["status"] = "failed"
    except Exception as error:
        errors.append("windows receipt: " + str(error))
    report["wall_s"] = time.monotonic() - started
    report["cleanup_errors"] = errors
    if errors or not report["linux_closed"] or not report["windows_closed"]:
        report["status"] = "failed"
    save(stage / "pair-host.json", report)


def wait_for_bootstrap_marker(marker: Path, owner: WslProcessSupervisor, *, started: float, bound: int) -> None:
    deadline = time.monotonic() + 25
    while True:
        now = time.monotonic()
        if now - started >= bound - PAIR_CLEANUP_RESERVE_S:
            raise TimeoutError("pair owner deadline exceeded; cleanup time reserved inside the wall budget")
        client = owner.client_state()
        if now > deadline or not client["started"] or not client["running"]:
            raise RuntimeError("ROS mission did not create its verified bootstrap marker with a living foreground client")
        if marker.exists():
            return
        time.sleep(.2)


def monitor_pair(worker, owner: WslProcessSupervisor, *, started: float, bound: int,
                 failure_path: Path | None = None, run_id: str | None = None, relay=None) -> dict | None:
    """Observe client exit, retaining SDK cleanup grace and the pair deadline."""
    exit_seen = None
    exit_record = None
    while worker.poll() is None:
        now = time.monotonic()
        if now - started >= bound - PAIR_CLEANUP_RESERVE_S:
            raise TimeoutError("pair owner deadline exceeded; cleanup time reserved inside the wall budget")
        if relay is not None:relay.assert_healthy()
        if failure_path is not None and failure_path.exists():
            failure = json.loads(failure_path.read_text(encoding="utf-8"))
            if failure.get("run_id") != run_id or failure.get("status") != "failed":
                raise RuntimeError("SDK failure checkpoint identity/status mismatch")
            error = failure["error"]
            raise RuntimeError(f"SDK failure before cleanup: {failure['stage']}: "
                               f"{error['type']}: {error['message']}")
        client = owner.client_state()
        if not client["started"]:
            raise RuntimeError("admitted foreground WSL client is missing")
        if not client["running"]:
            if exit_seen is None:
                exit_seen = now
                exit_record = {"observed_elapsed_s": now - started, "client": client,
                               "scope": "foreground client exit only; Linux closure not inferred"}
            if now - exit_seen > 90:
                raise RuntimeError("SDK did not finish cleanup within90s after ROS foreground client exit")
        time.sleep(2)
    return exit_record


def run(stage: Path, sdk: Path, bound: int, *, charge_robot: str | None = None, charge_resume_soc: float = .8,
        lidar_backend: str = "rtx_flat_scan", physics_cadence: str = "owned_step6",
        windows_dds_interface: str | None = None, linux_dds_interface: str | None = None,
        live_anchor_controls: bool = False, dds_explicit_peers: bool = False,
        nav2_startup_stagger_s: float = 0., capture_resolution: str = "640x360",
        capture_views: str = "all",
        carrier_modules: bool = False,
        isolated_topic_relay: bool = False, relay_windows_address: str | None = None,
        relay_linux_address: str | None = None, relay_port: int = 28473,
        startup_service_match_wait: bool = False) -> int:
    if os.name != "nt" or not 60 <= bound <= 10800:
        raise ValueError("local development requires Windows and an explicit 60..10800s bound")
    if lidar_backend not in {"rtx_flat_scan", "physx_planar"}:
        raise ValueError("unsupported local45 lidar backend")
    if physics_cadence not in {"app_update", "owned_step6"}:
        raise ValueError("unsupported local45 physics cadence")
    if type(nav2_startup_stagger_s) not in (float,int) or not 0 <= nav2_startup_stagger_s <= 5:
        raise ValueError("Nav2 startup stagger must be finite seconds in [0,5]")
    if capture_resolution not in {"640x360", "1280x720"}:
        raise ValueError("unsupported local45 capture resolution")
    if capture_views not in {"all", "overview"}:
        raise ValueError("unsupported local45 capture view selection")
    if type(carrier_modules) is not bool or (carrier_modules and lidar_backend != "physx_planar"):
        raise ValueError("carrier modules require explicit local45 planar development")
    _validate_dds_interfaces(windows_dds_interface, linux_dds_interface)
    if startup_service_match_wait and not isolated_topic_relay:
        raise ValueError("startup service match option requires OS-isolated topic transport")
    if isolated_topic_relay:
        if dds_explicit_peers or windows_dds_interface is not None:
            raise ValueError("isolated topic relay cannot share external DDS interfaces/peers")
        for address in (relay_windows_address,relay_linux_address):
            if not isinstance(address,str) or not IPv4Address(address).is_private or IPv4Address(address).is_loopback:
                raise ValueError("relay requires each OS's identified private TCP interface")
        if relay_windows_address==relay_linux_address or type(relay_port) is not int or not 1024<=relay_port<=65535:
            raise ValueError("relay endpoints/port differ from contract")
        windows_dds_interface=linux_dds_interface="127.0.0.1"
    if dds_explicit_peers and windows_dds_interface is None:
        raise ValueError("explicit DDS peers require both interface addresses")
    stage, sdk = stage.resolve(strict=True), sdk.resolve(strict=True)
    if charge_robot is not None and charge_robot not in {"carter1", "carter2", "carter3", "carter4"}:
        raise ValueError("charge robot must belong to the four-AMR development fleet")
    if not 0 < charge_resume_soc <= 1:
        raise ValueError("charge target must be a0..1 fraction")
    launch = json.loads((stage / "launch.json").read_text(encoding="utf-8"))
    run_id = launch["run_id"]
    output, source = stage / "results/run", stage / "source"
    metadata = output / "provenance/fleet-source.json"
    if (sha(metadata) != launch["provenance_sha256"] or sha(stage / "source.tar") != launch["source_archive_sha256"]
            or sha(stage / "build-attestation.json") != launch["build_attestation_sha256"]):
        raise ValueError("prepared launch provenance changed")
    if live_anchor_controls:
        # Additional opt-in source must be part of the already proved immutable archive.
        with tarfile.open(stage / "source.tar") as archive:
            member = archive.extractfile("factory_ops/live_control.py")
            if member is None or hashlib.sha256(member.read()).hexdigest() != sha(source / "factory_ops/live_control.py"):
                raise ValueError("live control source differs from the frozen source archive")
    if carrier_modules:
        carrier_pins = ("scripts/isaac/carrier_native.py", "factory_ops/carrier_module.py",
                        "factory_ops/carrier_robot.py", "configs/vehicles/tote-carrier-carter-v2.json")
        pinned = json.loads(metadata.read_text(encoding="utf-8"))["source_files"]
        with tarfile.open(stage/"source.tar") as archive:
            for name in carrier_pins:
                member = archive.extractfile(name)
                if (member is None or name not in pinned or pinned[name] != sha(source/name)
                        or hashlib.sha256(member.read()).hexdigest() != pinned[name]):
                    raise ValueError("carrier source/profile differs from the frozen pinned archive")
    receipt_path = stage / "pair-host.json"
    if receipt_path.exists() or (stage / "nav.log").exists() or (stage / "job-spec.json").exists():
        raise ValueError("preserve prior attempt; prepared stage must be unused")
    report = {"schema_version": 1, "run_id": run_id, "source_commit": launch["source_commit"],
        "scope": "bounded local45 development integration admission; no qualification or completion claim",
        "launcher_sha256": sha(Path(__file__)), "wsl_supervisor_sha256": sha(Path(__file__).resolve().parents[2] / "factory_ops/wsl_process_supervisor.py"),
        "status": "starting", "wall_bound_s": bound,
        "work_wall_bound_s": bound - PAIR_CLEANUP_RESERVE_S, "cleanup_reserve_s": PAIR_CLEANUP_RESERVE_S,
        "windows_closed": False, "linux_closed": False,
        "lidar_backend": lidar_backend, "requested_physics_cadence": physics_cadence}
    report["live_anchor_controls"] = live_anchor_controls
    report["dds_explicit_peers"] = dds_explicit_peers
    report["nav2_startup_stagger_s"] = nav2_startup_stagger_s
    report["capture_resolution"] = capture_resolution
    report["capture_views"] = capture_views
    report["carrier_modules"] = carrier_modules
    report["isolated_topic_relay"] = isolated_topic_relay
    report["startup_service_match_wait"] = startup_service_match_wait
    report["virtual_charging_request"] = {"robot_id": charge_robot, "resume_soc": charge_resume_soc}
    save(receipt_path, report)
    owner = WslProcessSupervisor(run_id)
    worker = None
    relay = None
    streams = []
    linux_start_attempted = windows_start_attempted = False
    started = time.monotonic()
    try:
        root_linux = linux(stage)
        attestation = json.loads((stage / "build-attestation.json").read_text(encoding="utf-8"))
        installed = linux(Path(attestation["actual_install_directory"]))
        config = json.loads((stage / "inputs/config.json").read_text(encoding="utf-8"))
        peers = (windows_dds_interface, linux_dds_interface) if dds_explicit_peers else ()
        transport, ros_environment = prepare_ros_transport(stage, config["ros_domain_id"],
            interface=windows_dds_interface, platform="windows", peers=peers,isolated_loopback=isolated_topic_relay)
        linux_transport = transport
        if linux_dds_interface is not None:
            linux_transport, _ = prepare_ros_transport(stage, config["ros_domain_id"],
                interface=linux_dds_interface, platform="linux", peers=peers,isolated_loopback=isolated_topic_relay,
                startup_service_match_wait=startup_service_match_wait)
        linux_environment = {**ros_environment, "FASTRTPS_DEFAULT_PROFILES_FILE": linux(linux_transport),
                             "FASTDDS_DEFAULT_PROFILES_FILE": linux(linux_transport)}
        report["ros_transport"] = {"profile": str(transport), "sha256": sha(transport),
            "windows_environment": ros_environment, "linux_environment": linux_environment,
            "scope": "configured run-local UDP profile; not packet-capture transport proof"}
        if windows_dds_interface is not None:
            report["ros_transport"]["interface_whitelist"] = {
                "windows": {"address": windows_dds_interface, "profile": str(transport), "sha256": sha(transport)},
                "linux": {"address": linux_dds_interface, "profile": str(linux_transport), "sha256": sha(linux_transport)}}
        save(receipt_path, report)
        if isolated_topic_relay:
            from factory_ops.ros_topic_relay_pair import RelayPair
            for name in ("factory_ops/ros_topic_relay.py","factory_ops/ros_topic_relay_pair.py"):
                with tarfile.open(stage/"source.tar") as archive:
                    member=archive.extractfile(name)
                    if member is None or hashlib.sha256(member.read()).hexdigest()!=sha(source/name):
                        raise ValueError("topic relay differs from frozen source archive")
            relay=RelayPair(stage,sdk,run_id,bound,ros_environment,linux_environment,
                relay_windows_address,relay_linux_address,relay_port)
            relay.start(linux,sha,save)
            report["relay_pair"]=relay.report
            save(receipt_path,report)
        charging_args = (["--virtual-charging", "--charge-robot", charge_robot,
                          "--charge-resume-soc", str(charge_resume_soc)] if charge_robot else [])
        command = "\n".join([
            "set -e", "source /opt/ros/jazzy/setup.bash",
            "export AMENT_PREFIX_PATH=" + shlex.quote(installed) + ':"${AMENT_PREFIX_PATH:-}"',
            "export LD_LIBRARY_PATH=" + shlex.quote(installed + "/lib") + ':"${LD_LIBRARY_PATH:-}"',
            "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1",
            *["export " + key + "=" + shlex.quote(value) for key, value in linux_environment.items()],
            "export FACTORY_OPS_G3_SOURCE_COMMIT=" + shlex.quote(launch["source_commit"]),
            "export FACTORY_OPS_NAV2_STARTUP_STAGGER_S=" + shlex.quote(str(float(nav2_startup_stagger_s))),
            "export FACTORY_OPS_G3_BUILD_ATTESTATION_PATH=" + shlex.quote(root_linux + "/build-attestation.json"),
            "export FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256=" + shlex.quote(launch["build_attestation_sha256"]),
            "/usr/bin/python3 -c " + shlex.quote("import json,os,sys; "
                "json.dump({k:os.environ.get(k) for k in ['RMW_FASTRTPS_USE_QOS_FROM_XML',"
                "'RMW_FASTRTPS_PUBLICATION_MODE','FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE']},"
                "open(sys.argv[1],'w'))") + " " + shlex.quote(root_linux+"/ros-environment.json"),
            "exec " + shlex.join(["/usr/bin/python3", root_linux + "/source/scripts/ros/fleet4_mission.py",
                "--bundle", root_linux + "/inputs", "--output", root_linux + "/results/run", "--run-id", run_id,
                "--runtime-profile", "local45_development", "--dashboard", *charging_args,
                *(["--live-anchor-controls"] if live_anchor_controls else [])]),
        ])
        (stage / "nav-command.sh").write_text(command + "\n", encoding="utf-8", newline="\n")
        linux_start_attempted = True
        report["linux_start"] = owner.start(["/bin/bash", linux(stage / "nav-command.sh")],
            working_directory=root_linux, log_path=root_linux + "/nav.log", runtime_bound_s=bound + 30)
        marker = output / "fleet-transport-verified.json"
        wait_for_bootstrap_marker(marker, owner, started=started, bound=bound)
        pins = json.loads(metadata.read_text(encoding="utf-8"))["source_files"]
        expected = {str(source / path): digest for path, digest in pins.items()}
        if live_anchor_controls:
            expected[str(source / "factory_ops/live_control.py")] = sha(source / "factory_ops/live_control.py")
        expected.update({str(path): sha(path) for path in (stage / "inputs").rglob("*") if path.is_file()})
        expected[str(metadata)] = sha(metadata)
        expected[str(marker)] = sha(marker)
        expected[str(transport)] = sha(transport)
        expected[str(linux_transport)] = sha(linux_transport)
        entry = source / "scripts/isaac/local45_fleet4_transport_scene.py"
        launcher = sdk / "python.bat"
        for path in (entry, launcher, output, stage):
            linux(path)  # Same metacharacter restriction protects the vendor cmd launcher.
        command_line = '"' + str(os.environ["COMSPEC"]) + '" /d /s /c ""' + str(launcher) + '" "' + str(entry) + '" --bundle "' + str(stage / "inputs") + '" --output "' + str(output) + '" --run-id ' + run_id + ' --lidar-backend ' + lidar_backend + ' --physics-cadence ' + physics_cadence + ' --capture-resolution ' + capture_resolution + ' --capture-views ' + capture_views + (' --carrier-modules' if carrier_modules else '') + '"'
        spec = {"run_id": run_id, "cwd": str(sdk), "command": command_line, "expected_inputs": expected}
        save(stage / "job-spec.json", spec)
        environment = {**os.environ, **ros_environment, "FACTORY_OPS_G3_SOURCE_COMMIT": launch["source_commit"],
            "FACTORY_OPS_FLEET4_PROVENANCE_SHA256": launch["provenance_sha256"], "PYTHONDONTWRITEBYTECODE": "1"}
        for label in ("sdk.stdout.log", "sdk.stderr.log"):
            streams.append((stage / label).open("xb"))
        windows_start_attempted = True
        worker = subprocess.Popen([sys.executable, str(source / "scripts/isaac/windows_job_runner.py"),
            "--spec", str(stage / "job-spec.json"), "--receipt", str(stage / "job.json"), "--timeout", str(bound),
            "--abort-request", str(stage / "job-abort-request.txt")],
            env=environment, stdout=streams[0], stderr=streams[1], creationflags=subprocess.CREATE_NO_WINDOW)
        report.update(status="running", windows_runner_pid=worker.pid, linux_unit=owner.unit_name)
        report["monitor_scope"] = "owned SDK failure checkpoint and foreground client; final owner.stop verifies Linux unit/cgroup/PID closure"
        save(receipt_path, report)
        client_exit = monitor_pair(worker, owner, started=started, bound=bound,
                                   failure_path=output / "sdk-failure.json", run_id=run_id,relay=relay)
        if client_exit is not None:
            report["ros_foreground_exit"] = client_exit
        report["windows_exit_code"] = worker.returncode
        mission_path = output / "mission-result.json"
        scene_path = output / "local45-fleet4-transport-scene.json"
        report["mission_status"] = (json.loads(mission_path.read_text(encoding="utf-8")).get("status")
                                    if mission_path.exists() else "missing")
        report["scene_status"] = (json.loads(scene_path.read_text(encoding="utf-8")).get("status")
                                  if scene_path.exists() else "missing")
        report["status"] = ("passed" if worker.returncode == 0
            and report["mission_status"] == "passed" and report["scene_status"] == "validated" else "failed")
    except Exception as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}", traceback=traceback.format_exc())
    finally:
        close_pair(worker, streams, owner, stage, report, started,
                   linux_start_attempted=linux_start_attempted, windows_start_attempted=windows_start_attempted)
        if relay is not None:
            report["relay_pair"]=relay.close(save)
            if report["relay_pair"]["status"]!="closed":report["status"]="failed"
            save(receipt_path,report)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, type=Path)
    parser.add_argument("--sdk", required=True, type=Path)
    parser.add_argument("--wall-bound-s", type=int, default=450)
    parser.add_argument("--live-anchor-controls", action="store_true")
    parser.add_argument("--dds-explicit-peers", action="store_true")
    parser.add_argument("--isolated-topic-relay", action="store_true")
    parser.add_argument("--startup-service-match-wait", action="store_true",
        help="Legacy flag: configure exact Linux lifecycle request/reply writer QoS; not a readiness wait")
    parser.add_argument("--relay-windows-address")
    parser.add_argument("--relay-linux-address")
    parser.add_argument("--relay-port",type=int,default=28473)
    parser.add_argument("--nav2-startup-stagger-s", type=float, default=0.)
    parser.add_argument("--capture-resolution", choices=("640x360", "1280x720"), default="640x360")
    parser.add_argument("--capture-views", choices=("all", "overview"), default="all")
    parser.add_argument("--carrier-modules", action="store_true")
    parser.add_argument("--charge-robot", choices=("carter1", "carter2", "carter3", "carter4"))
    parser.add_argument("--charge-resume-soc", type=float, default=.8)
    parser.add_argument("--lidar-backend", choices=("rtx_flat_scan", "physx_planar"), default="rtx_flat_scan")
    parser.add_argument("--physics-cadence", choices=("app_update", "owned_step6"), default="owned_step6")
    parser.add_argument("--windows-dds-interface")
    parser.add_argument("--linux-dds-interface")
    options = parser.parse_args()
    raise SystemExit(run(options.stage, options.sdk, options.wall_bound_s,
        charge_robot=options.charge_robot, charge_resume_soc=options.charge_resume_soc,
        lidar_backend=options.lidar_backend, physics_cadence=options.physics_cadence,
        windows_dds_interface=options.windows_dds_interface, linux_dds_interface=options.linux_dds_interface,
        live_anchor_controls=options.live_anchor_controls, dds_explicit_peers=options.dds_explicit_peers,
        nav2_startup_stagger_s=options.nav2_startup_stagger_s, capture_resolution=options.capture_resolution,
        capture_views=options.capture_views,
        carrier_modules=options.carrier_modules,
        isolated_topic_relay=options.isolated_topic_relay,relay_windows_address=options.relay_windows_address,
        relay_linux_address=options.relay_linux_address,relay_port=options.relay_port,
        startup_service_match_wait=options.startup_service_match_wait))
