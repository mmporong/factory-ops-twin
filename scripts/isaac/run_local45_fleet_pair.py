"""Run an already prepared SDK4.5/WSL development pair with separate tree owners."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from factory_ops.wsl_process_supervisor import WslProcessSupervisor


def save(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def linux(path: Path) -> str:
    if not path.is_absolute() or not path.drive or any(c in str(path) for c in '\r\n%&|<>^!"'):
        raise ValueError("runtime paths must be absolute drive paths without shell metacharacters")
    return "/mnt/" + path.drive[0].lower() + path.as_posix()[2:]


def prepare_ros_transport(stage: Path, domain_id: int) -> tuple[Path, dict[str, str]]:
    """Create the run-owned UDP participant profile before ROS processes start."""
    if isinstance(domain_id, bool) or not isinstance(domain_id, int) or not 0 <= domain_id <= 232:
        raise ValueError("bundle ROS domain must be an integer in0..232")
    profile = stage / "fastdds-udp.xml"
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
    with profile.open("x", encoding="utf-8", newline="\n") as stream:
        stream.write(text)
    return profile, {"ROS_DOMAIN_ID": str(domain_id), "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "ROS_LOCALHOST_ONLY": "0", "FASTRTPS_DEFAULT_PROFILES_FILE": str(profile),
        "FASTDDS_DEFAULT_PROFILES_FILE": str(profile)}


def close_pair(worker, streams, owner, stage: Path, report: dict, started: float) -> None:
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
    try:
        report["linux_end"] = owner.stop()
        report["linux_closed"] = report["linux_end"]["closure_verified"]
    except Exception as error:
        errors.append("linux stop: " + str(error))
    try:
        job = json.loads((stage / "job.json").read_text(encoding="utf-8"))
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
        if now - started > bound + 35:
            raise TimeoutError("pair owner deadline exceeded")
        client = owner.client_state()
        if now > deadline or not client["started"] or not client["running"]:
            raise RuntimeError("ROS mission did not create its verified bootstrap marker with a living foreground client")
        if marker.exists():
            return
        time.sleep(.2)


def monitor_pair(worker, owner: WslProcessSupervisor, *, started: float, bound: int) -> dict | None:
    """Observe client exit, retaining SDK cleanup grace and the pair deadline."""
    exit_seen = None
    exit_record = None
    while worker.poll() is None:
        now = time.monotonic()
        if now - started > bound + 35:
            raise TimeoutError("pair owner deadline exceeded")
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


def run(stage: Path, sdk: Path, bound: int, *, charge_robot: str | None = None, charge_resume_soc: float = .8) -> int:
    if os.name != "nt" or not 60 <= bound <= 7200:
        raise ValueError("local development requires Windows and an explicit 60..7200s bound")
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
    receipt_path = stage / "pair-host.json"
    if receipt_path.exists() or (stage / "nav.log").exists() or (stage / "job-spec.json").exists():
        raise ValueError("preserve prior attempt; prepared stage must be unused")
    report = {"schema_version": 1, "run_id": run_id, "source_commit": launch["source_commit"],
        "scope": "bounded local45 development integration admission; no qualification or completion claim",
        "launcher_sha256": sha(Path(__file__)), "wsl_supervisor_sha256": sha(Path(__file__).resolve().parents[2] / "factory_ops/wsl_process_supervisor.py"),
        "status": "starting", "wall_bound_s": bound, "windows_closed": False, "linux_closed": False}
    report["virtual_charging_request"] = {"robot_id": charge_robot, "resume_soc": charge_resume_soc}
    save(receipt_path, report)
    owner = WslProcessSupervisor(run_id)
    worker = None
    streams = []
    started = time.monotonic()
    try:
        root_linux = linux(stage)
        attestation = json.loads((stage / "build-attestation.json").read_text(encoding="utf-8"))
        installed = linux(Path(attestation["actual_install_directory"]))
        config = json.loads((stage / "inputs/config.json").read_text(encoding="utf-8"))
        transport, ros_environment = prepare_ros_transport(stage, config["ros_domain_id"])
        linux_environment = {**ros_environment, "FASTRTPS_DEFAULT_PROFILES_FILE": linux(transport),
                             "FASTDDS_DEFAULT_PROFILES_FILE": linux(transport)}
        report["ros_transport"] = {"profile": str(transport), "sha256": sha(transport),
            "windows_environment": ros_environment, "linux_environment": linux_environment,
            "scope": "configured run-local UDP profile; not packet-capture transport proof"}
        save(receipt_path, report)
        charging_args = (["--virtual-charging", "--charge-robot", charge_robot,
                          "--charge-resume-soc", str(charge_resume_soc)] if charge_robot else [])
        command = "\n".join([
            "set -e", "source /opt/ros/jazzy/setup.bash",
            "export AMENT_PREFIX_PATH=" + shlex.quote(installed) + ':"${AMENT_PREFIX_PATH:-}"',
            "export LD_LIBRARY_PATH=" + shlex.quote(installed + "/lib") + ':"${LD_LIBRARY_PATH:-}"',
            "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0 PYTHONDONTWRITEBYTECODE=1",
            *["export " + key + "=" + shlex.quote(value) for key, value in linux_environment.items()],
            "export FACTORY_OPS_G3_SOURCE_COMMIT=" + shlex.quote(launch["source_commit"]),
            "export FACTORY_OPS_G3_BUILD_ATTESTATION_PATH=" + shlex.quote(root_linux + "/build-attestation.json"),
            "export FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256=" + shlex.quote(launch["build_attestation_sha256"]),
            "exec " + shlex.join(["/usr/bin/python3", root_linux + "/source/scripts/ros/fleet4_mission.py",
                "--bundle", root_linux + "/inputs", "--output", root_linux + "/results/run", "--run-id", run_id,
                "--runtime-profile", "local45_development", "--dashboard", *charging_args]),
        ])
        (stage / "nav-command.sh").write_text(command + "\n", encoding="utf-8", newline="\n")
        report["linux_start"] = owner.start(["/bin/bash", linux(stage / "nav-command.sh")],
            working_directory=root_linux, log_path=root_linux + "/nav.log", runtime_bound_s=bound + 30)
        marker = output / "fleet-transport-verified.json"
        wait_for_bootstrap_marker(marker, owner, started=started, bound=bound)
        pins = json.loads(metadata.read_text(encoding="utf-8"))["source_files"]
        expected = {str(source / path): digest for path, digest in pins.items()}
        expected.update({str(path): sha(path) for path in (stage / "inputs").rglob("*") if path.is_file()})
        expected[str(metadata)] = sha(metadata)
        expected[str(marker)] = sha(marker)
        expected[str(transport)] = sha(transport)
        entry = source / "scripts/isaac/local45_fleet4_transport_scene.py"
        launcher = sdk / "python.bat"
        for path in (entry, launcher, output, stage):
            linux(path)  # Same metacharacter restriction protects the vendor cmd launcher.
        command_line = '"' + str(os.environ["COMSPEC"]) + '" /d /s /c ""' + str(launcher) + '" "' + str(entry) + '" --bundle "' + str(stage / "inputs") + '" --output "' + str(output) + '" --run-id ' + run_id + '"'
        spec = {"run_id": run_id, "cwd": str(sdk), "command": command_line, "expected_inputs": expected}
        save(stage / "job-spec.json", spec)
        environment = {**os.environ, **ros_environment, "FACTORY_OPS_G3_SOURCE_COMMIT": launch["source_commit"],
            "FACTORY_OPS_FLEET4_PROVENANCE_SHA256": launch["provenance_sha256"], "PYTHONDONTWRITEBYTECODE": "1"}
        for label in ("sdk.stdout.log", "sdk.stderr.log"):
            streams.append((stage / label).open("xb"))
        worker = subprocess.Popen([sys.executable, str(source / "scripts/isaac/windows_job_runner.py"),
            "--spec", str(stage / "job-spec.json"), "--receipt", str(stage / "job.json"), "--timeout", str(bound),
            "--abort-request", str(stage / "job-abort-request.txt")],
            env=environment, stdout=streams[0], stderr=streams[1], creationflags=subprocess.CREATE_NO_WINDOW)
        report.update(status="running", windows_runner_pid=worker.pid, linux_unit=owner.unit_name)
        report["monitor_scope"] = "owned foreground client only; final owner.stop verifies Linux unit/cgroup/PID closure"
        save(receipt_path, report)
        client_exit = monitor_pair(worker, owner, started=started, bound=bound)
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
        close_pair(worker, streams, owner, stage, report, started)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", required=True, type=Path)
    parser.add_argument("--sdk", required=True, type=Path)
    parser.add_argument("--wall-bound-s", type=int, default=450)
    parser.add_argument("--charge-robot", choices=("carter1", "carter2", "carter3", "carter4"))
    parser.add_argument("--charge-resume-soc", type=float, default=.8)
    options = parser.parse_args()
    raise SystemExit(run(options.stage, options.sdk, options.wall_bound_s,
        charge_robot=options.charge_robot, charge_resume_soc=options.charge_resume_soc))
