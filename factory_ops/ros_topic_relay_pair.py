"""Run-owned relay processes; neither this owner nor a relay imports the World."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

from factory_ops.wsl_process_supervisor import WslProcessSupervisor


class RelayPair:
    def __init__(self, stage, sdk, run_id, bound, windows_environment, linux_environment,
                 windows_address, linux_address, port):
        self.stage,self.sdk,self.run_id,self.bound=stage,sdk,run_id,bound
        self.windows_environment,self.linux_environment=windows_environment,linux_environment
        self.address,self.peer,self.port=windows_address,linux_address,port
        self.owner=WslProcessSupervisor(run_id);self.worker=None;self.streams=[]
        self.report={"run_id":run_id,"status":"starting","windows_closed":False,"linux_closed":False,
            "scope":"allowlisted application transport; ROS discovery/actions/services remain within each OS"}

    def start(self, linux, sha, save):
        source=self.stage/"source";entry=source/"factory_ops/ros_topic_relay.py"
        common=["--run-id",self.run_id,"--host",self.address,"--port",str(self.port),
            "--wall-bound-s",str(min(10830,self.bound+30))]
        stop=self.stage/"relay-stop-request.txt"
        win=[str(self.sdk/"kit/python/python.exe"),"-B",str(entry),"--side","windows",*common,
            "--expected-peer",self.peer,"--sdk",str(self.sdk),"--output",str(self.stage/"windows-relay.json"),
            "--stop-request",str(stop)]
        spec={"run_id":self.run_id,"cwd":str(source),"command":subprocess.list2cmdline(win),
            "expected_inputs":{str(p):sha(p) for p in [entry,source/"factory_ops/isaac45_runtime.py",
                Path(self.windows_environment["FASTRTPS_DEFAULT_PROFILES_FILE"]),self.sdk/"VERSION"]}}
        save(self.stage/"relay-job-spec.json",spec)
        for label in ("relay-job.stdout.log","relay-job.stderr.log"):
            self.streams.append((self.stage/label).open("xb"))
        self.worker=subprocess.Popen([sys.executable,"-B",str(source/"scripts/isaac/windows_job_runner.py"),
            "--spec",str(self.stage/"relay-job-spec.json"),"--receipt",str(self.stage/"relay-job.json"),
            "--timeout",str(min(10800,self.bound+30)),"--abort-request",str(self.stage/"relay-job-abort.txt")],
            env={**os.environ,**self.windows_environment,"PYTHONPATH":str(source),"PYTHONDONTWRITEBYTECODE":"1"},
            stdout=self.streams[0],stderr=self.streams[1],creationflags=subprocess.CREATE_NO_WINDOW)
        shell="\n".join(["set -e","source /opt/ros/jazzy/setup.bash",
            *["export "+key+"="+shlex.quote(value) for key,value in self.linux_environment.items()],
            "export PYTHONDONTWRITEBYTECODE=1",
            "export PYTHONPATH="+shlex.quote(linux(source))+':"${PYTHONPATH:-}"',
            "exec "+shlex.join(["/usr/bin/python3","-B",linux(entry),"--side","linux",*common,
                "--output",linux(self.stage/"linux-relay.json"),"--stop-request",linux(stop)])])+"\n"
        command=self.stage/"relay-command.sh";command.write_text(shell,encoding="utf-8",newline="\n")
        self.report["linux_start"]=self.owner.start(["/bin/bash",linux(command)],working_directory=linux(source),
            log_path=linux(self.stage/"linux-relay.log"),runtime_bound_s=min(10830,self.bound+30))
        deadline=time.monotonic()+20
        while True:
            self.assert_healthy()
            states=[json.loads((self.stage/(side+"-relay.json")).read_text(encoding="utf-8"))
                    if (self.stage/(side+"-relay.json")).exists() else {} for side in ("windows","linux")]
            if all(s.get("status")=="ready" for s in states):break
            if time.monotonic()>deadline:raise TimeoutError("paired topic relay readiness deadline")
            time.sleep(.1)
        self.report.update(status="ready",windows_runner_pid=self.worker.pid,linux_unit=self.owner.unit_name,
            tcp_endpoints=[self.address,self.peer,self.port],relay_source_sha256=sha(entry))
        save(self.stage/"relay-pair.json",self.report)

    def assert_healthy(self):
        if self.worker is not None and self.worker.poll() is not None:
            raise RuntimeError("owned Windows topic relay exited")
        state=self.owner.client_state()
        if state["started"] and not state["running"]:raise RuntimeError("owned Linux topic relay exited")
        for side in ("windows","linux"):
            path=self.stage/(side+"-relay.json")
            if path.exists():
                document=json.loads(path.read_text(encoding="utf-8"))
                if document.get("run_id")!=self.run_id or document.get("status")=="failed":
                    raise RuntimeError("topic relay failed: "+side+" "+str(document.get("error")))

    def close(self, save):
        errors=[]
        stop=self.stage/"relay-stop-request.txt"
        if not stop.exists():stop.write_text(self.run_id,encoding="ascii")
        try:
            if self.worker is not None:self.worker.wait(timeout=8)
        except subprocess.TimeoutExpired:
            (self.stage/"relay-job-abort.txt").write_text(self.run_id,encoding="ascii")
            try:self.worker.wait(timeout=10)
            except Exception as error:errors.append("Windows relay wait: "+str(error))
        try:
            self.report["linux_end"]=self.owner.stop()
            self.report["linux_closed"]=self.report["linux_end"]["closure_verified"]
        except Exception as error:errors.append("Linux relay stop: "+str(error))
        for stream in self.streams:stream.close()
        path=self.stage/"relay-job.json"
        if path.exists():
            self.report["windows_end"]=json.loads(path.read_text(encoding="utf-8"))
            job=self.report["windows_end"]
            self.report["windows_closed"]=bool(job.get("closed") and not job.get("remaining_owned_pids"))
        elif self.worker is None:self.report["windows_closed"]=True
        self.report["endpoints"]={side:json.loads((self.stage/(side+"-relay.json")).read_text(encoding="utf-8"))
            for side in ("windows","linux") if (self.stage/(side+"-relay.json")).exists()}
        self.report.update(status="closed" if self.report["windows_closed"] and self.report["linux_closed"] and not errors else "failed",
            cleanup_errors=errors)
        save(self.stage/"relay-pair.json",self.report)
        return self.report
