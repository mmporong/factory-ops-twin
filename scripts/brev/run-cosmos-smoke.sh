#!/usr/bin/env bash
set -euo pipefail
umask 077

batch_id="${1:-379}"
target_profile="${2:-scale234}"
[[ "$batch_id" =~ ^(379|380|381|382|383|384|385|386)$ ]] || exit 2
[[ "$target_profile" =~ ^(scale234|g0)$ ]] || exit 2
work="$HOME/workspace/factoryops-cosmos-$batch_id"
out="$work/output"
secrets=/dev/shm/factoryops-cosmos-$batch_id
container=factoryops-cosmos-$batch_id
image='nvcr.io/nim/nvidia/cosmos3@sha256:2add946b316a0baf72a15be4da85132d127c34da22d2fe4e9c4bd146ad8c4020'
image_index_digest="${image##*@sha256:}"
shared_model_cache="$HOME/workspace/factoryops-cosmos-cache/$image_index_digest"
if [[ "$target_profile" == g0 ]]; then
  expected_host_gpus=1
  minimum_vram_gib=38
  minimum_ram_gib=36
  selector='model_type=generator,model_variant=nano,precision=fp8,profile=latency,offload=layer,text_guard_offload=resident,video_guard_offload=resident,n_gpus=1'
else
  expected_host_gpus=2
  minimum_vram_gib=35
  minimum_ram_gib=64
  selector='model_type=generator,model_variant=nano,precision=fp8,profile=latency,offload=layer,text_guard_offload=offloaded,video_guard_offload=offloaded,n_gpus=1'
fi
sampler_pid=''
disk_watcher_pid=''
disk_watcher_exit=''
mkdir -p "$out" "$shared_model_cache" "$secrets/docker"
test -s "$secrets/ngc-key"
test -s "$work/inference-request.json"

phase() {
  printf '%s\n' "$1" > "$out/phase.txt"
  printf 'COSMOS_PHASE %s\n' "$1"
}

redact_log() {
  python3 -c 'import pathlib,re,sys; key=pathlib.Path(sys.argv[1]).read_text().strip(); text=sys.stdin.read().replace(key,"[REDACTED]"); text=re.sub(r"nvapi-[A-Za-z0-9_-]+","[REDACTED]",text); text=re.sub(r"(?i)([?&](?:token|signature|x-amz-signature|x-amz-security-token)=)[^&\s]+",r"\1[REDACTED]",text); sys.stdout.write(text)' "$secrets/ngc-key"
}

cleanup() {
  original_status=$?
  trap - EXIT
  set +e
  if docker inspect "$container" >/dev/null 2>&1; then
    docker logs --tail 4000 "$container" 2>&1 | redact_log > "$out/server.log"
    docker rm -f "$container" >/dev/null 2>&1
  fi
  if [[ -n "$sampler_pid" ]]; then
    kill "$sampler_pid" 2>/dev/null
    wait "$sampler_pid" 2>/dev/null
  fi
  if [[ -n "$disk_watcher_pid" ]]; then
    kill "$disk_watcher_pid" 2>/dev/null
    wait "$disk_watcher_pid" 2>/dev/null
    disk_watcher_exit=$?
  fi
  # Delete only these owned volatile credential files, never a shared Docker config.
  rm -f "$secrets/ngc-key" "$secrets/ngc.env" "$secrets/docker/config.json"
  rmdir "$secrets/docker" "$secrets" 2>/dev/null
  python3 - "$out" "$original_status" "$container" "$secrets" "$disk_watcher_pid" "$disk_watcher_exit" <<'PY'
import datetime,json,pathlib,subprocess,sys
out=pathlib.Path(sys.argv[1])
p=subprocess.run(['docker','ps','-aq','--filter','name=^'+sys.argv[3]+'$'],capture_output=True,text=True)
remaining=bool(p.stdout.strip()) if p.returncode==0 else None
record={'checked_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'script_exit_code':int(sys.argv[2]),'owned_container_remaining':remaining,
        'credential_file_remaining':(pathlib.Path(sys.argv[4])/'ngc-key').exists(),
        'disk_watcher_pid':int(sys.argv[5]) if sys.argv[5] else None,
        'disk_watcher_exit_code':int(sys.argv[6]) if sys.argv[6] else None,
        'scope':'owned container and volatile credentials only; cloud lifecycle is supervised separately'}
(out/'cleanup.json').write_text(json.dumps(record,indent=2)+'\n')
PY
  exit "$original_status"
}
trap cleanup EXIT
trap 'exit 143' TERM
trap 'exit 130' INT

phase hardware
nvidia-smi --query-gpu=index,name,memory.total,memory.free,driver_version --format=csv,noheader,nounits > "$out/gpus-before.csv"
nvidia-smi --query-compute-apps=pid,process_name --format=csv,noheader > "$out/compute-before.csv"
test ! -s "$out/compute-before.csv"
docker info --format '{{.ServerVersion}}' > "$out/docker-version.txt"
python3 - "$work" "$out" "$target_profile" "$expected_host_gpus" "$minimum_vram_gib" "$minimum_ram_gib" "$selector" <<'PY'
import csv,json,pathlib,shutil,sys
work,out=map(pathlib.Path,sys.argv[1:3])
profile=sys.argv[3]
expected_host_gpus=int(sys.argv[4])
minimum_vram_gib=int(sys.argv[5])
minimum_ram_gib=int(sys.argv[6])
selector=sys.argv[7]
selector_fields=selector.split(',')
selector_keys=[field.split('=',1)[0] for field in selector_fields]
assert len(selector_keys)==len(set(selector_keys)),'duplicate selector field'
gpus=list(csv.reader((out/'gpus-before.csv').read_text().splitlines()))
assert len(gpus)==expected_host_gpus and all('L40S' in row[1] for row in gpus),'unexpected GPU identity or cardinality'
mem={k:int(v.split()[0])*1024 for k,v in (line.split(':',1) for line in pathlib.Path('/proc/meminfo').read_text().splitlines())}
free_disk=shutil.disk_usage(work).free
record={'target_profile':profile,'visible_host_gpus':len(gpus),'expected_host_gpus':expected_host_gpus,
        'selected_device':0,'selected_free_vram_mib':int(gpus[0][3]),
        'host_available_ram_bytes':mem['MemAvailable'],'free_disk_bytes':free_disk,
        'authored_entry_minimum_free_disk_gib':32,'authored_runtime_low_disk_gib':24,
        'disk_guard_claim':'project-authored bounded guard; not an official NVIDIA minimum',
        'authored_minimum_free_vram_gib':minimum_vram_gib,
        'authored_minimum_host_ram_gib':minimum_ram_gib,'selector':selector}
(out/'hardware.json').write_text(json.dumps(record,indent=2)+'\n')
(out/'profile.json').write_text(json.dumps({'target_profile':profile,'selector':selector},indent=2)+'\n')
assert record['selected_free_vram_mib']>=minimum_vram_gib*1024,'insufficient free per-device VRAM'
assert mem['MemAvailable']>=minimum_ram_gib*1024**3,'insufficient available host RAM'
PY

minimum_disk_bytes=$((32 * 1024 * 1024 * 1024))
free_before=$(df -B1 --output=avail "$work" | tail -n 1 | tr -d ' ')
if (( free_before < minimum_disk_bytes )); then
  phase storage_preparation
  df -B1 -T > "$out/storage-filesystems-before.txt"
  set +e
  timeout --signal=TERM --kill-after=10s 120s docker system df > "$out/docker-system-df-before.txt" 2>&1
  docker_system_df_before_exit=$?
  timeout --signal=TERM --kill-after=10s 120s python3 -m pip cache info > "$out/pip-cache-info-before.txt" 2>&1
  pip_cache_info_before_exit=$?
  timeout --signal=TERM --kill-after=10s 120s python3 -m pip cache dir > "$out/pip-cache-dir.txt"
  pip_cache_dir_exit=$?
  set -e
  set +e
  pip_path_allowed=$(python3 - "$out" "$HOME" "$pip_cache_dir_exit" <<'PY'
import json,pathlib,sys
def validate_pip_cache_path(home_value,reported_value):
    home=pathlib.Path(home_value).resolve()
    reported=pathlib.Path(reported_value).expanduser()
    standard=home/'.cache'/'pip'
    record={'reported_path':str(reported),'standard_path':str(standard),
            'reported_is_symlink':reported.is_symlink(),'allowed':False}
    try:
        reported_resolved=reported.resolve()
        standard_resolved=standard.resolve()
        reported_resolved.relative_to(home)
        record['reported_resolved']=str(reported_resolved)
        record['standard_resolved']=str(standard_resolved)
        record['inside_home']=True
        record['canonical_standard_match']=reported_resolved==standard_resolved
        record['allowed']=(not reported.is_symlink() and reported_resolved==standard_resolved)
    except (OSError,ValueError) as error:
        record['validation_error']=type(error).__name__
    return record
out=pathlib.Path(sys.argv[1])
cache_dir_exit=int(sys.argv[3])
cache_dir_file=out/'pip-cache-dir.txt'
if cache_dir_exit==0 and cache_dir_file.is_file() and cache_dir_file.read_text().strip():
    record=validate_pip_cache_path(sys.argv[2],cache_dir_file.read_text().strip())
else:
    record={'reported_path':None,'standard_path':str(pathlib.Path(sys.argv[2]).resolve()/'.cache'/'pip'),
            'reported_is_symlink':None,'allowed':False,
            'validation_error':'pip_cache_dir_command_failed_or_empty'}
record['cache_dir_command_exit']=cache_dir_exit
(out/'pip-cache-verification.json').write_text(json.dumps(record,indent=2)+'\n')
print('true' if record['allowed'] else 'false')
PY
  )
  pip_pathguard_exit=$?
  set -e
  pip_purge_attempted=false
  pip_purge_exit=-1
  if [[ "$pip_pathguard_exit" -eq 0 && "$pip_path_allowed" == true ]]; then
    pip_purge_attempted=true
    set +e
    timeout --signal=TERM --kill-after=10s 120s python3 -m pip cache purge > "$out/pip-cache-purge.log" 2>&1
    pip_purge_exit=$?
    set -e
  fi
  free_after_pip=$(df -B1 --output=avail "$work" | tail -n 1 | tr -d ' ')
  builder_prune_attempted=false
  builder_prune_exit=-1
  if (( free_after_pip < minimum_disk_bytes )); then
    builder_prune_attempted=true
    set +e
    timeout --signal=TERM --kill-after=10s 120s docker builder prune --force --all --keep-storage 0GB > "$out/docker-builder-prune.log" 2>&1
    builder_prune_exit=$?
    set -e
  fi
  free_after=$(df -B1 --output=avail "$work" | tail -n 1 | tr -d ' ')
  disk_diagnostics_attempted=false
  workspace_du_exit=-1
  cache_du_exit=-1
  if (( free_after < minimum_disk_bytes )); then
    disk_diagnostics_attempted=true
    set +e
    timeout --signal=TERM --kill-after=10s 60s du -x -B1 -d2 "$HOME/workspace" > "$out/workspace-du-depth2.txt" 2>&1
    workspace_du_exit=$?
    timeout --signal=TERM --kill-after=10s 60s du -x -B1 -d2 "$HOME/.cache" > "$out/home-cache-du-depth2.txt" 2>&1
    cache_du_exit=$?
    set -e
  fi
  df -B1 -T > "$out/storage-filesystems-after.txt"
  set +e
  timeout --signal=TERM --kill-after=10s 120s docker system df > "$out/docker-system-df-after.txt" 2>&1
  docker_system_df_after_exit=$?
  timeout --signal=TERM --kill-after=10s 120s python3 -m pip cache info > "$out/pip-cache-info-after.txt" 2>&1
  pip_cache_info_after_exit=$?
  set -e
  python3 - "$out" "$minimum_disk_bytes" "$free_before" "$free_after_pip" "$free_after" \
    "$pip_purge_attempted" "$pip_purge_exit" "$builder_prune_attempted" "$builder_prune_exit" \
    "$disk_diagnostics_attempted" "$workspace_du_exit" "$cache_du_exit" \
    "$docker_system_df_before_exit" "$docker_system_df_after_exit" \
    "$pip_cache_info_before_exit" "$pip_cache_info_after_exit" \
    "$pip_cache_dir_exit" "$pip_pathguard_exit" "$pip_path_allowed" <<'PY'
import datetime,json,pathlib,sys
out=pathlib.Path(sys.argv[1])
record={'schema_version':1,'recorded_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'minimum_free_bytes':int(sys.argv[2]),'before_free_bytes':int(sys.argv[3]),
        'after_pip_free_bytes':int(sys.argv[4]),'after_free_bytes':int(sys.argv[5]),
        'pip_cache_purge':{'attempted':sys.argv[6]=='true','exit_code':int(sys.argv[7]),
                           'path_allowed':sys.argv[19]=='true','log':'pip-cache-purge.log'},
        'docker_builder_prune':{'attempted':sys.argv[8]=='true','exit_code':int(sys.argv[9]),
                                'keep_storage':'0GB','all_unused':True,'log':'docker-builder-prune.log'},
        'insufficient_space_diagnostics':{'attempted':sys.argv[10]=='true',
                'workspace_du_exit_code':int(sys.argv[11]),'cache_du_exit_code':int(sys.argv[12]),
                'workspace_log':'workspace-du-depth2.txt','cache_log':'home-cache-du-depth2.txt',
                'depth':2,'same_filesystem':True},
        'diagnostics':{
                'docker_system_df_before':{'exit_code':int(sys.argv[13]),'log':'docker-system-df-before.txt'},
                'docker_system_df_after':{'exit_code':int(sys.argv[14]),'log':'docker-system-df-after.txt'},
                'pip_cache_info_before':{'exit_code':int(sys.argv[15]),'log':'pip-cache-info-before.txt'},
                'pip_cache_info_after':{'exit_code':int(sys.argv[16]),'log':'pip-cache-info-after.txt'},
                'pip_cache_dir':{'exit_code':int(sys.argv[17]),'log':'pip-cache-dir.txt'},
                'pip_pathguard':{'exit_code':int(sys.argv[18]),'log':'pip-cache-verification.json',
                                 'path_allowed':sys.argv[19]=='true'}},
        'prohibited_cleanup_not_run':['docker image prune','docker container prune','docker volume prune',
                                      'dataset deletion','model weight deletion','installed package deletion']}
record['passed']=record['after_free_bytes']>=record['minimum_free_bytes']
(out/'storage-preparation.json').write_text(json.dumps(record,indent=2)+'\n')
PY
  test "$free_after" -ge "$minimum_disk_bytes"
fi
python3 - "$out" "$shared_model_cache" "$image" "$image_index_digest" "$target_profile" "$selector" <<'PY'
import json,pathlib,re,sys
out=pathlib.Path(sys.argv[1])
cache=pathlib.Path(sys.argv[2])
image=sys.argv[3]
digest=sys.argv[4]
profile=sys.argv[5]
selector=sys.argv[6]
expected=pathlib.Path.home()/'workspace'/'factoryops-cosmos-cache'/digest
assert re.fullmatch(r'[0-9a-f]{64}',digest),'invalid image index digest'
assert not cache.is_symlink(),'shared model cache path must not be a symlink'
assert cache.resolve()==expected.resolve(),'shared model cache path changed'
record={'schema_version':1,'cache_path':str(cache.resolve()),'image':image,
        'image_index_digest':digest,'target_profile':profile,'selector':selector,
        'reuse_scope':'owned content-addressed model cache shared across bounded Cosmos runs',
        'collected_with_run_output':False,'automatic_deletion':False}
(out/'model-cache-contract.json').write_text(json.dumps(record,indent=2)+'\n')
PY

script_full=$(readlink -f "$0")
parent_pid=$$
parent_starttime=$(python3 - "$parent_pid" <<'PY'
import pathlib,sys
text=(pathlib.Path('/proc')/sys.argv[1]/'stat').read_text()
print(text.rsplit(')',1)[1].split()[19])
PY
)
parent_pgid=$(ps -o pgid= -p "$parent_pid" | tr -d ' ')
leader_starttime=$(python3 - "$parent_pgid" <<'PY'
import pathlib,sys
text=(pathlib.Path('/proc')/sys.argv[1]/'stat').read_text()
print(text.rsplit(')',1)[1].split()[19])
PY
)
cat > "$work/disk-watcher.py" <<'PY'
import json
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import time


def classify_parent_identity(expected, observed):
    parent_same=(observed.get('parent_pid')==expected['parent_pid'] and
                 observed.get('parent_starttime')==expected['parent_starttime'])
    if not parent_same:
        return False, False, 'parent_pid_or_starttime_changed'
    if observed.get('pgid')!=expected['pgid']:
        return False, True, 'process_group_changed'
    if observed.get('leader_pid')!=expected['pgid']:
        return False, True, 'group_leader_pid_mismatch'
    if observed.get('leader_starttime')!=expected['leader_starttime']:
        return False, True, 'group_leader_starttime_changed'
    argv=observed.get('leader_argv') or []
    if not argv or pathlib.Path(argv[0]).name!='timeout':
        return False, True, 'group_leader_is_not_gnu_timeout'
    signature=[expected['run_script'],expected['batch_id'],expected['target_profile']]
    if not any(argv[index:index+3]==signature for index in range(len(argv)-2)):
        return False, True, 'timeout_command_does_not_match_owned_run'
    return True, True, 'owned_timeout_group'


def proc_starttime(pid):
    text=(pathlib.Path('/proc')/str(pid)/'stat').read_text()
    return text.rsplit(')',1)[1].split()[19]


def observe(expected):
    parent=expected['parent_pid']
    pgid=os.getpgid(parent)
    leader=pgid
    argv=(pathlib.Path('/proc')/str(leader)/'cmdline').read_bytes().split(b'\0')
    return {'parent_pid':parent,'parent_starttime':proc_starttime(parent),'pgid':pgid,
            'leader_pid':leader,'leader_starttime':proc_starttime(leader),
            'leader_argv':[value.decode(errors='replace') for value in argv if value]}


def atomic_json(path,record):
    temporary=path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(record,indent=2)+'\n')
    os.replace(temporary,path)


def stop_owned_container(name):
    inspect=subprocess.run(['docker','inspect','--format','{{.Name}}',name],capture_output=True,text=True,timeout=5)
    result={'name':name,'inspect_exit_code':inspect.returncode,'verified_exact_name':False,
            'kill_exit_code':None,'remove_exit_code':None}
    if inspect.returncode==0 and inspect.stdout.strip()=='/'+name:
        result['verified_exact_name']=True
        result['kill_exit_code']=subprocess.run(['docker','kill',name],capture_output=True,timeout=5).returncode
        result['remove_exit_code']=subprocess.run(['docker','rm','--force',name],capture_output=True,timeout=5).returncode
    return result


out=pathlib.Path(sys.argv[1])
work=pathlib.Path(sys.argv[2])
threshold=int(sys.argv[3])
expected={'parent_pid':int(sys.argv[4]),'parent_starttime':sys.argv[5],
          'pgid':int(sys.argv[6]),'leader_starttime':sys.argv[7],
          'run_script':sys.argv[8],'batch_id':sys.argv[9],'target_profile':sys.argv[10]}
container=sys.argv[11]
summary_path=out/'disk-guard.json'
trigger_path=out/'disk-guard-trigger.json'
stopping=False
def handle_stop(_signum,_frame):
    global stopping
    stopping=True
signal.signal(signal.SIGTERM,handle_stop)
signal.signal(signal.SIGINT,handle_stop)
initial=shutil.disk_usage(work).free
minimum=initial
phase_name=None
phase_started=time.monotonic()
summary={'schema_version':1,'status':'starting','initial_free_bytes':initial,
         'minimum_free_bytes':minimum,'current_free_bytes':initial,
         'low_disk_threshold_bytes':threshold,'sample_interval_seconds':1,
         'claim':'project-authored runtime guard; not an official NVIDIA minimum'}
atomic_json(summary_path,summary)
while True:
    try:
        observation=observe(expected)
    except (FileNotFoundError,ProcessLookupError,PermissionError,OSError) as error:
        summary.update(status='parent_gone_or_unreadable',identity_error=type(error).__name__)
        atomic_json(summary_path,summary)
        raise SystemExit(0)
    owned,parent_same,reason=classify_parent_identity(expected,observation)
    if not owned:
        summary.update(status='parent_identity_rejected',identity_reason=reason,
                       parent_identity_same=parent_same)
        atomic_json(summary_path,summary)
        raise SystemExit(0)
    current=shutil.disk_usage(work).free
    minimum=min(minimum,current)
    try:
        observed_phase=(out/'phase.txt').read_text().strip()
    except OSError:
        observed_phase='unknown'
    if observed_phase!=phase_name:
        phase_name=observed_phase
        phase_started=time.monotonic()
    phase_elapsed=time.monotonic()-phase_started
    image_pull_expired=(phase_name=='image_pull' and phase_elapsed>=1800)
    summary.update(status='monitoring',minimum_free_bytes=minimum,current_free_bytes=current,
                   identity_reason=reason,phase=phase_name,
                   phase_elapsed_seconds=round(phase_elapsed,3),image_pull_limit_seconds=1800)
    atomic_json(summary_path,summary)
    if current<threshold or image_pull_expired:
        trigger_reason='low_disk' if current<threshold else 'image_pull_deadline'
        trigger={'schema_version':1,'status':'triggered','trigger_reason':trigger_reason,'free_bytes':current,
                 'threshold_bytes':threshold,'identity_reason':reason,
                 'phase':phase_name,'phase_elapsed_seconds':phase_elapsed,
                 'owned_container':container,'recorded_unix_seconds':time.time()}
        atomic_json(trigger_path,trigger)
        try:
            trigger['container_stop']=stop_owned_container(container)
        except (OSError,subprocess.TimeoutExpired) as error:
            trigger['container_stop']={'error_type':type(error).__name__}
        atomic_json(trigger_path,trigger)
        try:
            latest=observe(expected)
            group_owned,parent_same,latest_reason=classify_parent_identity(expected,latest)
        except (FileNotFoundError,ProcessLookupError,PermissionError,OSError) as error:
            group_owned,parent_same,latest_reason=False,False,type(error).__name__
        trigger.update(group_owned=group_owned,parent_identity_same=parent_same,
                       final_identity_reason=latest_reason)
        summary.update(status=trigger_reason+'_triggered',minimum_free_bytes=minimum,
                       current_free_bytes=current,final_identity_reason=latest_reason)
        atomic_json(trigger_path,trigger)
        atomic_json(summary_path,summary)
        if group_owned:
            os.killpg(expected['pgid'],signal.SIGTERM)
        elif parent_same:
            os.kill(expected['parent_pid'],signal.SIGTERM)
        raise SystemExit(2)
    if stopping:
        summary.update(status='stopped_by_cleanup',minimum_free_bytes=minimum,
                       current_free_bytes=current)
        atomic_json(summary_path,summary)
        raise SystemExit(0)
    time.sleep(1)
PY
runtime_disk_threshold=$((24 * 1024 * 1024 * 1024))
python3 "$work/disk-watcher.py" "$out" "$work" "$runtime_disk_threshold" \
  "$parent_pid" "$parent_starttime" "$parent_pgid" "$leader_starttime" \
  "$script_full" "$batch_id" "$target_profile" "$container" > "$out/disk-watcher.log" 2>&1 &
disk_watcher_pid=$!
sleep 1
kill -0 "$disk_watcher_pid"
nvidia-smi --query-gpu=timestamp,index,memory.used,utilization.gpu --format=csv --loop=5 > "$out/gpu-samples.csv" &
sampler_pid=$!

phase image_pull
cat "$secrets/ngc-key" | docker --config "$secrets/docker" login nvcr.io --username '$oauthtoken' --password-stdin 2>&1 | redact_log > "$out/registry-login.log"
docker --config "$secrets/docker" pull "$image" 2>&1 | redact_log > "$out/image-pull.log"
docker image inspect "$image" --format '{{json .RepoDigests}}' > "$out/image-digests.json"
docker run --rm --entrypoint /bin/bash "$image" -lc 'for p in /opt/nim/LICENSE /opt/nim/MODEL_LICENSE /opt/nim/NOTICE; do test -f "$p"; printf "\nFILE %s\n" "$p"; cat "$p"; done' > "$out/image-license-notices.txt"

phase profile_preflight
timeout --signal=TERM --kill-after=30s 240s docker run --rm --name "$container" --gpus '"device=0"' \
  -e NIM_TAGS_SELECTOR="$selector" --entrypoint /bin/bash "$image" -lc \
  '/opt/nim/.venv/bin/python -m profile_selection.startup --output /tmp/cosmos3-preflight.env && cat /tmp/cosmos3-preflight.env' \
  2>&1 | redact_log > "$out/profile-preflight.log"

phase cold_start
{ printf 'NGC_API_KEY='; cat "$secrets/ngc-key"; printf '\n'; } > "$secrets/ngc.env"
docker run -d --name "$container" --gpus '"device=0"' --shm-size=32g \
  --ulimit nofile=65536:65536 -p 127.0.0.1:18080:8000 \
  --env-file "$secrets/ngc.env" -e NIM_TAGS_SELECTOR="$selector" \
  -e NIM_TELEMETRY_MODE=0 -e NIM_LOG_LEVEL=INFO -e NIM_TRITON_REQUEST_TIMEOUT=3600000000 \
  -v "$shared_model_cache:/opt/nim/.cache" "$image" > "$out/container-id.txt"
python3 - "$out" "$work" "$container" "$batch_id" "$target_profile" <<'PY'
import base64,datetime,hashlib,json,pathlib,subprocess,time,urllib.error,urllib.request,sys
out,work=map(pathlib.Path,sys.argv[1:3])
base='http://127.0.0.1:18080'
deadline=time.monotonic()+3000
checks=[]
while time.monotonic()<deadline:
    state=subprocess.run(['docker','inspect','--format','{{.State.Running}}',sys.argv[3]],capture_output=True,text=True)
    if state.returncode or state.stdout.strip()!='true':
        (out/'readiness.json').write_text(json.dumps(checks,indent=2)+'\n')
        raise RuntimeError('owned model container exited before readiness')
    try:
        with urllib.request.urlopen(base+'/v1/health/ready',timeout=15) as response:
            status=response.status
        checks.append({'elapsed_seconds':round(3000-(deadline-time.monotonic()),2),'http_status':status})
        if status==200: break
    except (urllib.error.URLError,TimeoutError,ConnectionError) as error:
        checks.append({'elapsed_seconds':round(3000-(deadline-time.monotonic()),2),'error_type':type(error).__name__})
    (out/'readiness.json').write_text(json.dumps(checks,indent=2)+'\n')
    time.sleep(15)
else:
    raise RuntimeError('model readiness deadline exceeded')
(out/'readiness.json').write_text(json.dumps(checks,indent=2)+'\n')
for name in ['metadata','manifest','license']:
    with urllib.request.urlopen(base+'/v1/'+name,timeout=30) as response:
        body=response.read()
    (out/(name+'.json')).write_bytes(body)
payload=(work/'inference-request.json').read_bytes()
request=json.loads(payload)
assert request['num_frames']==121 and request['seed']==2026
(out/'phase.txt').write_text('inference\n')
started=time.monotonic()
with urllib.request.urlopen(urllib.request.Request(base+'/v1/infer',data=payload,headers={'Content-Type':'application/json'}),timeout=3600) as response:
    result=json.load(response)
video=base64.b64decode(result['b64_video'],validate=True)
assert 0<len(video)<=100*1024**2,'unexpected output size'
(out/'output.mp4').write_bytes(video)
record={'schema_version':1,'status':'inference_completed','completed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'batch_id':int(sys.argv[4]),'target_profile':sys.argv[5],
        'request_sha256':hashlib.sha256(payload).hexdigest(),'source_bundle_sha256':hashlib.sha256((work/'bundle.json').read_bytes()).hexdigest(),
        'inference_wall_seconds':time.monotonic()-started,'output_bytes':len(video),'output_sha256':hashlib.sha256(video).hexdigest(),
        'requested_frames':121,'requested_fps':30,'seed':2026,'training_ready':False,'model_output_labels_approved':False,
        'claim_limits':['one static-input compatibility smoke','not motion validation, label approval or a detector performance result']}
(out/'result.json').write_text(json.dumps(record,indent=2)+'\n')
(out/'phase.txt').write_text('inference_completed\n')
print(json.dumps({'status':record['status'],'output_bytes':len(video),'inference_wall_seconds':record['inference_wall_seconds']}),flush=True)
PY
phase completed
