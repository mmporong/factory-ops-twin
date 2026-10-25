#!/usr/bin/env bash
set -euo pipefail
umask 077

batch_id="${1:-379}"
target_profile="${2:-scale234}"
[[ "$batch_id" =~ ^(379|380|381|382)$ ]] || exit 2
[[ "$target_profile" =~ ^(scale234|g0)$ ]] || exit 2
work="$HOME/workspace/factoryops-cosmos-$batch_id"
out="$work/output"
secrets=/dev/shm/factoryops-cosmos-$batch_id
container=factoryops-cosmos-$batch_id
image='nvcr.io/nim/nvidia/cosmos3@sha256:2add946b316a0baf72a15be4da85132d127c34da22d2fe4e9c4bd146ad8c4020'
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
mkdir -p "$out" "$work/cache" "$secrets/docker"
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
  # Delete only these owned volatile credential files, never a shared Docker config.
  rm -f "$secrets/ngc-key" "$secrets/ngc.env" "$secrets/docker/config.json"
  rmdir "$secrets/docker" "$secrets" 2>/dev/null
  python3 - "$out" "$original_status" "$container" "$secrets" <<'PY'
import datetime,json,pathlib,subprocess,sys
out=pathlib.Path(sys.argv[1])
p=subprocess.run(['docker','ps','-aq','--filter','name=^'+sys.argv[3]+'$'],capture_output=True,text=True)
remaining=bool(p.stdout.strip()) if p.returncode==0 else None
record={'checked_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'script_exit_code':int(sys.argv[2]),'owned_container_remaining':remaining,
        'credential_file_remaining':(pathlib.Path(sys.argv[4])/'ngc-key').exists(),
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
        'authored_minimum_free_disk_gib':100,'authored_minimum_free_vram_gib':minimum_vram_gib,
        'authored_minimum_host_ram_gib':minimum_ram_gib,'selector':selector}
(out/'hardware.json').write_text(json.dumps(record,indent=2)+'\n')
(out/'profile.json').write_text(json.dumps({'target_profile':profile,'selector':selector},indent=2)+'\n')
assert record['selected_free_vram_mib']>=minimum_vram_gib*1024,'insufficient free per-device VRAM'
assert mem['MemAvailable']>=minimum_ram_gib*1024**3,'insufficient available host RAM'
assert free_disk>=100*1024**3,'insufficient authored disk reserve'
PY
nvidia-smi --query-gpu=timestamp,index,memory.used,utilization.gpu --format=csv --loop=5 > "$out/gpu-samples.csv" &
sampler_pid=$!

phase image_pull
cat "$secrets/ngc-key" | docker --config "$secrets/docker" login nvcr.io --username '$oauthtoken' --password-stdin 2>&1 | redact_log > "$out/registry-login.log"
timeout --signal=TERM --kill-after=30s 1800s docker --config "$secrets/docker" pull "$image" 2>&1 | redact_log > "$out/image-pull.log"
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
  -v "$work/cache:/opt/nim/.cache" "$image" > "$out/container-id.txt"
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
