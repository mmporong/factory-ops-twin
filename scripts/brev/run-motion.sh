#!/usr/bin/env bash
set -euo pipefail
root=${1:?persisted workspace root required}
run=${2:?new motion-NNN run label required}
expected_commit=${3:?verified source commit required}
[[ "$run" =~ ^motion-[0-9]{3}$ ]]
[[ "$expected_commit" =~ ^[0-9a-f]{40}$ ]]
test "$(git -C "$root/project" rev-parse HEAD)" = "$expected_commit"
test -z "$(git -C "$root/project" status --porcelain)"
test ! -e "$root/results/$run"
test -z "$(docker ps -aq --filter name=^/factory-ops-)"
image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
run_id=$(python3 -c 'import secrets; print(secrets.token_hex(16))')
cache="$root/sdk-cache"
container="factory-ops-$run"
sampler_pid=
cleanup() {
  docker rm -f "$container" >/dev/null 2>&1 || true
  docker rm -f "$container-validation" >/dev/null 2>&1 || true
  if [ -n "$sampler_pid" ]; then
    kill "$sampler_pid" >/dev/null 2>&1 || true
    wait "$sampler_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
sudo -n install -d -m 0755 -o 1234 -g 1234 "$root/results"
python3 - "$root" "$run" "$run_id" "$expected_commit" "$image" <<'PY'
from datetime import datetime, timezone
from pathlib import Path
import hashlib,json,subprocess,sys
root,run=Path(sys.argv[1]),sys.argv[2]
paths=['configs/factory-v3/motion-qualification.json','factory_ops/drive_profile.py',
       'factory_ops/motion.py','factory_ops/motion_validation.py','factory_ops/motion_transients.py',
       'scripts/isaac/motion_qualification.py',
       'scripts/isaac/motion_evidence.py','scripts/isaac/nav_scene.py',
       'scripts/isaac/robot_qualification.py','scripts/isaac/stage_smoke.py']
record={'run_label':run,'run_id':sys.argv[3],'source_commit':sys.argv[4],
        'started_at_utc':datetime.now(timezone.utc).isoformat(),'isaac_image':sys.argv[5],
        'files':{p:hashlib.sha256((root/'project'/p).read_bytes()).hexdigest() for p in paths},
        'gpu':subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version,memory.total',
                                     '--format=csv,noheader,nounits'],text=True).strip()}
(root/'logs'/f'{run}-environment.json').write_text(json.dumps(record,indent=2)+'\n')
PY
docker create --name "$container" --gpus all --user 1234:1234 --entrypoint /bin/bash --workdir /isaac-sim \
  -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 \
  -v "$cache/cache/main:/isaac-sim/.cache:rw" -v "$cache/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$cache/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$cache/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$cache/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$cache/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$cache/pkg:/isaac-sim/.local/share/ov/pkg:rw" -v "$cache/hub:/var/cache/hub:rw" \
  -v "$root/project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$image" -lc "timeout --signal=TERM --kill-after=30s 900s ./python.sh /workspace/project/scripts/isaac/motion_qualification.py --config /workspace/project/configs/factory-v3/motion-qualification.json --output /workspace/results/$run --run-id $run_id" >/dev/null
docker inspect "$container" > "$root/logs/$run-container.json"
nvidia-smi --query-gpu=timestamp,name,memory.used,utilization.gpu --format=csv,noheader,nounits -l 1 > "$root/logs/$run-gpu.csv" &
sampler_pid=$!
set +e
docker start -ai "$container" > "$root/logs/$run-sim.log" 2>&1
worker_status=$?
set -e
docker inspect "$container" --format '{{json .State}}' > "$root/logs/$run-exit.json"
sudo -n python3 - "$root/results/$run" "$worker_status" "$run_id" <<'PY'
from pathlib import Path
import json,sys
output=Path(sys.argv[1]); status=int(sys.argv[2]); run_id=sys.argv[3]
scene=json.loads((output/'motion-scene.json').read_text()) if (output/'motion-scene.json').is_file() else None
record={'run_id':run_id,'worker_exit_code':status,'motion-scene.json':scene}
(output/'worker-exit.json').write_text(json.dumps(record,indent=2)+'\n')
assert status==0 and isinstance(scene,dict), f'motion worker failed ({status})'
assert scene['run_id']==run_id and scene['status']=='validated', 'motion worker evidence not validated'
print('FACTORY_OPS_MOTION_WORKER_EXIT=PASS')
PY
docker run --rm --name "$container-validation" --user 1234:1234 --entrypoint /bin/bash --workdir /isaac-sim \
  -v "$root/project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$image" -lc "timeout --signal=TERM --kill-after=10s 120s ./python.sh /workspace/project/scripts/isaac/motion_evidence.py --output /workspace/results/$run --config /workspace/project/configs/factory-v3/motion-qualification.json --run-id $run_id"
