#!/usr/bin/env bash
set -euo pipefail

root=${1:?persisted workspace root required}
run=${2:?new g4-probe/render-NNN run label required}
expected_commit=${3:?verified source commit required}
[[ "$run" =~ ^g4-(probe|render)-[0-9]{3}$ ]]
[[ "$expected_commit" =~ ^[0-9a-f]{40}$ ]]
project="$root/project"
output="$root/results/$run"
name="factory-ops-$run"
cache="$root/sdk-cache"
test "$(git -C "$project" rev-parse HEAD)" = "$expected_commit"
test -z "$(git -C "$project" status --porcelain)"
test ! -e "$output"
test ! -e "$root/logs/$run-environment.json"
test -z "$(docker ps -aq --filter name=^/factory-ops-)"
image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
args=(--variant-seeds 0..11 --captures-per-scene 30)
if [[ "$run" == g4-probe-* ]]; then
  args=(--variant-seeds 0 --captures-per-scene 2 --probe)
fi
sampler_pid=
created=false
cleanup() {
  if [ "$created" = true ]; then
    docker rm -f "$name" >/dev/null 2>&1 || true
  fi
  if [ -n "$sampler_pid" ]; then
    kill "$sampler_pid" >/dev/null 2>&1 || true
    wait "$sampler_pid" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
python3 - "$root" "$run" "$expected_commit" "$image" <<'PY'
from datetime import datetime,timezone
from pathlib import Path
import hashlib,json,subprocess,sys
root,run=Path(sys.argv[1]),sys.argv[2]
project=root/'project'
paths=[
    'scripts/brev/run-g4.sh','scripts/isaac/g4_capture.py',
    'scripts/isaac/factory_visuals.py','scripts/isaac/g2_scene.py',
    'scripts/isaac/stage_smoke.py','factory_ops/visual_scene.py',
    'factory_ops/visual_state.py','factory_ops/usda.py','factory_ops/layout.py',
    'configs/factory-v3/layout.json','configs/factory-v3/g2-scenarios.json',
    'configs/factory-v3/dataset-profile.json','configs/factory-v3/render-profile.json',
    'configs/factory-v3/visual-assets.json',
]
record={
    'run_id':run,'source_commit':sys.argv[3],'isaac_image':sys.argv[4],
    'started_at_utc':datetime.now(timezone.utc).isoformat(),
    'scope':'diagnostic_unreviewed' if run.startswith('g4-probe-') else 'capture_pending_validation',
    'source_files':{p:hashlib.sha256((project/p).read_bytes()).hexdigest() for p in paths},
    'free_disk_bytes':__import__('shutil').disk_usage(root).free,
    'gpu':subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version,memory.total',
                                 '--format=csv,noheader,nounits'],text=True).strip(),
}
(root/'logs'/f'{run}-environment.json').write_text(json.dumps(record,indent=2)+'\n')
PY
docker create --name "$name" --gpus all --network host --ipc host --user 1234:1234 \
  --entrypoint /bin/bash --workdir /isaac-sim \
  -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 \
  -v "$cache/cache/main:/isaac-sim/.cache:rw" \
  -v "$cache/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$cache/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$cache/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$cache/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$cache/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$cache/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$cache/hub:/var/cache/hub:rw" \
  -v "$project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$image" -lc 'exec timeout --signal=TERM --kill-after=30s 7200s ./python.sh /workspace/project/scripts/isaac/g4_capture.py "$@"' \
  factory-ops-g4 --output "/workspace/results/$run" --run-id "$run" \
  --source-commit "$expected_commit" --layout /workspace/project/configs/factory-v3/layout.json \
  --resolution 1280x720 "${args[@]}" >/dev/null
created=true
docker inspect "$name" > "$root/logs/$run-container.json"
nvidia-smi --query-gpu=timestamp,name,memory.used,utilization.gpu --format=csv,noheader,nounits -l 1 \
  > "$root/logs/$run-gpu.csv" &
sampler_pid=$!
set +e
docker start --attach "$name" > "$root/logs/$run.log" 2>&1
attach_exit=$?
set -e
worker_exit=$(docker inspect "$name" --format '{{.State.ExitCode}}')
python3 - "$root" "$run" "$expected_commit" "$worker_exit" "$attach_exit" <<'PY'
from datetime import datetime,timezone
from pathlib import Path
import hashlib,json,sys
root,run=Path(sys.argv[1]),sys.argv[2]
record={'run_id':run,'source_commit':sys.argv[3],'worker_exit_code':int(sys.argv[4]),
        'attach_exit_code':int(sys.argv[5]),'finished_at_utc':datetime.now(timezone.utc).isoformat()}
output=root/'results'/run
for name in ('dataset.json','failed.json','gpu-resource-audit.json'):
    path=output/name
    if path.is_file():
        record[name]={'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
(root/'logs'/f'{run}-exit.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
PY
test "$attach_exit" = 0
test "$worker_exit" = 0
test -f "$output/dataset.json"
printf '%s\n' 'FACTORY_OPS_G4_CAPTURE_EXIT=SUCCESS_UNREVIEWED'
