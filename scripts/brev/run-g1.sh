#!/usr/bin/env bash
set -euo pipefail
root=${1:?persisted workspace root required}
run=${2:?new g1-NNN run label required}
expected_commit=${3:?verified source commit required}
[[ "$run" =~ ^g1-[0-9]{3}$ ]]
[[ "$expected_commit" =~ ^[0-9a-f]{40}$ ]]
test "$(git -C "$root/project" rev-parse HEAD)" = "$expected_commit"
test -z "$(git -C "$root/project" status --porcelain)"
test ! -e "$root/results/$run"
test -z "$(docker ps -aq --filter name=^/factory-ops-g1-)"
isaac_image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
nav_image=$(docker image inspect factory-ops-nav2:jazzy-g1 --format '{{.Id}}')
run_id=$(python3 -c 'import secrets; print(secrets.token_hex(16))')
cache="$root/sdk-cache"
sim="factory-ops-$run-sim"
nav="factory-ops-$run-nav"
sampler_pid=
sim_log_pid=
created=()
cleanup() {
  for container in "${created[@]}"; do docker rm -f "$container" >/dev/null 2>&1 || true; done
  for process_id in "$sampler_pid" "$sim_log_pid"; do
    if [ -n "$process_id" ]; then kill "$process_id" >/dev/null 2>&1 || true; wait "$process_id" >/dev/null 2>&1 || true; fi
  done
}
trap cleanup EXIT
sudo -n install -d -m 0755 -o 1234 -g 1234 "$root/results/$run"
python3 - "$root" "$run" "$run_id" "$expected_commit" "$isaac_image" "$nav_image" <<'PY'
from datetime import datetime, timezone
from pathlib import Path
import hashlib,json,subprocess,sys
root,run=Path(sys.argv[1]),sys.argv[2]
paths=['configs/factory-v3/layout.json','configs/factory-v3/g1.json','configs/factory-v3/nav2-g1.yaml',
       'factory_ops/motion.py','factory_ops/navigation.py','scripts/isaac/nav_scene.py',
       'scripts/ros/g1_mission.py','scripts/ros/g1.launch.py','runs/local/scene/factory-v3.usda',
       'runs/local/maps/map.pgm','runs/local/maps/keepout.pgm']
record={'run_label':run,'run_id':sys.argv[3],'source_commit':sys.argv[4],
        'started_at_utc':datetime.now(timezone.utc).isoformat(),
        'isaac_image':sys.argv[5],'nav_image':sys.argv[6],
        'files':{p:hashlib.sha256((root/'project'/p).read_bytes()).hexdigest() for p in paths},
        'gpu':subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version,memory.total','--format=csv,noheader,nounits'],text=True).strip()}
(root/'logs'/f'{run}-environment.json').write_text(json.dumps(record,indent=2)+'\n')
PY
docker create --name "$sim" --gpus all --network host --ipc host --user 1234:1234 \
  --entrypoint /bin/bash --workdir /isaac-sim \
  -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 -e ROS_DOMAIN_ID=71 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$cache/cache/main:/isaac-sim/.cache:rw" \
  -v "$cache/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$cache/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$cache/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$cache/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$cache/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$cache/pkg:/isaac-sim/.local/share/ov/pkg:rw" -v "$cache/hub:/var/cache/hub:rw" \
  -v "$root/project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$isaac_image" -lc "timeout --signal=TERM --kill-after=30s 2500s ./python.sh /workspace/project/scripts/isaac/nav_scene.py --scene /workspace/project/runs/local/scene/factory-v3.usda --layout /workspace/project/configs/factory-v3/layout.json --config /workspace/project/configs/factory-v3/g1.json --output /workspace/results/$run --run-id $run_id" >/dev/null
created+=("$sim")
docker create --name "$nav" --network host --ipc host --user 1234:1234 \
  --entrypoint /bin/bash -e HOME=/tmp -e ROS_DOMAIN_ID=71 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$root/project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$nav_image" -lc "source /opt/ros/jazzy/setup.bash; timeout --signal=TERM --kill-after=30s 2500s python3 /workspace/project/scripts/ros/g1_mission.py --output /workspace/results/$run --run-id $run_id" >/dev/null
created+=("$nav")
docker inspect "$sim" "$nav" > "$root/logs/$run-containers.json"
nvidia-smi --query-gpu=timestamp,name,memory.used,utilization.gpu --format=csv,noheader,nounits -l 1 > "$root/logs/$run-gpu.csv" &
sampler_pid=$!
docker start "$sim" >/dev/null
docker logs -f "$sim" > "$root/logs/$run-sim.log" 2>&1 &
sim_log_pid=$!
set +e
docker start -ai "$nav" 2>&1 | tee "$root/logs/$run-nav.log"
nav_status=${PIPESTATUS[0]}
if [ "$nav_status" -ne 0 ] && [ ! -f "$root/results/$run/mission-control.json" ]; then
  sudo -n python3 - "$root/results/$run/mission-control.json" "$run_id" <<'PY'
from pathlib import Path
import json,sys
path=Path(sys.argv[1]); temporary=path.with_suffix('.tmp')
temporary.write_text(json.dumps({'run_id':sys.argv[2],'status':'failed','reason':'ROS container exited before terminal control'}))
temporary.replace(path)
PY
fi
sim_status=$(docker wait "$sim")
set -e
docker logs "$sim" > "$root/logs/$run-sim.log" 2>&1
docker inspect "$sim" "$nav" --format '{{.Name}} {{json .State}}' > "$root/logs/$run-container-exits.jsonl"
sudo -n python3 - "$root/results/$run" "$nav_status" "$sim_status" "$run_id" <<'PY'
from pathlib import Path
import json,sys
out=Path(sys.argv[1]); nav_status,sim_status=map(int,sys.argv[2:4])
record={'run_id':sys.argv[4],'nav_exit_code':nav_status,'sim_exit_code':sim_status}
for file in ['mission-result.json','g1-scene.json']:
    record[file]=json.loads((out/file).read_text()) if (out/file).exists() else None
(out/'pair-exit.json').write_text(json.dumps(record,indent=2)+'\n')
assert nav_status==sim_status==0, f'worker failure nav={nav_status},sim={sim_status}'
assert record['mission-result.json']['run_id']==record['g1-scene.json']['run_id']==sys.argv[4]
assert record['mission-result.json']['status']=='passed'
assert record['g1-scene.json']['status']=='validated'
print('FACTORY_OPS_G1_PAIR_EXIT=PASS')
PY
