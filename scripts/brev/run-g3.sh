#!/usr/bin/env bash
set -euo pipefail

root=${1:?persisted workspace root required}
run=${2:?new g3-q/p-condition-policy-NNN run label required}
expected_commit=${3:?verified source commit required}
profile=${4:?qualification or pilot profile required}
condition=${5:?normal, closure, or burst condition required}
policy=${6:?fifo or eta policy required}
seed=${7:?seed from 0 through 9 required}

[[ "$profile" =~ ^(qualification|pilot)$ ]]
[[ "$condition" =~ ^(normal|closure|burst)$ ]]
[[ "$policy" =~ ^(fifo|eta)$ ]]
[[ "$seed" =~ ^[0-9]$ ]]
profile_code=${profile:0:1}
[[ "$run" =~ ^g3-(q|p)-(normal|closure|burst)-(fifo|eta)-[0-9]{3}$ ]]
[[ "$run" == "g3-$profile_code-$condition-$policy-"* ]]
[[ "$expected_commit" =~ ^[0-9a-f]{40}$ ]]

project="$root/project"
session="$project/runs/${run}-session"
output="$root/results/$run"
test "$(git -C "$project" rev-parse HEAD)" = "$expected_commit"
test -z "$(git -C "$project" status --porcelain)"
test ! -e "$session"
test ! -e "$output"
test -z "$(docker ps -aq --filter name=^/factory-ops-)"

isaac_image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
nav_image=$(docker image inspect factory-ops-nav2:jazzy-g1 --format '{{.Id}}')
run_id=$(python3 -c 'import secrets; print(secrets.token_hex(16))')
cache="$root/sdk-cache"
sim="factory-ops-$run-sim"
nav="factory-ops-$run-nav"
evidence="factory-ops-$run-evidence"
sampler_pid=
sim_log_pid=
created=()

cleanup() {
  for container in "${created[@]}"; do
    docker rm -f "$container" >/dev/null 2>&1 || true
  done
  for process_id in "$sampler_pid" "$sim_log_pid"; do
    if [ -n "$process_id" ]; then
      kill "$process_id" >/dev/null 2>&1 || true
      wait "$process_id" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT

(
  cd "$project"
  python3 -m factory_ops.g3_session \
    --output "runs/${run}-session" --profile "$profile" --condition "$condition" \
    --policy "$policy" --seed "$seed"
  python3 -m factory_ops.g3_session --output "runs/${run}-session" --verify \
    --profile "$profile" --condition "$condition" --policy "$policy" --seed "$seed"
)
session_sha=$(python3 - "$session/session.json" <<'PY'
from pathlib import Path
import hashlib,sys
print(hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest())
PY
)
scenario=$(python3 - "$session/session.json" <<'PY'
from pathlib import Path
import json,sys
value=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
scenario=value.get('scenario_id')
if not isinstance(scenario,str) or not scenario:
    raise SystemExit('session scenario_id is missing')
print(scenario)
PY
)

sudo -n install -d -m 0755 -o 1234 -g 1234 "$output"
python3 - "$root" "$run" "$run_id" "$expected_commit" "$isaac_image" "$nav_image" \
  "$profile" "$condition" "$policy" "$seed" "$scenario" <<'PY'
from datetime import datetime, timezone
from pathlib import Path
import hashlib,json,subprocess,sys

root,run=Path(sys.argv[1]),sys.argv[2]
project=root/'project'
session=project/'runs'/f'{run}-session'
paths=[
    'configs/factory-v3/layout.json','configs/factory-v3/traffic.json',
    'configs/factory-v3/g2.json','configs/factory-v3/g3-routing.json',
    'configs/factory-v3/g3-experiment.json','configs/factory-v3/g3-runtime.json',
    'configs/factory-v3/visual-assets.json','configs/factory-v3/render-profile.json',
    'configs/factory-v3/nav2-g2.yaml','factory_ops/drive_profile.py',
    'factory_ops/experiments.py','factory_ops/fleet.py','factory_ops/fleet_execution.py',
    'factory_ops/fleet_routing.py','factory_ops/footprint.py','factory_ops/g3_prepare.py',
    'factory_ops/g3_session.py','factory_ops/layout.py','factory_ops/motion.py',
    'factory_ops/motion_authorization.py','factory_ops/motion_validation.py',
    'factory_ops/navigation.py','factory_ops/route_reservation.py',
    'factory_ops/traffic.py','factory_ops/traffic_observation.py','factory_ops/usda.py',
    'factory_ops/visual_scene.py','factory_ops/visual_state.py','factory_ops/visual_evidence.py',
    'scripts/brev/run-g3.sh',
    'scripts/isaac/nav_scene.py','scripts/isaac/g2_scene.py','scripts/isaac/g3_scene.py',
    'scripts/isaac/factory_visuals.py',
    'scripts/ros/g2.launch.py','scripts/ros/g3_mission.py','scripts/ros/g3_evidence.py',
    'scripts/ros/action_services.py',
]
paths.extend(p.relative_to(project).as_posix() for p in sorted((project/'factory_ops/operations').glob('*.py')))
session_value=json.loads((session/'session.json').read_text(encoding='utf-8'))
record={
    'run_label':run,'run_id':sys.argv[3],'source_commit':sys.argv[4],
    'started_at_utc':datetime.now(timezone.utc).isoformat(),
    'isaac_image':sys.argv[5],'nav_image':sys.argv[6],
    'profile':sys.argv[7],'condition':sys.argv[8],'policy':sys.argv[9],
    'seed':int(sys.argv[10]),'scenario_id':sys.argv[11],
    'session':session_value,
    'session_json_sha256':hashlib.sha256((session/'session.json').read_bytes()).hexdigest(),
    'files':{p:hashlib.sha256((project/p).read_bytes()).hexdigest() for p in paths},
    'gpu':subprocess.check_output(
        ['nvidia-smi','--query-gpu=name,driver_version,memory.total',
         '--format=csv,noheader,nounits'],text=True
    ).strip(),
}
if (record['profile'],record['condition'],record['policy'],record['seed'],record['scenario_id']) != (
        session_value['profile'],session_value['condition'],session_value['policy'],
        session_value['seed'],session_value['scenario_id']):
    raise ValueError('session identity differs from requested run')
(root/'logs'/f'{run}-environment.json').write_text(
    json.dumps(record,indent=2,sort_keys=True)+'\n',encoding='utf-8'
)
PY

docker create --name "$sim" --gpus all --network host --ipc host --user 1234:1234 \
  --entrypoint /bin/bash --workdir /isaac-sim \
  -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 -e ROS_DOMAIN_ID=72 \
  -e FACTORY_OPS_G3_SESSION_SHA256="$session_sha" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$cache/cache/main:/isaac-sim/.cache:rw" \
  -v "$cache/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$cache/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$cache/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$cache/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$cache/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$cache/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$cache/hub:/var/cache/hub:rw" \
  -v "$project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$isaac_image" -lc "timeout --signal=TERM --kill-after=30s 5500s ./python.sh /workspace/project/scripts/isaac/g3_scene.py --session /workspace/project/runs/${run}-session --output /workspace/results/$run --run-id $run_id" >/dev/null
created+=("$sim")

docker create --name "$nav" --network host --ipc host --user 1234:1234 \
  --entrypoint /bin/bash -e HOME=/tmp -e ROS_DOMAIN_ID=72 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FACTORY_OPS_G3_SESSION_SHA256="$session_sha" \
  -v "$project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$nav_image" -lc "source /opt/ros/jazzy/setup.bash; timeout --signal=TERM --kill-after=30s 5500s python3 /workspace/project/scripts/ros/g3_mission.py --session /workspace/project/runs/${run}-session --output /workspace/results/$run --run-id $run_id" >/dev/null
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
if [ "$nav_status" -ne 0 ] && [ ! -f "$output/mission-control.json" ]; then
  sudo -n python3 - "$output/mission-control.json" "$run_id" "$scenario" <<'PY'
from pathlib import Path
import json,sys
path=Path(sys.argv[1]); temporary=path.with_suffix('.tmp')
temporary.write_text(json.dumps({
    'run_id':sys.argv[2],'scenario_id':sys.argv[3],'status':'failed',
    'reason':'ROS container exited before terminal control',
},sort_keys=True)+'\n',encoding='utf-8')
temporary.replace(path)
PY
fi
sim_status=$(docker wait "$sim")
wait "$sim_log_pid"
sim_log_status=$?
sim_log_pid=
set -e
printf '%s\n' "$sim_log_status" > "$root/logs/$run-sim-log-exit.txt"

docker logs "$sim" > "$root/logs/$run-sim.log" 2>&1
sudo -n install -m 0644 -o 1234 -g 1234 "$root/logs/$run-sim.log" "$output/isaac.log"
docker inspect "$sim" "$nav" --format '{{.Name}} {{json .State}}' > "$root/logs/$run-container-exits.jsonl"
sudo -n python3 - "$output" "$nav_status" "$sim_status" "$run_id" "$session/session.json" <<'PY'
from pathlib import Path
import json,sys

out=Path(sys.argv[1]); nav_status,sim_status=map(int,sys.argv[2:4])
session=json.loads(Path(sys.argv[5]).read_text(encoding='utf-8'))
record={
    'run_id':sys.argv[4],'scenario_id':session['scenario_id'],
    'profile':session['profile'],'condition':session['condition'],
    'policy':session['policy'],'seed':session['seed'],
    'nav_exit_code':nav_status,'sim_exit_code':sim_status,
}
for file in ('mission-result.json','g3-scene.json'):
    record[file]=json.loads((out/file).read_text(encoding='utf-8')) if (out/file).is_file() else None
(out/'pair-exit.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n',encoding='utf-8')
assert nav_status==sim_status==0, f'worker failure nav={nav_status},sim={sim_status}'
assert record['mission-result.json']['run_id']==record['g3-scene.json']['run_id']==sys.argv[4]
assert record['mission-result.json']['scenario_id']==record['g3-scene.json']['scenario_id']==session['scenario_id']
assert record['mission-result.json']['status']=='passed'
assert record['g3-scene.json']['status']=='validated'
print('FACTORY_OPS_G3_PAIR_EXIT=PASS')
PY

docker create --name "$evidence" --network none --user 1234:1234 \
  --entrypoint /bin/bash -e HOME=/tmp \
  -e FACTORY_OPS_G3_SESSION_SHA256="$session_sha" \
  -v "$project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$nav_image" -lc "source /opt/ros/jazzy/setup.bash; timeout --signal=TERM --kill-after=10s 600s python3 /workspace/project/scripts/ros/g3_evidence.py --session /workspace/project/runs/${run}-session --output /workspace/results/$run --run-id $run_id" >/dev/null
created+=("$evidence")
docker start -ai "$evidence" 2>&1 | tee "$root/logs/$run-validation.log"
