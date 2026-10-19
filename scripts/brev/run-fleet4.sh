#!/usr/bin/env bash
# One owned development pair. The host preserves the stage and stops Brev.
set -euo pipefail
stage=$1
run_id=$2
commit=$3
nav_image=$4
attestation=$5
attestation_sha=$6
instance_root="$HOME/workspace/factory-ops-g0"
[[ "$(dirname "$stage")" == "$instance_root" ]]
run=$(basename "$stage")
[[ "$run" =~ ^fleet4-transport-(normal|closure)-[0-9]{3}$ ]]
[[ "$run_id" =~ ^[0-9a-f]{32}$ && "$commit" =~ ^[0-9a-f]{40}$ ]]
[[ "$nav_image" =~ ^sha256:[0-9a-f]{64}$ && "$attestation_sha" =~ ^[0-9a-f]{64}$ ]]
[[ "$attestation" == "$instance_root"/build-attestations/*/build-attestation.json ]]
nav_id= sim_id= gpu_pid=
cleanup() {
  rc=$?
  trap - EXIT
  set +e
  if [ -n "$gpu_pid" ]; then kill "$gpu_pid"; wait "$gpu_pid"; fi
  for label in nav sim; do
    cid=$nav_id; [ "$label" = nav ] || cid=$sim_id
    [ -n "$cid" ] || continue
    if [ "$(docker inspect "$cid" --format '{{.State.Running}}' 2>/dev/null)" = true ]; then
      docker stop -t 20 "$cid" > "$stage/$label-stop.log" 2>&1
      printf '%s\n' "$?" > "$stage/$label-stop-exit.txt"
    fi
    docker logs "$cid" > "$stage/$label.log" 2>&1
    docker inspect "$cid" > "$stage/$label-inspect.json"
    docker rm "$cid" > "$stage/$label-remove.log" 2>&1
    printf '%s\n' "$?" > "$stage/$label-remove-exit.txt"
  done
  python3 - "$stage" "$run_id" "$commit" "$rc" "$nav_id" "$sim_id" <<'PY'
import datetime,json,pathlib,sys
root=pathlib.Path(sys.argv[1]); data={'run_id':sys.argv[2],'source_commit':sys.argv[3],
 'original_runner_exit_code':int(sys.argv[4]),'finished_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'container_ids':{'nav':sys.argv[5],'sim':sys.argv[6]},'cleanup_errors':[]}
for label,cid in (('nav',sys.argv[5]),('sim',sys.argv[6])):
 p=root/(label+'-inspect.json')
 data[label]=None
 if cid:
  try:
   data[label]=json.loads(p.read_text())[0]['State']
   if data[label]['Running']:data['cleanup_errors'].append(label+' still running after stop')
  except (OSError,ValueError,KeyError,IndexError) as error:data['cleanup_errors'].append(label+' inspection failed: '+str(error))
 for suffix in ('stop','remove'):
  status=root/(label+'-'+suffix+'-exit.txt')
  if status.is_file() and status.read_text().strip()!='0':data['cleanup_errors'].append(label+' '+suffix+' failed')
data['cleanup_passed']=not data['cleanup_errors']
data['runner_exit_code']=int(sys.argv[4]) or (0 if data['cleanup_passed'] else 1)
with (root/'runner-exit.json').open('x') as f:json.dump(data,f,indent=2)
raise SystemExit(0 if data['cleanup_passed'] else 1)
PY
  seal_status=$?
  if [ "$seal_status" -ne 0 ] && [ "$rc" -eq 0 ]; then rc=$seal_status; fi
  exit "$rc"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
test ! -e "$stage/abort-requested"
test -f "$attestation"
test ! -L "$attestation"
test "$(sha256sum "$attestation" | cut -d ' ' -f 1)" = "$attestation_sha"
test "$(docker image inspect "$nav_image" --format '{{.Id}}')" = "$nav_image"
isaac_image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
[[ "$isaac_image" == nvcr.io/nvidia/isaac-sim@sha256:* ]]
project="$stage/source"
inputs="$stage/inputs"
output="$stage/results/run"
test -d "$project"
test -d "$output/provenance"
test ! -e "$stage/runner-exit.json"
test -z "$(docker ps -aq --filter 'name=^/factory-ops-')"
docker run --rm --network none --user "$(id -u):$(id -g)" --entrypoint python3 \
  -e PYTHONPATH=/workspace/project -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$project:/workspace/project:ro" -v "$inputs:/workspace/fleet4-inputs:ro" \
  "$nav_image" -m factory_ops.fleet_transport --bundle /workspace/fleet4-inputs --verify > "$stage/input-verification.json"
wall_limit=$(python3 -c 'import json,sys; print(int(json.load(open(sys.argv[1]))["max_wall_time_s"]))' "$inputs/config.json")
[[ "$wall_limit" =~ ^[0-9]+$ && "$wall_limit" -le 10800 ]]
cache="$instance_root/sdk-cache"
mounts=()
for mapping in 'cache/main:.cache' 'cache/computecache:.nv/ComputeCache' 'cache/kit:kit/cache' 'logs:.nvidia-omniverse/logs' 'config:.nvidia-omniverse/config' 'data:.local/share/ov/data' 'pkg:.local/share/ov/pkg'; do
  mounts+=(-v "$cache/${mapping%%:*}:/isaac-sim/${mapping#*:}:rw")
done
mounts+=(-v "$cache/hub:/var/cache/hub:rw")
sudo -n chown -R 1234:1234 "$stage/results"
test ! -e "$stage/abort-requested"
nav_id=$(docker create --name "factory-ops-$run-nav" --network host --ipc host --user 1234:1234 \
  --label factory_ops.run_id="$run_id" --label factory_ops.run_label="$run" \
  --entrypoint /bin/bash -e HOME=/tmp -e ROS_DOMAIN_ID=72 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e PYTHONDONTWRITEBYTECODE=1 -e FACTORY_OPS_G3_SOURCE_COMMIT="$commit" \
  -e FACTORY_OPS_G3_BUILD_ATTESTATION_PATH=/workspace/fleet4-build/build-attestation.json \
  -e FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256="$attestation_sha" \
  -v "$project:/workspace/project:ro" -v "$inputs:/workspace/fleet4-inputs:ro" \
  -v "$stage/results:/workspace/fleet4-results:rw" -v "$(dirname "$attestation"):/workspace/fleet4-build:ro" \
  "$nav_image" -lc "source /opt/ros/jazzy/setup.bash; timeout --signal=TERM --kill-after=30s ${wall_limit}s python3 /workspace/project/scripts/ros/fleet4_mission.py --bundle /workspace/fleet4-inputs --output /workspace/fleet4-results/run --run-id $run_id --dashboard")
metadata_sha=$(sha256sum "$output/provenance/fleet-source.json" | cut -d ' ' -f 1)
test ! -e "$stage/abort-requested"
sim_id=$(docker create --name "factory-ops-$run-sim" --gpus all --network host --ipc host --user 1234:1234 \
  --label factory_ops.run_id="$run_id" --label factory_ops.run_label="$run" \
  --entrypoint /bin/bash --workdir /isaac-sim -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 \
  -e ROS_DOMAIN_ID=72 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e PYTHONDONTWRITEBYTECODE=1 \
  -e FACTORY_OPS_G3_SOURCE_COMMIT="$commit" -e FACTORY_OPS_FLEET4_PROVENANCE_SHA256="$metadata_sha" \
  "${mounts[@]}" -v "$project:/workspace/project:ro" -v "$inputs:/workspace/fleet4-inputs:ro" \
  -v "$stage/results:/workspace/fleet4-results:rw" "$isaac_image" \
  -lc "timeout --signal=TERM --kill-after=30s ${wall_limit}s ./python.sh /workspace/project/scripts/isaac/fleet4_transport_scene.py --bundle /workspace/fleet4-inputs --output /workspace/fleet4-results/run --run-id $run_id")
printf '%s\n' "$nav_id" > "$stage/nav.id"
printf '%s\n' "$sim_id" > "$stage/sim.id"
python3 "$project/scripts/brev/g3_gpu_sampler.py" --interval-s 8 --raw-path "$stage/gpu-raw.jsonl" > "$stage/gpu.csv" &
gpu_pid=$!
test ! -e "$stage/abort-requested"
docker start "$nav_id" >/dev/null
marker_deadline=$((SECONDS + 180))
while [ ! -f "$output/fleet-transport-verified.json" ]; do
  test "$(docker inspect "$nav_id" --format '{{.State.Running}}')" = true
  test "$SECONDS" -lt "$marker_deadline"
  sleep 1
done
test ! -e "$stage/abort-requested"
docker start "$sim_id" >/dev/null
echo "FACTORY_OPS_FLEET4_PAIR_STARTED $run_id"
deadline=$((SECONDS + wall_limit + 60))
sim_exit_seen=0
while :; do
  test ! -e "$stage/abort-requested"
  nav_running=$(docker inspect "$nav_id" --format '{{.State.Running}}')
  sim_running=$(docker inspect "$sim_id" --format '{{.State.Running}}')
  [ "$nav_running" = true ] || break
  if [ "$sim_running" != true ]; then
    if [ "$sim_exit_seen" -eq 0 ]; then sim_exit_seen=$SECONDS; fi
    if [ "$(docker inspect "$sim_id" --format '{{.State.ExitCode}}')" -ne 0 ] || [ "$SECONDS" -gt $((sim_exit_seen + 60)) ]; then
      echo 'FACTORY_OPS_FLEET4_SIM_EXITED_BEFORE_NAV_TERMINAL' >&2
      exit 1
    fi
  fi
  test "$SECONDS" -lt "$deadline"
  artifact_bytes=$(du -sb "$stage/results" | cut -f 1)
  test "$artifact_bytes" -lt 4294967296
  python3 -c 'import shutil,sys; assert shutil.disk_usage(sys.argv[1]).free > 6000000000' "$stage"
  sleep 5
done
nav_status=$(docker wait "$nav_id")
if [ "$nav_status" -ne 0 ]; then exit 1; fi
sim_status=$(docker wait "$sim_id")
test "$sim_status" -eq 0
echo 'FACTORY_OPS_FLEET4_PAIR_CLOSED'
