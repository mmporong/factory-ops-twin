#!/usr/bin/env bash
set -euo pipefail

# Keep the extracted Git source unchanged while creating and verifying sessions.
export PYTHONDONTWRITEBYTECODE=1

root=${1:?persisted workspace root required}
run=${2:?new g3-q/p/m-condition-policy-NNN run label required}
expected_commit=${3:?verified source commit required}
profile=${4:?qualification, pilot, or primary profile required}
condition=${5:?normal, closure, or burst condition required}
policy=${6:?fifo or eta policy required}
seed=${7:?qualification/primary seed 0..9 or pilot seed 100 required}
factory_profile=${FACTORY_OPS_G3_FACTORY_PROFILE:-factory-v3}
[[ "$factory_profile" =~ ^factory-v[34]$ ]]
primary_inputs=${8:-}
primary_inputs_sha=${9:-}
billing=${10:-}
build_attestation=${FACTORY_OPS_G3_BUILD_ATTESTATION:?externally pinned G3 build attestation required}
build_attestation_sha=${FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256:?external build attestation SHA-256 required}
build_receipt_sha=${FACTORY_OPS_G3_BUILD_RECEIPT_SHA256:?external build receipt SHA-256 required}

[[ "$profile" =~ ^(qualification|pilot|primary)$ ]]
[[ "$condition" =~ ^(normal|closure|burst)$ ]]
[[ "$policy" =~ ^(fifo|eta)$ ]]
case "$profile" in
  qualification) profile_code=q; [[ "$seed" =~ ^[0-9]$ ]] ;;
  pilot) profile_code=p; [[ "$seed" = 100 ]] ;;
  primary)
    profile_code=m
    [[ "$seed" =~ ^[0-9]$ ]]
    [[ "$primary_inputs" = /* && "$primary_inputs_sha" =~ ^[0-9a-f]{64}$ ]]
    test -f "$primary_inputs"
    test ! -L "$primary_inputs"
    test "$(sha256sum "$primary_inputs" | cut -d ' ' -f 1)" = "$primary_inputs_sha"
    ;;
esac
if [ "$profile" != primary ]; then
  test -z "$primary_inputs"
  test -z "$primary_inputs_sha"
fi
if [ "$profile" != qualification ]; then
  [[ "$billing" = /* ]]
  test -f "$billing"
  test ! -L "$billing"
fi
[[ "$run" =~ ^g3-(q|p|m)-(normal|closure|burst)-(fifo|eta)-[0-9]{3}$ ]]
[[ "$run" == "g3-$profile_code-$condition-$policy-"* ]]
[[ "$expected_commit" =~ ^[0-9a-f]{40}$ ]]

repository="$root/project"
project="$root/source-snapshots/$run"
archive="$root/source-snapshots/$run.tar"
commit_object="$root/source-snapshots/$run.commit"
session="$project/runs/${run}-session"
output="$root/results/$run"
test "$(git -C "$repository" rev-parse HEAD)" = "$expected_commit"
test -z "$(git -C "$repository" status --porcelain)"
test ! -e "$project"
test ! -e "$archive"
test ! -e "$commit_object"
test ! -e "$output"
test -z "$(docker ps -aq --filter name=^/factory-ops-)"
mkdir -p "$root/source-snapshots"
git -C "$repository" archive --format=tar --output="$archive" "$expected_commit"
git -C "$repository" cat-file commit "$expected_commit" > "$commit_object"
mkdir "$project"
tar --no-same-owner -xf "$archive" -C "$project"

isaac_image=$(docker image inspect nvcr.io/nvidia/isaac-sim:6.2.0 --format '{{index .RepoDigests 0}}')
[[ "$build_attestation" = /* && "$build_attestation_sha" =~ ^[0-9a-f]{64}$ ]]
[[ "$build_receipt_sha" =~ ^[0-9a-f]{64}$ ]]
test -f "$build_attestation"
test ! -L "$build_attestation"
build_bundle_source=$(dirname "$build_attestation")
build_bundle="$root/source-snapshots/$run-build-bundle"
test ! -e "$build_bundle"
mkdir "$build_bundle"
for name in build-attestation.json build-receipt.json commands.jsonl built-image.iid \
  base-image-inspect.json built-image-inspect.json installed-measurement.json; do
  test -f "$build_bundle_source/$name"
  test ! -L "$build_bundle_source/$name"
  install -m 0444 "$build_bundle_source/$name" "$build_bundle/$name"
done
attestation_copy="$build_bundle/build-attestation.json"
test "$(sha256sum "$attestation_copy" | cut -d ' ' -f 1)" = "$build_attestation_sha"
build_attestation="$attestation_copy"
mapfile -t attested_images < <(python3 "$project/scripts/brev/g3_build_attestation.py" references \
  --attestation "$build_attestation" --expected-sha256 "$build_attestation_sha")
test "${#attested_images[@]}" = 2
base_inspect="$root/source-snapshots/$run-current-base-image-inspect.json"
built_inspect="$root/source-snapshots/$run-current-built-image-inspect.json"
test ! -e "$base_inspect" && test ! -e "$built_inspect"
docker image inspect "${attested_images[0]}" > "$base_inspect"
docker image inspect "${attested_images[1]}" > "$built_inspect"
preflight_measurement="$root/source-snapshots/$run-preflight-measurement"
validation_json=$(python3 "$project/scripts/brev/g3_build_attestation.py" validate \
  --attestation "$build_attestation" --expected-sha256 "$build_attestation_sha" \
  --source-root "$project" --source-commit "$expected_commit" --run-label "$run" \
  --base-inspect "$base_inspect" --built-inspect "$built_inspect" --source-archive "$archive" \
  --build-receipt "$build_bundle/build-receipt.json" --build-commands "$build_bundle/commands.jsonl" \
  --expected-build-receipt-sha256 "$build_receipt_sha" \
  --build-iidfile "$build_bundle/built-image.iid" \
  --build-base-inspect "$build_bundle/base-image-inspect.json" \
  --build-built-inspect "$build_bundle/built-image-inspect.json" \
  --build-installed-measurement "$build_bundle/installed-measurement.json" \
  --measurement-dir "$preflight_measurement")
nav_image=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["built_image_id"])' <<<"$validation_json")
preflight_measurement_sha=$(python3 -c 'import json,sys; print(json.load(sys.stdin)["preflight_measurement_sha256"])' <<<"$validation_json")
[[ "$nav_image" =~ ^sha256:[0-9a-f]{64}$ ]]
[[ "$preflight_measurement_sha" =~ ^[0-9a-f]{64}$ ]]
run_id=$(python3 -c 'import secrets; print(secrets.token_hex(16))')
cache="$root/sdk-cache"
sim="factory-ops-$run-sim"
nav="factory-ops-$run-nav"
evidence="factory-ops-$run-evidence"
resources="$root/resources/$run"
pack_program="$project/scripts/brev/g3_resource_pack.py"
sampler_pid=
sim_log_pid=
nav_log_pid=
created=()
resources_started=false
resources_sealed=false
evidence_status=not_started

finalize_resources() {
  if [ "$resources_started" != true ] || [ ! -d "$output" ]; then return; fi
  for phase in workers-finished validation-finished; do
    if [ ! -e "$resources/$phase.json" ]; then
      sudo -n python3 "$pack_program" "$phase" --root "$root" --run-label "$run" --run-id "$run_id"
    fi
  done
  sudo -n python3 "$pack_program" archive --root "$root" --run-label "$run" --run-id "$run_id"
  if [ -f "$resources/attempt-start.json" ]; then
    mapfile -t times < <(python3 - "$resources/archive-record.json" <<'PY'
from pathlib import Path
import json,sys
value=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
for key in ('workers_finished_at_utc','validation_finished_at_utc','archive_finished_at_utc'):
    print(value[key])
PY
)
    test "${#times[@]}" = 3
    finalize_args=(finalize --start "$resources/attempt-start.json" --output "$resources/attempt-receipt.json"
      --evidence-root "$root" --workers-finished-at-utc "${times[0]}"
      --validation-finished-at-utc "${times[1]}" --archive-finished-at-utc "${times[2]}"
      --output-manifest "$resources/output-manifest.json" --archive "$resources/output.tar.gz" --disk "$resources/disk.json")
    if [ -s "$root/logs/$run-gpu.csv" ] && [ -s "$root/logs/$run-gpu-raw.jsonl" ] && [ -s "$resources/gpu-sampler-status.json" ]; then
      finalize_args+=(--gpu-csv "$root/logs/$run-gpu.csv" --gpu-raw "$root/logs/$run-gpu-raw.jsonl"
        --gpu-sampler-status "$resources/gpu-sampler-status.json")
    else
      # Partial sampler originals remain in logs; they cannot become collected measurements.
      evidence_status=gpu_resource_collection_incomplete
    fi
    for specification in "scene-summary:$output/g3-scene.json" \
                         "report:$output/g3-evidence.json" "mission:$output/mission-result.json"; do
      flag=${specification%%:*}; path=${specification#*:}
      if [ -s "$path" ]; then finalize_args+=("--$flag" "$path"); fi
    done
    if [ "$evidence_status" != 0 ]; then
      finalize_args+=(--collection-failure-reason "closed physical evidence validation or resource collection did not complete successfully")
    fi
    sudo -n env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$project" python3 -m factory_ops.g3_supervisor "${finalize_args[@]}"
  fi
  resources_sealed=true
}

cleanup() {
  original_status=$?
  trap - EXIT
  for container in "${created[@]}"; do
    # Preserve remaining container stdout outside the frozen output artifact set.
    cleanup_log="$root/logs/$container-cleanup.log"
    if [ ! -e "$cleanup_log" ]; then docker logs "$container" > "$cleanup_log" 2>&1 || true; fi
    docker rm -f "$container" >/dev/null 2>&1 || true
  done
  for process_id in "$sampler_pid" "$sim_log_pid" "$nav_log_pid"; do
    if [ -n "$process_id" ]; then
      kill "$process_id" >/dev/null 2>&1 || true
      wait "$process_id" >/dev/null 2>&1 || true
    fi
  done
  if [ "$resources_sealed" != true ]; then
    if ! finalize_resources > "$root/logs/$run-resource-finalize.log" 2>&1; then
      printf '%s\n' 'Resource finalization failed; preserve this attempt for diagnosis' >&2
      original_status=1
    fi
  fi
  exit "$original_status"
}
trap cleanup EXIT
sudo -n python3 "$pack_program" start --root "$root" --run-label "$run" --run-id "$run_id"
resources_started=true

if [ "$profile" = primary ]; then
  # ROS/CDR gate verification runs before GPU physics starts.
  sudo -n install -d -m 0755 -o 1234 -g 1234 "$project/runs"
  docker run --rm --name "factory-ops-$run-session" --network none --user 1234:1234 \
    --entrypoint /bin/bash --workdir /workspace/project -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 \
    -v "$root:$root:ro" -v "$project:/workspace/project:rw" \
    "$nav_image" -lc 'source /opt/ros/jazzy/setup.bash; python3 -m factory_ops.g3_session "$@"' \
    factory-ops-g3-session --config-dir "/workspace/project/configs/$factory_profile" \
    --output "runs/${run}-session" --profile "$profile" \
    --condition "$condition" --policy "$policy" --seed "$seed" \
    --primary-inputs "$primary_inputs" --primary-inputs-sha256 "$primary_inputs_sha"
else (
  cd "$project"
  python3 -m factory_ops.g3_session \
    --config-dir "configs/$factory_profile" \
    --output "runs/${run}-session" --profile "$profile" --condition "$condition" \
    --policy "$policy" --seed "$seed"
  python3 -m factory_ops.g3_session --output "runs/${run}-session" --verify \
    --config-dir "configs/$factory_profile" \
    --profile "$profile" --condition "$condition" --policy "$policy" --seed "$seed"
)
fi
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
  "$profile" "$condition" "$policy" "$seed" "$scenario" "$primary_inputs" "$primary_inputs_sha" \
  "$build_attestation" "$build_attestation_sha" "$build_receipt_sha" "$preflight_measurement_sha" \
  "$build_bundle" "$base_inspect" "$built_inspect" "$preflight_measurement" <<'PY'
from datetime import datetime, timezone
from pathlib import Path
import hashlib,json,subprocess,sys

root,run=Path(sys.argv[1]),sys.argv[2]
project=root/'source-snapshots'/run
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
    'scripts/brev/run-g3.sh','scripts/ros/Dockerfile','scripts/ros/Dockerfile.g3',
    'scripts/brev/g3_build_attestation.py',
    'scripts/brev/g3_resource_pack.py',
    'scripts/brev/g3_gpu_sampler.py',
    'scripts/brev/g3_pair_wait.py',
    'scripts/isaac/nav_scene.py','scripts/isaac/g2_scene.py','scripts/isaac/g3_scene.py',
    'scripts/isaac/factory_visuals.py',
    'scripts/ros/g2.launch.py','scripts/ros/g3_mission.py','scripts/ros/g3_evidence.py',
    'scripts/ros/action_services.py',
]
paths.extend(p.relative_to(project).as_posix() for p in sorted((project/'factory_ops/operations').glob('*.py')))
paths=sorted(set(paths) | {
    p.relative_to(project).as_posix()
    for directory,patterns in (
        ('factory_ops',('*.py',)),('scripts/isaac',('*.py',)),('scripts/ros',('*.py',)),
        ('ros_ws/src/factory_ops_goal_checker',('*',)),
        ('configs',('*.json','*.yaml')))
    for pattern in patterns for p in (project/directory).rglob(pattern)
    if p.is_file() and not p.is_symlink()
})
source_files={p:hashlib.sha256((project/p).read_bytes()).hexdigest() for p in paths}
build_sources={
    'provenance/build/build-attestation.json':Path(sys.argv[18])/'build-attestation.json',
    'provenance/build/build-receipt.json':Path(sys.argv[18])/'build-receipt.json',
    'provenance/build/commands.jsonl':Path(sys.argv[18])/'commands.jsonl',
    'provenance/build/built-image.iid':Path(sys.argv[18])/'built-image.iid',
    'provenance/build/base-image-inspect.json':Path(sys.argv[18])/'base-image-inspect.json',
    'provenance/build/built-image-inspect.json':Path(sys.argv[18])/'built-image-inspect.json',
    'provenance/build/installed-measurement.json':Path(sys.argv[18])/'installed-measurement.json',
    'provenance/build/current-base-image-inspect.json':Path(sys.argv[19]),
    'provenance/build/current-built-image-inspect.json':Path(sys.argv[20]),
    'provenance/build/preflight/preflight-measurement.json':Path(sys.argv[21])/'preflight-measurement.json',
    'provenance/build/preflight/installed/package.xml':Path(sys.argv[21])/'installed/package.xml',
    'provenance/build/preflight/installed/plugins.xml':Path(sys.argv[21])/'installed/plugins.xml',
    'provenance/build/preflight/installed/libfactory_ops_goal_checker.so':Path(sys.argv[21])/'installed/libfactory_ops_goal_checker.so',
}
build_files=[]
for relative,path in sorted(build_sources.items()):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f'build provenance source is not a regular file: {path}')
    build_files.append({'path':relative,'bytes':path.stat().st_size,
                        'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
build_manifest={'schema_version':1,'classification':'g3_build_provenance_bundle','files':build_files}
build_manifest_path=root/'logs'/f'{run}-build-provenance-manifest.json'
with build_manifest_path.open('x',encoding='utf-8',newline='\n') as stream:
    json.dump(build_manifest,stream,indent=2,sort_keys=True); stream.write('\n')
build_manifest_sha=hashlib.sha256(build_manifest_path.read_bytes()).hexdigest()
build_receipt=json.loads((Path(sys.argv[18])/'build-receipt.json').read_text(encoding='utf-8'))
sys.path.insert(0,str(project))
from factory_ops.source_provenance import verify_source_archive
proof=verify_source_archive(root/'source-snapshots'/f'{run}.tar',
    root/'source-snapshots'/f'{run}.commit',sys.argv[4],source_files,snapshot_root=project)
if proof.get('passed') is not True:
    raise ValueError(f'fixed Git source proof failed: {proof}')
session_value=json.loads((session/'session.json').read_text(encoding='utf-8'))
record={
    'schema_version':1,
    'run_label':run,'run_id':sys.argv[3],'source_commit':sys.argv[4],
    'checkout_head':sys.argv[4],'checkout_clean':True,'source_git_object_verification':True,
    'source_archive_sha256':proof['source_archive_sha256'],
    'source_commit_object_sha256':proof['commit_object_sha256'],
    'source_snapshot_relative':'project',
    'source_snapshot_host_relative':project.relative_to(root).as_posix(),
    'source_files':source_files,'source_verification':proof,
    'started_at_utc':datetime.now(timezone.utc).isoformat(),
    'isaac_image':sys.argv[5],'nav_image':sys.argv[6],
    'profile':sys.argv[7],'condition':sys.argv[8],'policy':sys.argv[9],
    'seed':int(sys.argv[10]),'scenario_id':sys.argv[11],
    'primary_inputs_path':sys.argv[12] or None,
    'primary_inputs_sha256':sys.argv[13] or None,
    'build_attestation_path':'provenance/build/build-attestation.json',
    'build_attestation_sha256':sys.argv[15],
    'build_receipt_path':'provenance/build/build-receipt.json',
    'build_receipt_sha256':sys.argv[16],
    'build_builder_mode':build_receipt['builder_mode'],
    'build_docker_client_version':build_receipt['docker_client_version'],
    'build_docker_engine_version':build_receipt['docker_engine_version'],
    'build_preflight_measurement_path':'provenance/build/preflight/preflight-measurement.json',
    'build_preflight_measurement_sha256':sys.argv[17],
    'build_provenance_manifest_path':'provenance/build-manifest.json',
    'build_provenance_manifest_sha256':build_manifest_sha,
    'session':session_value,
    'session_json_sha256':hashlib.sha256((session/'session.json').read_bytes()).hexdigest(),
    'files':source_files,
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

provenance_sha=$(python3 - "$root/logs/$run-environment.json" <<'PY'
from pathlib import Path
import hashlib,sys
print(hashlib.sha256(Path(sys.argv[1]).read_bytes()).hexdigest())
PY
)
test ! -e "$output/provenance"
sudo -n install -d -m 0755 -o 1234 -g 1234 \
  "$output/provenance" "$output/provenance/build" "$output/provenance/build/preflight" \
  "$output/provenance/build/preflight/installed"
sudo -n install -m 0444 -o 1234 -g 1234 "$archive" "$output/provenance/source.tar"
sudo -n install -m 0444 -o 1234 -g 1234 "$commit_object" "$output/provenance/source-commit.raw"
sudo -n install -m 0444 -o 1234 -g 1234 "$root/logs/$run-environment.json" "$output/provenance/source-environment.json"
sudo -n install -m 0444 -o 1234 -g 1234 "$root/logs/$run-build-provenance-manifest.json" \
  "$output/provenance/build-manifest.json"
for name in build-attestation.json build-receipt.json commands.jsonl built-image.iid \
  base-image-inspect.json built-image-inspect.json installed-measurement.json; do
  sudo -n install -m 0444 -o 1234 -g 1234 "$build_bundle/$name" "$output/provenance/build/$name"
done
sudo -n install -m 0444 -o 1234 -g 1234 "$base_inspect" \
  "$output/provenance/build/current-base-image-inspect.json"
sudo -n install -m 0444 -o 1234 -g 1234 "$built_inspect" \
  "$output/provenance/build/current-built-image-inspect.json"
sudo -n install -m 0444 -o 1234 -g 1234 "$preflight_measurement/preflight-measurement.json" \
  "$output/provenance/build/preflight/preflight-measurement.json"
for name in package.xml plugins.xml libfactory_ops_goal_checker.so; do
  sudo -n install -m 0444 -o 1234 -g 1234 "$preflight_measurement/installed/$name" \
    "$output/provenance/build/preflight/installed/$name"
done
chmod -R a-w "$project"
chmod a-w "$archive" "$commit_object"

if [ -n "$billing" ]; then
  sudo -n chown 1234:1234 "$resources"
  mapfile -t start_values < <(python3 - "$resources/start.json" <<'PY'
from pathlib import Path
import json,sys
value=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
print(value['observed_at_utc']); print(value['free_disk_bytes'])
PY
)
  test "${#start_values[@]}" = 2
  start_args=(start --output "$resources/attempt-start.json" --evidence-root "$root"
    --run-id "$run_id" --profile "$profile" --condition "$condition" --policy "$policy" --seed "$seed"
    --source-commit "$expected_commit" --config "$project/configs/$factory_profile/g3-experiment.json"
    --session-root "$session" --billing "$billing" --started-at-utc "${start_values[0]}"
    --free-disk-bytes-before "${start_values[1]}")
  if [ "$profile" = primary ]; then
    mapfile -t primary_paths < <(python3 - "$primary_inputs" <<'PY'
from pathlib import Path
import json,sys
value=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
print(value['trial_path']); print(value['calibration_path'])
PY
)
    test "${#primary_paths[@]}" = 2
    start_args+=(--trial-json "${primary_paths[0]}" --gate "${primary_paths[1]}"
      --primary-inputs "$primary_inputs" --primary-inputs-sha256 "$primary_inputs_sha")
  fi
  docker run --rm --name "factory-ops-$run-start" --network none --user 1234:1234 \
    --entrypoint /bin/bash -e HOME=/tmp -v "$root:$root:ro" -v "$resources:$resources:rw" \
    -v "$project:/workspace/project:ro" "$nav_image" \
    -lc 'source /opt/ros/jazzy/setup.bash; cd /workspace/project; python3 -m factory_ops.g3_supervisor "$@"' \
    factory-ops-g3-supervisor "${start_args[@]}"
fi

docker create --name "$sim" --gpus all --network host --ipc host --user 1234:1234 \
  --entrypoint /bin/bash --workdir /isaac-sim \
  -e ACCEPT_EULA=Y -e OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1 -e ROS_DOMAIN_ID=72 \
  -e FACTORY_OPS_G3_SESSION_SHA256="$session_sha" \
  -e FACTORY_OPS_G3_SOURCE_COMMIT="$expected_commit" -e FACTORY_OPS_G3_PROVENANCE_SHA256="$provenance_sha" \
  -e FACTORY_OPS_G3_PRIMARY_INPUTS_PATH="$primary_inputs" -e FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256="$primary_inputs_sha" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$cache/cache/main:/isaac-sim/.cache:rw" \
  -v "$cache/cache/computecache:/isaac-sim/.nv/ComputeCache:rw" \
  -v "$cache/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$cache/logs:/isaac-sim/.nvidia-omniverse/logs:rw" \
  -v "$cache/config:/isaac-sim/.nvidia-omniverse/config:rw" \
  -v "$cache/data:/isaac-sim/.local/share/ov/data:rw" \
  -v "$cache/pkg:/isaac-sim/.local/share/ov/pkg:rw" \
  -v "$cache/hub:/var/cache/hub:rw" \
  -v "$root:$root:ro" -v "$project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$isaac_image" -lc "timeout --signal=TERM --kill-after=30s 5500s ./python.sh /workspace/project/scripts/isaac/g3_scene.py --session /workspace/project/runs/${run}-session --output /workspace/results/$run --run-id $run_id" >/dev/null
created+=("$sim")

docker create --name "$nav" --network host --ipc host --user 1234:1234 \
  --entrypoint /bin/bash -e HOME=/tmp -e ROS_DOMAIN_ID=72 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FACTORY_OPS_G3_SESSION_SHA256="$session_sha" \
  -e FACTORY_OPS_G3_SOURCE_COMMIT="$expected_commit" -e FACTORY_OPS_G3_PROVENANCE_SHA256="$provenance_sha" \
  -e FACTORY_OPS_G3_BUILD_ATTESTATION_PATH="/workspace/results/$run/provenance/build/build-attestation.json" \
  -e FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256="$build_attestation_sha" \
  -e FACTORY_OPS_G3_PRIMARY_INPUTS_PATH="$primary_inputs" -e FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256="$primary_inputs_sha" \
  -v "$root:$root:ro" -v "$project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$nav_image" -lc "source /opt/ros/jazzy/setup.bash; timeout --signal=TERM --kill-after=30s 5500s python3 /workspace/project/scripts/ros/g3_mission.py --session /workspace/project/runs/${run}-session --output /workspace/results/$run --run-id $run_id" >/dev/null
created+=("$nav")

docker inspect "$sim" "$nav" > "$root/logs/$run-containers.json"
sampler_started_at=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)
python3 "$project/scripts/brev/g3_gpu_sampler.py" --raw-path "$root/logs/$run-gpu-raw.jsonl" > "$root/logs/$run-gpu.csv" &
sampler_pid=$!
# The ROS worker seals session verification before the simulator is started.
docker start "$nav" >/dev/null
session_verified_deadline=$((SECONDS + 1800))
while [ ! -f "$output/session-verified.json" ]; do
  test "$(docker inspect "$nav" --format '{{.State.Running}}')" = true
  test "$SECONDS" -lt "$session_verified_deadline"
  sleep 1
done
python3 - "$output/session-verified.json" "$run_id" "$session_sha" "$primary_inputs_sha" "$expected_commit" <<'PY'
from pathlib import Path
import json,sys
value=json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
assert value['run_id']==sys.argv[2]
assert value['session_sha256']==sys.argv[3]
assert value.get('primary_inputs_sha256')==(sys.argv[4] or None)
assert value.get('source_commit')==sys.argv[5]
PY
docker start "$sim" >/dev/null
docker logs -f "$sim" > "$root/logs/$run-sim.log" 2>&1 &
sim_log_pid=$!

docker logs -f "$nav" > "$root/logs/$run-nav.log" 2>&1 &
nav_log_pid=$!
python3 "$project/scripts/brev/g3_pair_wait.py" --nav "$nav" --sim "$sim" \
  --record "$root/logs/$run-pair-lifecycle.json"
set +e
nav_status=$(docker wait "$nav")
wait "$nav_log_pid"
nav_log_status=$?
nav_log_pid=
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
printf '%s\n' "$nav_log_status" > "$root/logs/$run-nav-log-exit.txt"

docker logs "$sim" > "$root/logs/$run-sim.log" 2>&1
sudo -n install -m 0644 -o 1234 -g 1234 "$root/logs/$run-sim.log" "$output/isaac.log"
docker logs "$nav" > "$root/logs/$run-nav-complete.log" 2>&1
sudo -n install -m 0644 -o 1234 -g 1234 "$root/logs/$run-nav-complete.log" "$output/mission-worker.log"
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
print('FACTORY_OPS_G3_PAIR_EXIT_RECORDED')
PY
sudo -n python3 "$pack_program" workers-finished --root "$root" --run-label "$run" --run-id "$run_id"
sampler_alive=false
if kill -0 "$sampler_pid" >/dev/null 2>&1; then sampler_alive=true; fi
sampler_stop_requested_at=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)
sampler_intentional_stop=false
if [ "$sampler_alive" = true ] && kill "$sampler_pid" >/dev/null 2>&1; then sampler_intentional_stop=true; fi
set +e
wait "$sampler_pid"
sampler_exit=$?
set -e
sampler_pid=
sudo -n python3 "$project/scripts/brev/g3_gpu_sampler.py" --seal-status "$resources/gpu-sampler-status.json" \
  --run-id "$run_id" --csv-path "$root/logs/$run-gpu.csv" --raw-path "$root/logs/$run-gpu-raw.jsonl" \
  --sampling-started-at-utc "$sampler_started_at" --stop-requested-at-utc "$sampler_stop_requested_at" \
  --sampler-alive-before-stop "$sampler_alive" --intentional-stop-requested "$sampler_intentional_stop" --exit-code "$sampler_exit"

docker create --name "$evidence" --network none --user 1234:1234 \
  --entrypoint /bin/bash -e HOME=/tmp \
  -e FACTORY_OPS_G3_SESSION_SHA256="$session_sha" \
  -e FACTORY_OPS_G3_SOURCE_COMMIT="$expected_commit" -e FACTORY_OPS_G3_PROVENANCE_SHA256="$provenance_sha" \
  -e FACTORY_OPS_G3_BUILD_ATTESTATION_PATH="/workspace/results/$run/provenance/build/build-attestation.json" \
  -e FACTORY_OPS_G3_BUILD_ATTESTATION_SHA256="$build_attestation_sha" \
  -e FACTORY_OPS_G3_PRIMARY_INPUTS_PATH="$primary_inputs" -e FACTORY_OPS_G3_PRIMARY_INPUTS_SHA256="$primary_inputs_sha" \
  -v "$root:$root:ro" -v "$project:/workspace/project:ro" -v "$root/results:/workspace/results:rw" \
  "$nav_image" -lc "source /opt/ros/jazzy/setup.bash; timeout --signal=TERM --kill-after=10s 600s python3 /workspace/project/scripts/ros/g3_evidence.py --session /workspace/project/runs/${run}-session --output /workspace/results/$run --run-id $run_id" >/dev/null
created+=("$evidence")
set +e
docker start -ai "$evidence" 2>&1 | tee "$root/logs/$run-validation.log"
evidence_status=${PIPESTATUS[0]}
set -e
if [ "$sampler_alive" != true ] || [ "$sampler_intentional_stop" != true ] || [ "$sampler_exit" != 0 ]; then
  evidence_status=gpu_resource_collection_failed
fi
sudo -n python3 "$pack_program" validation-finished --root "$root" --run-label "$run" --run-id "$run_id"
finalize_resources
test "$evidence_status" = 0
