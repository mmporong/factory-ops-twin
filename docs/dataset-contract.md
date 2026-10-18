# 렌더 데이터 계약

`dataset.json`은 데이터셋의 권위 있는 진입점이다. `sim_time_ns` 정수가 시간 결합의 기준이며, 부동소수점 시각은 저장하지 않는다. 데이터 범위는 한 합성 공장 배치에서 만든 독립 시각 구성(`scene_id`)이다. 캡처 프레임 수를 독립 장면 수로 해석하거나 모델 성능을 주장하지 않는다.

## 최소 구조

```text
dataset.json
visual-review.json
scenes/scene-000/captures/f000000120-t000000002000000/
  capture.json
  overhead/{rgb.png,depth.npy,semantic.npy,instance.npy,bbox2d.raw.json,camera_params.json,normalized.json}
  aisle/{...}
  amr_follow/{...}
```

`dataset.json`의 최소 완성 예시는 다음과 같다. `artifact_manifest`는 `dataset.json` 자신을 제외한 모든 정규 파일과 정확히 일치해야 한다.

```json
{
  "schema_version": 1,
  "dataset_id": "factory-v3-render-001",
  "run_id": "g4-render-001",
  "scope": "independent visual configurations of one synthetic layout",
  "source": {
    "commit": "40자리 Git SHA",
    "scene_path": "provenance/scene-sources.json",
    "scene_sha256": "64자리 SHA-256",
    "asset_manifest_path": "provenance/asset-manifest.json",
    "asset_manifest_sha256": "64자리 SHA-256",
    "render_profile_path": "provenance/render-profile.json",
    "render_profile_sha256": "64자리 SHA-256"
  },
  "profile_sha256": "dataset-profile.json의 SHA-256",
  "split_assignments": {"scene-000": "train"},
  "scenes": [
    {
      "scene_id": "scene-000",
      "scene_variant_id": "lighting-a-material-a-clutter-a",
      "run_id": "g4-render-001",
      "captures": [
        {
          "capture_id": "scene-000--f000000120--t000000002000000",
          "physics_frame": 120,
          "sim_time_ns": 2000000,
          "robot_pose": {"position_xyz": [1.0, 2.0, 0.0], "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
          "record_path": "scenes/scene-000/captures/f000000120-t000000002000000/capture.json"
        }
      ]
    }
  ],
  "artifact_manifest": [
    {"path": "visual-review.json", "bytes": 123, "sha256": "64자리 SHA-256"}
  ]
}
```

`capture.json`은 장면 항목과 동일한 ID·프레임·시각·로봇 자세를 반복해 파일 경계에서 결합을 검증하고, 프로필에 고정된 카메라 이름을 모두 포함한다.

```json
{
  "scene_id": "scene-000",
  "scene_variant_id": "lighting-a-material-a-clutter-a",
  "capture_id": "scene-000--f000000120--t000000002000000",
  "physics_frame": 120,
  "sim_time_ns": 2000000,
  "robot_pose": {"position_xyz": [1.0, 2.0, 0.0], "orientation_wxyz": [1.0, 0.0, 0.0, 0.0]},
  "cameras": {
    "overhead": {
      "rgb": ".../overhead/rgb.png",
      "depth": ".../overhead/depth.npy",
      "semantic": ".../overhead/semantic.npy",
      "instance": ".../overhead/instance.npy",
      "bbox2d_raw": ".../overhead/bbox2d.raw.json",
      "camera_params": ".../overhead/camera_params.json",
      "normalized": ".../overhead/normalized.json"
    }
  }
}
```

`source.scene_path`가 가리키는 `scene-sources.json`은 모든 `scene_id`와 `scene_variant_id`를 정확히 한 번씩 포함한다. 각 variant는 서로 겹치지 않는 `render_scene_path`·`visual_manifest_path`와 각각의 SHA-256, 실제 Nova Carter USD URI, 런타임 prop의 `prim_path`·`source`·`asset_source_sha256`을 기록한다. visual manifest의 asset provenance SHA는 데이터셋의 asset manifest SHA와 같아야 한다. 시각 asset root는 이 manifest, Carter root는 실제 USD URI의 SHA-256, payload와 safety barrier root는 선언된 source 문자열의 SHA-256에서 허용 source SHA를 다시 계산한다. Carter의 `robot_asset_identity_sha256`은 원격 USD 파일 바이트가 아니라 UTF-8 URI의 identity digest이며 `robot_asset_hash_basis: "utf8-uri"`로 명시한다. 런타임 prop도 `hash_basis: "utf8-source-descriptor"`를 명시한다. 로컬 visual config와 render USDA는 파일 바이트 SHA를 쓴다. annotation에 들어 있는 SHA만으로 asset 출처를 인정하지 않는다. 서로 다른 split의 장면이 RGB·depth·semantic·instance 전체를 그대로 공유하면 누출로 판정한다.

컨테이너는 `/workspace/g4-environment.json`을 source proof로 사용한다. supervisor는 파일 전체 SHA-256을 `FACTORY_OPS_G4_PROVENANCE_SHA256`에 전달한다. manifest와 `source.verification`은 `schema_version: 1`을 가지며, manifest는 같은 `source_commit`·`checkout_head`, clean checkout, Git object 검증 결과, `source_archive_sha256`·`source_commit_object_sha256`, `source_snapshot_relative: "project"`, `{상대경로: SHA-256}` 형식의 `source_files`를 가진다. capture producer는 raw commit object의 Git SHA-1, archive 전체의 blob·mode·tree, readonly snapshot의 source pin을 다시 계산해 서로 같은 commit인지 검사한 뒤에만 Isaac을 시작한다. 원본 manifest와 source archive/commit object를 데이터셋 provenance에 복사하고 종료 직전에 manifest·archive·commit object·전체 source map을 다시 검증한다. QA도 보존된 archive와 raw commit으로 같은 계산을 반복한다.

QA report는 선택 profile의 ID·SHA와 `validation_gate`를 기록하고 다음 상태를 분리한다.

- `deterministic_data_passed`: manifest, source, 12 scene·30 capture·3 camera, 1,080 RGB, modality, 좌표, mask, depth, class, split 누출, leaf join과 가림 coverage를 포함한 결정적 데이터 검사가 모두 통과했다.
- `portfolio_qa_passed`: 결정적 데이터 검사가 통과했고, 닫힌 데이터셋 밖의 AI 포트폴리오 검수 sidecar가 모든 scene-camera와 카메라별 clear·partial·heavy·compound 대표를 실제 RGB와 원형 가림 자료에 결속했다. sidecar를 제공하지 않으면 `false`다.
- `human_release_approved`: 원본 `visual-review.json`의 모든 필수 항목이 명시적 `reviewer_type: "human"` attestation을 충족했다. reviewer type이 없는 과거 문서는 `legacy_unspecified`로 해석하며 자동 승인하지 않는다.
- `production_passed`: `deterministic_data_passed`, 고정 production profile, `human_release_approved`가 모두 참이다. AI 검수는 이 상태를 참으로 만들 수 없다.

기존 `passed`는 호출에서 요청한 전체 검증에 오류가 없다는 호환 의미를 유지한다. 보고서는 `data_errors`, `human_review_errors`, `portfolio_qa_errors`를 따로 기록하고 세 목록을 합친 `errors`도 유지한다. 프로젝트에 고정된 `factory-v3-portfolio-render-v1`의 SHA, 12 scene, scene당 30 capture, 3 camera, 8/2/2 scene split과 정확히 같을 때만 production profile 자격이 생긴다. 작은 test profile은 검증 자체가 통과해도 `diagnostic_fixture`이며 생산 데이터 승인으로 해석하지 않는다.

split 누출 fingerprint는 capture와 camera 순서에 무관한 multiset이다. RGB는 PNG 압축 수준과 scanline filter를 제거한 실제 pixel bytes로, NPY는 header 버전과 padding을 제외한 dtype·shape·payload로 digest를 만든다. 파일 인코딩만 바꾼 같은 이미지·배열이 서로 다른 split에 들어가면 같은 내용으로 판정한다.

`camera_params.json`은 `scene_id`, `capture_id`, `camera_id`, `physics_frame`, `sim_time_ns`, `width`, `height`, `pose_xyz`, `orientation_wxyz`, `focal_length`, `horizontal_aperture`, `vertical_aperture`, `clipping_range`, `world_to_camera`(16개), `projection`(16개)을 가진다. 행렬과 렌즈 값은 모두 유한해야 하고 해상도는 RGB 및 배열과 같아야 한다. `world_to_camera`는 USD row-vector convention에서 pose translation과 unit quaternion으로 역산한 extrinsic과 같아야 한다. `projection`은 focal length, 두 aperture, near/far clipping으로 다시 계산한 perspective matrix와 같아야 한다. `BACKGROUND`의 `distance_to_image_plane`은 양의 무한대이고, semantic과 instance 원형 label map에서 `UNLABELLED`로 확인된 렌더 geometry는 `[near, far]` 안의 유한한 양수 depth를 가진다.

각 camera 디렉터리의 `semantic.raw-info.json`, `instance.raw-info.json`, `camera.raw-params.json`은 정규화 전에 저장한 annotator metadata다. `normalized.json.raw_info.source_artifacts`는 앞의 두 파일명과 SHA-256을, `camera_params.json.raw_camera_params_artifact`는 camera 원형 파일명과 SHA-256을 기록한다. 정규화 문서에 복제된 raw label map과 camera params는 보존 파일의 JSON 값과 같아야 한다.

한 촬영 시점의 세 카메라 payload를 먼저 읽고 모든 카메라의 RGB·depth·semantic·instance·원형 bbox·metadata를 저장한 뒤 shape 검사와 정규화를 시작한다. 정규화 실패 시 저장된 원형 파일을 유지하며 승인 자료로 분류하지 않는다.

`bbox2d.raw.json`은 Replicator가 반환한 원형 정보를 JSON 표현으로 보존한다.

```json
{
  "data": [{"semanticId": 4, "x_min": 10, "y_min": 20, "x_max": 40, "y_max": 60, "occlusionRatio": 0.25}],
  "info": {
    "idToLabels": {"4": {"class": "rack"}},
    "primPaths": [["/World/Factory/Rack_01"]]
  }
}
```

이 계약에서 bbox `semanticId`, semantic segmentation ID, instance segmentation ID는 서로 독립된 수치 공간이다. bbox 원형의 `info.idToLabels`와 행 인덱스에 대응하는 `info.primPaths`를 가공하지 않고 보존한다. semantic·instance annotator의 원형 label map과 instance ID별 prim path는 `normalized.json`의 `raw_info`에 따로 보존한다. `reserved_instance_ids`는 숫자 자체로 정하지 않고, 보존된 instance `idToLabels`가 해당 ID를 `BACKGROUND` 또는 `UNLABELLED`로 명시할 때만 허용한다. 일반 class pixel의 instance ID를 reserved로 선언해 정규 instance와 bbox 검증에서 숨길 수 없다.

```json
{
  "scene_id": "scene-000",
  "capture_id": "scene-000--f000000120--t000000002000000",
  "camera_id": "overhead",
  "physics_frame": 120,
  "sim_time_ns": 2000000,
  "raw_annotations_sha256": "bbox2d.raw.json의 SHA-256",
  "raw_info": {
    "semantic_id_to_labels": {"0": {"class": "BACKGROUND"}, "9": {"class": "rack"}},
    "instance_id_to_labels": {"27": {"class": "rack"}},
    "instance_id_to_prim_paths": {"27": "/World/Factory/Rack_01/Mesh"},
    "instance_id_to_labelled_roots": {"27": "/World/Factory/Rack_01"},
    "instance_id_to_asset_source_sha256": {"27": "64자리 SHA-256"}
  },
  "instances": [
    {
      "instance_id": 27,
      "bbox_semantic_id": 4,
      "class": "rack",
      "usd_root_path": "/World/Factory/Rack_01",
      "asset_source_sha256": "64자리 SHA-256",
      "stable_instance_key": "64자리 SHA-256",
      "bbox_xyxy": [10, 20, 40, 60],
      "raw_bbox_index": 0,
      "edge_truncated": false,
      "raw_bbox_binding": {
        "kind": "instance_mask",
        "bbox_xyxy": [10, 20, 40, 60],
        "instance_ids": [27],
        "edge_truncated": false,
        "occlusion_ratio": 0.25,
        "occlusion_status": "measured_raw",
        "occlusion_raw_value": 0.25
      }
    }
  ]
}
```

안정 ID는 UTF-8 바이트열 `scene_variant_id + "\\0" + usd_root_path + "\\0" + asset_source_sha256`의 SHA-256이다. `instance_id_to_prim_paths`는 annotator가 반환한 원본 child prim path를 보존하고, `instance_id_to_labelled_roots`는 semantic class가 붙은 조상 root를 보존한다. child path는 labelled root와 같거나 그 하위여야 한다. bbox 행의 각 prim path는 labelled root와 조상·자손 관계여야 한다. `bbox2d.raw.json`의 `data`와 `info.primPaths`는 길이가 같고 행별로 일대일 대응한다. 모든 USD path는 `/World`에서 시작하는 canonical path여야 한다.

bbox와 instance는 가장 가까운 semantic-labelled root가 정확히 같을 때 연결한다. 같은 클래스의 부모와 자식이라는 이유로 합치지 않는다. SDK가 상위 semantic aggregate bbox를 함께 반환한 경우, 해당 상위 root 아래 보이는 모든 하위 instance의 mask bounds 합집합과 원본 bbox가 정확히 같을 때만 `raw_info.excluded_bbox_rows`에 `hierarchical_semantic_aggregate`로 기록한다. 부모와 자식의 class가 달라도 source ancestry로 입증된 계층이면 가능하다. 기록에는 정렬된 전체 `descendant_instance_ids`를 넣고 원형 행을 유지한다. QA는 상위 root, 전체 하위 ID 집합, 각 mask bounds의 합집합을 독립적으로 재검증하며 하위 ID 누락이나 bbox 확대·축소를 거부한다.

semantic, instance, bbox label map의 키는 `0`, `9`, `27` 같은 비음수 canonical 10진 문자열만 허용한다. `09`와 `9`, 음수, 비수치 키는 거부한다. 프로필의 `minimum_class_pixels`는 양의 정수여야 하며 문자열이나 0으로 production gate를 낮출 수 없다.

`bbox_semantic_id`는 선택한 원본 bbox 행의 `semanticId`와 같아야 하며 bbox label map의 class, instance label map의 class, semantic mask class가 모두 같아야 한다. `bbox_xyxy`와 `edge_truncated`는 parent instance mask에서 얻은 양끝 포함 tight bbox와 가장자리 접촉 여부다. `raw_bbox_binding.kind: "instance_mask"`이면 원본 bbox와 parent mask 좌표 차이가 프로필의 1픽셀 허용 범위 안이어야 한다. 이 범위를 벗어나면 parent와 source-bound visible descendant 전체의 정렬된 ID 집합 및 mask bbox 합집합이 원본과 0픽셀로 일치할 때만 `kind: "semantic_subtree_union"`을 허용한다. `raw_bbox_binding`은 원본 bbox 좌표·edge·occlusion을 별도로 보존하며 subtree의 edge나 occlusion을 parent instance 속성으로 사용하지 않는다. Isaac Sim 6.2가 `occlusionRatio: -1.0`을 반환하면 binding 안에 `occlusion_raw_value: -1.0`, `occlusion_ratio: null`, `occlusion_status: "unavailable_raw_sentinel"`로 기록하며 leaf 가림 coverage로 세지 않는다.

실제 렌더 한 프레임에서 source-bound foreground instance가 instance mask에 정확히 1픽셀로 존재하지만 그 canonical child root의 개별 SDK bbox 행은 없고, 이미 검증된 상위 aggregate 행의 전체 visible descendant 집합에는 포함된 사례가 관찰됐다. 이 경우에만 `raw_bbox_binding.kind: "individual_sdk_bbox_absent"`로 instance를 보존한다. `bbox_semantic_id`, `raw_bbox_index`, 원본 bbox 좌표·edge·occlusion 값은 null이며, mask tight bbox `[x,y,x,y]`, `bbox_source: "instance_segmentation_mask"`, `mask_pixel_count: 1`, 동일 픽셀의 semantic ID, 검증된 aggregate 행 인덱스를 별도 필드로 기록한다. QA는 실제 mask pixel 수와 bounds, canonical path·class·source·stable identity, 개별 child bbox 행이 전혀 없다는 사실, strict ancestor aggregate의 정확한 전체 descendant union과 child ID 포함을 독립적으로 다시 계산한다. 이 상태는 원본 bbox 가림 통계에 포함하지 않고 별도 개수로 보고한다. SDK 내부에서 개별 행이 누락된 이유는 확인되지 않았으므로 추정 원인을 데이터 계약에 기록하지 않는다.

같은 1픽셀 조건에서 검증된 상위 aggregate 행도 없으면 `raw_bbox_binding.kind: "individual_sdk_bbox_absent_source_bound_mask"`를 사용한다. 이 변형은 `covering_aggregate_raw_bbox_index` 필드를 두지 않으며, canonical root·class·source asset SHA·stable key, 정확히 1픽셀인 instance mask, 그 픽셀의 단일 semantic ID와 class, 동일 canonical identity를 가리키는 SDK bbox 행이 0개라는 조건을 모두 만족해야 한다. aggregate 행이 하나라도 있으면 기존 aggregate-backed 형식을 사용하고, 둘 이상이면 모호한 원본으로 거부한다. 두 변형 모두 mask tight bbox를 표시용 geometry로 쓸 뿐 SDK bbox 또는 SDK occlusion 측정으로 취급하지 않으며, QA는 전체 누락 수와 aggregate-backed/source-bound 수를 나눠 보고한다. 이 제한적 정규화는 데이터 무결성 검증이며 human release 승인, production 통과, 1080개 실제 캡처 완료를 뜻하지 않는다.

## Leaf 가림 증거

가림 검증은 `instance_id_segmentation`의 renderer leaf ID와 `OcclusionSD`를 같은 paused physics frame에서 수집한다. 각 카메라는 기존 산출물에 더해 다음 네 파일을 갖는다.

- `instance-id.npy`와 `instance-id.raw-info.json`
- `occlusion.raw.json`
- `renderer-instance-uri.raw.json`

세 JSON은 `scene_id`, `capture_id`, `camera_id`, `physics_frame`, `sim_time_ns`가 같아야 한다. `renderer-instance-uri.raw.json`은 `occlusion.raw.json`의 모든 `instanceId`를 SDK `get_uri_from_instance_segmentation_id()`로 조회한 전체 표다. 화면에 보이는 `instance-id.npy`의 0 이외 ID는 raw `idToLabels`와 SDK URI가 정확히 같아야 한다. 키 누락, ID 재번호화, 경로 치환은 허용하지 않는다.

한 timestamp의 overhead, aisle, amr_follow 원형 파일을 순차 저장한 다음 SDK URI 결속과 정규화를 시작한다. 첫 카메라의 URI 조회나 join이 실패해도 저장을 마친 세 카메라의 원형 SDK 응답을 재검사할 수 있다. 저장 중 중단에 대한 디렉터리 단위 원자성은 보장하지 않는다.

scene의 `semantic-roots.raw.json`은 visual manifest, Nova Carter URI, staged payload/barrier source에서 정한 labelled root와 class를 보존한다. leaf는 가장 가까운 ancestor root에만 결합한다. root가 없는 조명 같은 geometry는 `no_taxonomy_root`로 모두 기록하고 coverage에서 제외한다. raw ratio가 `[0,1]`이면 `measured_leaf`, `-1`이면 `unavailable_raw_sentinel`이다. 그 밖의 값은 실패다.

Isaac Sim은 부모와 자식의 semantic class를 `rack,tote`, `floor,floor_marking`처럼 합쳐 반환할 수 있다. 이 원형 문자열의 마지막 token이나 token 순서로 class를 고르지 않는다. visible instance path의 권위 있는 labelled ancestor class 집합이 SDK token 집합과 정확히 같은지 확인하고, nearest labelled root의 class를 normalized leaf class로 쓴다. 모든 visible foreground instance ID는 원본 `idToSemantics`에 있어야 하며 그 token 집합도 같은 source ancestry에 정확히 결합되어야 한다. semantic mask ID는 동일 픽셀의 instance ID들과 결합했을 때 하나의 leaf class만 가져야 한다. bbox ID공간은 별개이므로 각 bbox row의 `primPaths`에서 같은 검사를 다시 수행한다. raw label map과 보이지 않는 raw ID는 그대로 보존한다. normalized instance map의 키는 visible foreground instance ID 집합, normalized bbox map의 키는 원본 bbox row의 semantic ID 집합과 각각 정확히 같아야 하며 관측되지 않은 ID를 추가할 수 없다. normalized semantic map은 실제 visible ID와 reserved background/unlabelled ID만 담는다.

여러 renderer leaf로 구성된 labelled root는 `compound_root_unknown`이며 `true_compound_occlusion_ratio`는 항상 null이다. leaf 비율의 평균·합이나 bbox 비율을 compound object의 실제 가림률로 바꾸지 않는다. 한 leaf로만 구성되고 유효 측정이 있는 root만 `single_leaf_measured`가 될 수 있다.

production profile은 `reason: verifiedSDKleaf-onlymapping`을 고정하고 scene-camera마다 taxonomy에 결합된 유효 leaf 1개 이상, 카메라별 clear/partial/heavy bin, 지정 class의 전역 leaf 측정, join failure 0을 요구한다. 시각 검수에는 카메라별 세 bin과 compound unknown 대표가 필요하며 RGB SHA, raw occlusion SHA, renderer ID, leaf path, labelled root를 같은 frame에 결속한다. 검수 전 상태나 미측정 coverage는 승인되지 않는다.

캡처 사이에 live articulation root를 순간 이동시키지 않는다. 초기 spawn에서 physics warmup을 마친 AMR을 zero-wheel 상태로 유지하고 카메라·조명·시각 variant로 구성을 바꾼다. 각 visual-review entry는 `robot_ground_contact_verified: true`를 포함해야 하며, 차체·휠·caster가 분리되거나 지면에서 부유한 프레임은 승인할 수 없다.

`visual-review.json`은 장면·카메라별 대표 샘플을 포함한다. 각 항목은 `scene_id`, `camera_id`, `capture_id`, `rgb_path`, `rgb_sha256`, `reviewed`, `verdict`, `reviewer`, `reviewer_type`을 가진다. 인간 릴리스 attestation의 `reviewer_type`은 `human`이어야 한다. `reviewer`는 RGB를 확인한 사람의 수동 검수 식별자이며 `pending`, `unreviewed`, `unknown` 같은 placeholder는 승인으로 인정하지 않는다. 이 attestation은 이미지에서 지면 정합을 확인했다는 기록이며 물리 접촉 센서 증거를 뜻하지 않는다. 모든 장면과 카메라 조합이 `reviewed: true`, `verdict: "pass"`가 되기 전에는 프로덕션 데이터셋을 승인하지 않는다. 테스트 fixture의 `reviewer_type: "human"`은 분기 검증용 입력이며 실제 사람의 승인을 주장하지 않는다.

## AI 포트폴리오 시각 검수 sidecar

AI 검수 결과는 원본 manifest에 넣거나 `visual-review.json`을 바꾸지 않고 닫힌 데이터셋 디렉터리 밖의 새 JSON 파일에 저장한다. 검증 API의 선택 인자 `portfolio_review_path` 또는 CLI의 `--portfolio-review`로 전달한다. 정규 파일만 허용하며 symlink, junction, 데이터셋 내부 경로, 다른 dataset·run·profile·`dataset.json` SHA 결속은 거부한다. 검증이 끝나면 원본 manifest의 모든 파일에 대해 root 결속, resolve 결과, 정규 파일, 크기, symlink/junction 조상, SHA를 다시 검사한다. `dataset.json`도 다시 해시한다. sidecar는 최초 resolve 결과·크기·파일 identity·SHA를 보존하고 종료 시 같은 조건과 link/junction 부재를 다시 검사한다. 같은 바이트를 다른 파일이나 연결된 부모 경로로 바꾸는 것도 실패한다.

최소 상위 구조는 다음과 같다.

```json
{
  "schema_version": 1,
  "review_scope": "portfolio_qa_only",
  "reviewer_type": "ai",
  "reviewer": "검수 실행 식별자",
  "declared_model": "설정에서 읽은 모델 식별자",
  "model_identity_source": "installed_role_catalog",
  "review_protocol": "factory-dataset-portfolio-visual-review",
  "review_protocol_version": "1",
  "review_protocol_sha256": "64자리 SHA-256",
  "dataset_contract_sha256": "docs/dataset-contract.md의 SHA-256",
  "physical_contact_evidence": false,
  "limitations": ["RGB 관찰은 물리 접촉력 또는 충돌 센서 증거가 아님"],
  "dataset": {
    "dataset_id": "dataset.json과 같은 값",
    "run_id": "dataset.json과 같은 값",
    "dataset_json_sha256": "검수 대상 dataset.json의 SHA-256",
    "profile_sha256": "선택 profile의 SHA-256"
  },
  "entries": []
}
```

`model_identity_source`는 설치된 역할 catalog의 `installed_role_catalog` 또는 실행 설정의 `runtime_configuration`만 허용한다. 실행 호스트가 증명하지 않은 모델을 `host_attested`로 기록하지 않는다. reviewer·model 값은 비어 있거나 placeholder일 수 없다. 검수 protocol은 이름 `factory-dataset-portfolio-visual-review`, version `1`로 고정한다. `review_protocol_sha256`은 UTF-8 바이트열 `review_protocol + "\0" + review_protocol_version`의 SHA-256이어야 한다. 형식만 맞는 다른 SHA는 인정하지 않는다. AI sidecar 또는 개별 entry가 `human_release_approved`, `production_passed`, 인간 reviewer type을 주장하면 실패한다.

각 entry는 `scene_id`, `camera_id`, `capture_id`, `rgb_path`, `rgb_sha256`, `evidence_source: "dataset_rgb"`, `verdict: "pass"`, `observations`, `visible_defects`, `visual_contact_assessment`를 가진다. `visual_contact_assessment`는 화면에서 부품 분리·부유·지면 관통이 관찰되지 않은 `visible_consistent` 또는 가림·화면 밖으로 판단할 수 없는 `not_observable`이다. AI entry에서 `physical_contact_verified`나 `robot_ground_contact_verified`를 참으로 기록하면 실패한다. 정상적이고 라벨이 사실인 가림과 edge truncation은 결함이 아니며, 보이지 않는 AMR이나 부품의 접촉 상태를 추정해 참으로 바꾸지 않는다. `visible_defects`에 관찰된 분리·부유·관통이 하나라도 있으면 포트폴리오 QA를 통과하지 않는다. 생성 이미지나 구상 이미지는 `dataset_rgb` 증거가 될 수 없다.

모든 scene-camera 조합에는 `evidence_kind: "scene_camera_review"` 항목이 필요하다. 카메라별 clear·partial·heavy 대표는 `evidence_kind: "leaf_occlusion_bin"`으로 원형 `occlusion.raw.json` 경로와 SHA, renderer instance ID, leaf prim path, labelled root path를 기록한다. compound 대표는 `evidence_kind: "compound_root_unknown"`으로 같은 frame의 원형 SHA와 labelled root를 기록한다. 검증기는 RGB·capture·camera 결속, 원형 가림 SHA, normalized frame binding, leaf ID·path·root, 실제 ratio bin과 compound unknown 상태를 다시 계산한다.
