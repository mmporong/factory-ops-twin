# fleet4-ros-006 / attempt 1 / revision 6

사용자의 최신 목표는 전체 약50%에서80%까지 일반 구현을 계속하는 것이다. 현재 native goal은 blocked이며 Conductor/Ralplan을 시작하지 않는다. 작성자는 기존 control 세션이며 root가 통합·검수한다.

## 목표와 소유권

기존 두 대 qualification과 세션 검증을 보존하면서 registry 기반 네 대 ROS/Nav2 운송용 개발 입구를 구현한다. 수정 소유권은 `scripts/ros/g2_mission.py`, `scripts/ros/g3_mission.py`, 신규 `scripts/ros/fleet4_mission.py`, 신규 `scripts/ros/fleet4.launch.py`, 관련 ROS helper/entry 테스트다. 기존 g2.launch.py는 가능한 한 보존한다. 테스트 파일이 추가로 필요하면 새 `tests/test_fleet_transport_mission.py`를 사용한다. root는 layout/routing/config/bundle/Isaac scene/충전·화물·문서·Git 통합을 소유한다. 같은 파일을 동시에 수정하지 않는다.

입력은 HEAD73640fb, 수용된 `coordination/collab/results/fleet-ros-map-004.json`, FleetRegistry와 기존 ROS helper다. 현재 UI 변경은 다른 요청의 산출물로 보존하고 이 작업에서 되돌리거나 임의 커밋하지 않는다. 필요하면 해당 변경의 기존 검증·파일 SHA를 별도 결과로 알려준다.

## root와의 고정 인터페이스

root가 신규 `factory_ops/fleet_transport.py`를 작성한다. `load_bundle(root: Path) -> dict[str, Any]`는 입력 SHA·registry·배치·경로·주차 검증을 거친 뒤 다음을 반환한다.

- `config`: config.json의 G2 구동 설정 + `g3_runtime`, sim/wall 한계
- `session`: `scenario_id`, `condition`, `policy`, `profile="development"`, `primary_evaluation=false`, `scope="fleet4_transport_development"`, `scene_closure`, `workload`, `workload_sha256`
- `registry`: FleetRegistry 인스턴스
- `book`: RouteBook 인스턴스
- `bundle_sha256`: bundle.json 바이트 SHA256

bundle에는 config.json, scenarios.json, session.json, registry.json, layout.json, g3-routing.json, nav2-g2.yaml, render-profile.json, visual-assets.json, scene/factory-v3.usda가 존재한다. session.workload는 기존 G3 shape(initial_robots, load, orders, horizon_s)이며 네 시작·주차는 P1/P2/P3/P4다. namespace는 robot ID와 다를 수 있으므로 명시 mapping을 전 구간 전달한다. load_bundle이 아직 없으면 import를 늦추고 isolated tests를 먼저 작성한다. 이 인터페이스를 바꿔야 하면 root에 결과 파일로 먼저 제안한다.

새 개발 입구 CLI: --bundle, --output, --run-id, --dashboard. 기존 G3의 verify_session 및 primary_trial_identity를 새 개발 입구에서 위조·우회 호출하지 않는다. 별도 fleet-transport-verified.json marker로 run/scenario/bundle SHA/robot namespace+prim/source 정보를 기록한다. G3 session-verified.json과 의미를 섞지 않는다. ready의 run/scenario/정확 robot ID/namespace/prim과 footprint를 registry에 대조한다. root가 Isaac scene lifecycle와 GPU supervisor를 구현한다.

## 수용 조건

1. ROBOT_IDS monkeypatch 없이 정확 1/2/4대 집합과 명시 namespace mapping을 전달한다. params, TF, actions, scan filtering/recorder topics, initial HOLD barrier, cutoff, terminal odometry, runtime provenance까지 일관된 집합을 쓴다.
2. 기존 두 대 entry 및 qualification/session/CDR·goal checker·STOP/retained hold·GT 금지 게이트는 유지한다.
3. 네 대 초기 HOLD/terminal 검사에서 누락·추가 차량, stale/부분 ACK와 late odometry를 거부한다. 다른 namespace의 params/topic 생성도 검사한다.
4. 물리 운송·적재 성공은 CPU 테스트만으로 주장하지 않는다. root가 GPU 실행한다.
5. 관련 회귀 테스트·새 helper/entry 테스트 및 가능한 범위의 typecheck를 수행한다. 기존 전체 g3 함수 복잡도 진단은 새 오류와 구분한다.

## 운영 경계와 결과

이 저장소에는 root와 다른 요청의 변경도 있다. 배정 경로만 수정하고 다른 세션의 변경을 되돌리지 않는다. GPU 시작, 원격 조작, commit/push, 새 세션/서브에이전트, 서버 종료를 하지 않는다. 작업 ID/attempt를 ACK하고 진행·산출물·검증·미해결을 반환한다. 계약/공유 경로 확장은 root에 올린다. accepted 판정은 root 소유다.

결과: `coordination/collab/results/fleet4-ros-006.json`. task/attempt/revision/ACK, 입력 commit, 실제 변경 경로·SHA, 실행 명령·결과, 실패·미검증, 다음 필요 사항을 기록한다. UI 변경은 별도 목록으로 분리한다.
