# fleet4-closed-evidence-012 / attempt1 / revision11

011은 root가 수용했다. 기존 control 페인에 다음 작업을 전달한다.

root는 011의 소스 SHA와 준비9개+caller10개 검사를 확인해411670c에 커밋·push했다. 새 immutable worktree002에서 normal-002를 실행 중이다. runtime 생산자는 수정하지 않는다. control은 종료된 네 대 개발 운송의 별도 CPU 증거 검증 도구를 작성한다. 사용자 요청은 리뷰 양을 줄이고 핵심 로직을 먼저 구현하는 것이므로 기존 전체 qualification을 새로 설계하지 않는다.

## 소유권과 경계

- 신규 `factory_ops/fleet_transport_evidence.py`, `tests/test_fleet_transport_evidence.py`, 결과 receipt만 작성한다.
- `factory_ops/fleet_transport.py`, ROS/SDK/runner/API/UI/Git/실제 VM은 수정하거나 실행하지 않는다. 다른 변경을 되돌리지 않는다.
- 원본은 읽기 전용이며 새 report 경로만 쓴다. 활성 bag SQLite를 열지 않는다. 이 도구는 JSON/JSONL만 검사한다. SQL/RPC·새 에이전트·Brev·commit/push는 사용하지 않는다.

## 완료 기준

CLI는 `--bundle <inputs> --stage <closed stage> --expected-run-id <32hex> --expected-source-commit <40hex> --report <new json>`를 받는다. 입력 bundle은 기존 load_bundle, source archive/raw commit은 기존 verify_source_archive를 재사용한다. stage의 fresh closure-confirmation.json에서 정확한 run ID/label과 남은 소유 컨테이너 없음, runner-exit.json의 두 worker 종료/cleanup을 요구한다. 없거나 실패하면 통과하지 않는다.

종료 결과 형식은 소스의 실제 기록을 읽어 구현한다: `results/run/fleet4-transport-scene.json`, `mission-result.json`, `mission-control.json`, `robots/*/trajectory.jsonl`, `lidar-acquisition-events.jsonl`, `media/frames.json`, `provenance/fleet-source.json`. 공유 g2_scene/g3_mission의 실제 키를 근거로 검사하며 잘못 추정한 fixture로 PASS를 만들지 않는다. 실제 자료가 아직 없으면 기록 소스의 줄/키와 미검증 범위를 receipt에 남긴다.

최소 판정은 실행/시나리오/입력/생산자 결속, SDK validated+정리 오류 없음, ROS passed+4개 일치 차량, 4대 각각의 실제 이동 trajectory와 성공한 FollowPath/정지, 모든 선언 주문의 accepted handoff·출하와 최종 주차, 8개 LiDAR의 전진한 GMO/ROS counts, 인덱스에 결속된 실제 RGB 파일 해시다. 각도/이동/정지 허용치는 기존 config/profile와 실제 필드에 근거하며 임의로 느슨하게 만들지 않는다. 실제 scalar 기준이 없는 항목은 검증 범위를 제한해 명시한다.

작은 음성 검사로 다른 run/source, 미참여 차량, worker 실패/closure 없음, 소스/미디어 변조를 거절한다. worker exit를 전체 QA나 물리 적재·충전 성공으로 확대하지 않는다. 결과 scope는 `four-AMR development JSON evidence; not full bag/action qualification, physical loading, charging or policy comparison`처럼 분명히 한다. 새 파일/함수는 작은 단위로 작성한다.

root가 실제 종료 원본에 이 도구를 실행하고 결과를 수용한다. 결과는 `coordination/collab/results/fleet4-closed-evidence-012.json`: ACK, 변경 SHA, 기록 형식 근거, 검사와 제약을 기록한다. 질문은 그 receipt나 기존 pane 응답으로 root에 전달한다.
