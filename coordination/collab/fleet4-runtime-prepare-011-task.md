# fleet4-runtime-prepare-011 / attempt1 / revision10

실제 normal-001 준비 단계에서 VM 기본 Python3.10이 StrEnum을 import하지 못해 종료됐다. Isaac/ROS 운송 worker는 시작하지 않았으며 실패 원본은 D/E SHA743df3424f2936fca7133e5d86d5d2a5000840cfaf6160b7d5b68a819ad1623a로 보존했다. 기존 caller가 Stop 확인 중이다. root는 host/sh의 준비 단계를 캐시된 ROS Jazzy 이미지(Python3.12) CPU 컨테이너로 감싸고 있다. control은 현재 UI 검증을 마친 뒤 아래 두 파일을 구현한다.

## 소유권

- 신규 `scripts/brev/fleet4_prepare.py`, 신규 `tests/test_fleet4_prepare.py`만 작성한다.
- root는 `scripts/brev/fleet4_cloud_run.py`, `scripts/brev/run-fleet4.sh`, 문서·Git·원격을 소유한다.
- 다른 세션의 UI와 변경을 되돌리지 않는다. 진행 중 UI 결과도 원래 지정 receipt에 반환한다.

## 도구 계약

CLI: `python3 scripts/brev/fleet4_prepare.py --stage <absolute existing stage> --source-root <absolute readonly source snapshot> --run-id <32hex> --source-commit <40hex>`.

표준 라이브러리만 최상위에서 import하고 Python3.12 요구를 프로젝트 모듈 import 전에 검사한다. stage는 source.tar와 source.commit, 추출된 source 폴더를 가진다. source-root는 Docker에서 /workspace/project, stage는 /workspace/fleet4-stage가 될 수 있으므로 별도 인자로 받는다. 둘의 source 내용은 Git archive proof로 결속한다. symlink·기존 inputs/results 출력 재사용을 거절한다.

기존 factory_ops.source_provenance.verify_source_archive로 실제 source.tar/raw commit/현재 source-root를 검사한다. pins는 scripts.isaac.fleet4_transport_scene.SOURCE_PINS에 새 준비도구와 두 실행기 경로를 더한다. archive가 유효한 뒤에만 기존 factory_ops.fleet_transport.prepare_bundle(stage/inputs, source_root/configs/factory-fleet4)로 입력을 생성한다.

stage/results/run/provenance를 새로 만들고 source.tar와 source.commit을 각각 source.tar, source-commit.raw로 복사한다. fleet-source.json에는 현재 SDK가 요구하는 run_id/scenario_id/source_commit/bundle_sha256/source_files/source_archive_sha256과 proof를 넣고 preparation_python 버전도 기록한다. 기존 transport marker는 ROS가 나중에 생성하므로 여기서는 만들지 않는다. source-root·source archive·원본 파일을 수정하지 않는다.

완료 시 stdout JSON에 passed, bundle_sha256, Python 버전, source proof 요약을 반환한다. `--stage` 출력은 새 경로만 쓴다. 물리/SDK/GPU 또는 네 대 운송 성공을 주장하지 않는다.

## 검증과 결과

Python3.10 guard가 프로젝트 import/출력 생성 전에 거절하는 검사, 실제 임시 Git archive/raw commit fixture로 소스 proof와 준비 성공, archive/source 불일치 거절, 기존 출력 재사용 거절을 검사한다. 임시 Git fixture는 로컬 전용이며 원격 생성·push를 하지 않는다. GPU·실제 VM·서버·Git commit/push·새 에이전트 호출은 하지 않는다. 전체165개 재검증은 필요 없다.

결과 `coordination/collab/results/fleet4-runtime-prepare-011.json`: task/attempt/revision/ACK, 변경 파일 SHA, 실행 검사와 실제 범위, 실패·미검증을 적는다. 수용은 root 소유다.
