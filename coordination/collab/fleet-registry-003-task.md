# fleet-registry-003 / attempt1 / contract_revision3

기준 commit d0aaee2bb3e459244b1e546dbb5e22877db975b5. 기존 control 페인을 유지합니다.

소유 경로: factory_ops/fleet_registry.py, tests/test_fleet_registry.py, coordination/collab/results/fleet-registry-003.json만 수정하세요. 공유 코드이며 다른 세션 변경을 되돌리지 않습니다. root는 scripts/isaac/g2_scene.py, 신규 fleet4_scene.py 및 개발 프로필/통합 테스트를 맡습니다. commit/push, GPU, 새 프로세스/에이전트 금지.

구현 계약: stdlib만 사용하는 FleetRegistry(config: Mapping[str,Any], scenario: Mapping[str,Any]) 클래스를 만듭니다.
- 불변 robot_ids: tuple[str,...] (config robots 순서 유지), definitions: tuple[dict[str,Any],...], spawns: dict[str,tuple[float,float,float]], sensor_keys: frozenset[tuple[str,str]] (모든 robot × front/rear)
- config['robots']는 비어 있지 않은 리스트, robot_id/namespace/prim_path 고유. ROS 이름은 [A-Za-z_][A-Za-z0-9_]*, namespace는 선행 슬래시 없는 단일 이름, prim_path는 /World/ 아래 유효한 USD identifier segment. prim path 중첩 금지.
- scenario['spawns'] 키 집합 정확히 robot_ids, 각 값3개 finite numeric bool금지, 서로 같은 XY 금지(로봇 외곽 분리 검증을 대신하지 않음).
- config['lidars']는 정확히 front/rear 각1, frame_id 고유한 유효 이름. finite translation XYZ 및 yaw 검증. 복제/누락 거부.
- __init__ 입력을 변경하지 않으며 내부 사본을 보존합니다. 유효성 실패는 ValueError.
- manifest() -> dict[str,Any]: schema_version1, robot_ids, robots(원래 definitions+spawn_xy_yaw, 각 scan raw/filtered/odom/cmd_vel topic은 namespace 기준), lidar identities/총수, scope=declared configuration only; not runtime qualification. 실제 토픽 생성 코드는 기존 Isaac bridge가 담당합니다.
- 실제 ROS/Isaac 실행 없이 검증 가능합니다. 기존 configs/factory-v4/g2.json과 g2-scenarios.json의 실제 구조를 참고하고1/2/4대, 중복 namespace/경로/좌표, 잘못된 수치, lidar 누락/중복·입력변경 방지를 검사하세요.

새 패키지나 엄격 워크플로를 추가하지 마세요. ACK 후 위 3개 경로만 구현하고 테스트·타입 확인 결과와 SHA를 receipt에 남기세요. accepted 판정은 root가 합니다. main runtime/profile의 지원 범위로 주장을 확대하지 마세요.
