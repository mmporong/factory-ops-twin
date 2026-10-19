# fleet-ros-map-004 / attempt1 / revision4

현재 root가60513d9의4대 정지 센서 시험1회를 기존 Brev에서 실행·보존·Stop합니다. 공유 제품 소스 변경은 금지하며 실행을 건드리지 않습니다.

control은 scripts/ros/g3_mission.py, scripts/ros/g2_mission.py의 재사용 helper, 관련 tests/test_g3_mission.py 및 factory_ops/fleet.py만 읽어, 다음4대 물류 주행 단계에 필요한 최소 변경 경로를 정리합니다. 기존2대 G3 qualification/session/CDR 게이트를 완화하지 않는 별도 development 진입점과 registry 연결을 우선 비교하세요. 실제 문제는 import ROBOT_IDS, _load_ready, build_namespaced_params, runtime loops/parking/terminal ACK 등에서2대가 고정된 것입니다.

산출물은 coordination/collab/results/fleet-ros-map-004.json 하나입니다. 질문별 파일:라인, 재사용 가능 helper, 최소 수정 파일/인터페이스, 아직4대에 부족한 실제 입력·주차/traffic 조건, 의미 있는 CPU 검사와 GPU 판정 조건을 간단히 반환하세요. 리서치 확대·새 의존성·제품 코드/README 수정·GPU·commit/push·새 프로세스/에이전트 금지. 기존 세션 유지, 다른 변경을 되돌리지 않습니다. ACK와 입력 commit을 결과에 남기고 최대한 적은 변경으로 실제4대 운송에 연결될 경로를 제시하세요. accepted는 root가 판정합니다.
