# 문제 기록과 재발 방지

증상 하나마다 문서를 늘리지 않고, 원인이 같은 실패는 아래 항목에 모은다. 실행 ID·소스·관찰 근거·수정·검사·남은 제약을 함께 갱신한다. 후속 실행이 성공해도 이전 실패의 원본 판정은 유지한다.

## 상태 기준

- **미확정**: 관찰된 현상은 있으나 최초 원인을 가리지 못함.
- **수정·CPU 확인**: 코드와 회귀 검사를 확인했으며 실제 GPU 임무 검증은 남음.
- **실제 실행 확인**: 관련 동작을 실제 실행에서 확인함. 전체 임무 승인과 구분한다.
- **외부 장애**: 서비스·권한 등 저장소 밖의 문제로 해결 여부를 확인하지 못함.

| ID | 문제 | 근거 실행 | 상태 | 재발 방지 |
|---|---|---|---|---|
| FOT-001 | 물류 시각 상태를 최신 파일 하나로 전달하면 중간 인계 상태를 놓침 | q021 → q028 | q028 발행/적용16개 일치; 전체 QA는 FOT-011에서 실패 | 변경 불가능한 순번별 상태 스트림, 순서·해시 검사, 발행/적용 전수 대조 |
| FOT-002 | 목표 자세 통과 시 움직이는 상태로 성공하여 감속 중 방향 초과 | q022 → q028 | q028 주행62개·정지62개·인계8회 확인; 전체 QA는 FOT-011에서 실패 | stateful 기본 판정과 실제 정지 속도 결합, 0 Twist 감속 후보, 원래 HOLD·정지·예약 검사 유지 |
| FOT-003 | 네 raw LaserScan 입력이 같은 시각에 끊기고 AMCL TF가 오래됨 | q023 | 최초 원인 미확정 | GMO 획득과 ROS 전달을 나누어 관찰, raw header와 TF 신선도 검사; TTL 완화 금지 |
| FOT-004 | 단발 촬영 뒤 Replicator가 STEPPED에 남아 연속 ROS writer가 재개되지 않음 | q024 → q028 | q028 네 scan·AMCL 입력의4.47~1094.93초 범위 대조 통과 | zero-delta 촬영의 pause/resume 복원, PLAY 이벤트 회귀 검사, 촬영 이후 네 raw stream의 증가 확인 |
| FOT-005 | Brev Stop이 AWS invalid security token 400으로 실패 | industrial-040 → 049 | 049의 Stop 요청 exit0 후03:15:34 UTC 실제 STOPPED 확인 | 명령 수락과 실제 중지 구분, 백업 후 실제 상태 확인; 새 인스턴스·강제 종료로 우회하지 않음 |
| FOT-006 | 진단 qualification에 24초 연속 촬영이 다시 켜짐 | q024 설정 | 기본 비활성화 수정; 최종 미디어와 진단 구분 | v4의 `real_time_media.profiles=[]`, compact session 검사; 최종 촬영 전 별도 활성화·소스 결속 |
| FOT-007 | pause 요청 직후 is_playing을 확인해 실제 첫 촬영 전에 실패 | q025 → q028 | q028 촬영127시점·RGB381장 검사 통과 | 즉시 상태를 전제로 한 검사 제거, 지연 pause/play 상태 모델 검사, 기존 time/frame/pose 불변 검사 유지 |
| FOT-008 | 실행 중 SQLite 진단 조회가 reader lock을 만들어 rosbag 기록기 종료 | q026 → q028 | 조회기3개 차단·guard 검사3개 통과; q027/q028 기록기 정상 종료 | repo AGENTS의 live SQL 금지, 실행 중 ROS 토픽 구독/상태 저널만 사용, bag 분석은 기록기 종료 후 수행 |
| FOT-009 | 현재 XY 경계 재검사가 확인된 stateful 도착 상태를 해제해 회전 방향 반복 전환 | q027 → q028 | C++12개/Python34개 통과; q028 두 주문 출하·주차 완료 | 확인된 XY 상태와 현재 XY 진단 분리, stock 판정과 정지 조건 결합, confirmed/unconfirmed·stateful/stateless·reset 검사 |
| FOT-010 | 전체 종료 이벤트 수를 개별 출하 주문 수로 해석 | q028 관찰 도구 | 기존 원장의 출하1건/2건과 대조해 수정 확인 | accepted·nonduplicate OUT drop의 고유 order ID로 집계, 전체 종료 이벤트와 필드 분리 |
| FOT-011 | stock RPP를 호출하지 않는 감속 정지 구간에도 최신 lookahead를 요구 | q028 | CPU35개와 닫힌 원본62개 경로의 관측 사후 검사 통과; 원래 전체 QA false 유지 | stock 관측과 terminal-stop trace의 역할 구분, action/epoch·0 Twist·정지 성공 결속; 단순 신선도 한계 확대 금지 |

## FOT-002 — 정지 전에 도착 성공

q022의 해당 FollowPath 성공 시 XY 오차는 약0.060m, yaw 오차는 약0.148rad였지만 선속도는 약0.058m/s, 각속도는 약−0.381rad/s였다. 이후 0 명령을 받고 감속하는 동안 추가 회전해 정지 후 도착 검사를 통과하지 못했다. 오래된 TF나 이전 XY latch 이탈로 설명하지 않는다.

정지 후보는 Nav2 기본 판정기가 허용한 자세에서 감속을 요청하며, 선속도·각속도가 각각0.02 이하가 되어야 성공을 허용한다. 최초 수정의 CPU C++11개와 Python73개는 통과했으나, 현재 XY를 매번 재검사한 부분이 기본 stateful 의미와 충돌하는 FOT-009를 만들었다. 확인된 XY 도착 상태를 보존하면서 방향·실제 정지를 확인하도록 함께 수정한다. q023/q024는 도착 단계에 이르지 못했으므로 실제 수정 성공의 근거가 아니다.

- 회귀 검사: `RequiresStoppedVelocityAfterStockAcceptanceAndPreservesStatefulXy`, `test_controller_trace_validates_mixed_stock_and_terminal_stop_cycles`.
- 진단: `artifacts/brev/q022-arrival-diagnosis-001.json`.
- 검사 기록: `artifacts/brev/terminal-stop-038-validation-receipt.json`.

## FOT-003 — 원인 미확정인 입력 중단

q023에서 네 raw scan의 마지막 header 시각은14.3초, AMCL의 마지막 map→odom 시각은14.8초였다. clock·odom은 이후에도 진행했다. 0.5초 위치 신선도 검사가 안전 종료한 것은 정상 동작이다. 촬영 근처에서 발생했으나 같은 촬영 코드로 q022의 센서 입력은 더 오래 유지됐으므로 촬영을 확정 원인으로 기록하지 않는다.

GMO header의 증가만으로 ROS writer 발행이나 외부 DDS 수신을 증명할 수 없다. `has_data=true`도 신선도 판정으로 사용하지 않는다. 이후 실행에서 raw scan의 실제 header·유효 포인트·수신 범위를 대조한다.

- 진단: `artifacts/brev/q023-localization-diagnosis-001.json`.
- 원본 보존: `industrial-040-q023-canonical-001`, D/E 복사·정확한 복원·source124개 pin 검사 통과.

## FOT-004 — 재생 유지가 writer 재시작을 막은 회귀

q024는 초기 Isaac 내부 raw scan 준비를 통과했다. 이후 GMO frame·timestamp는 네 센서에서 계속 증가했지만, 외부 bag의 네 raw scan topic은 모두0건이었다. bag은 분할되지 않았고 clock·odom·TF는 기록됐다. 따라서 GPU 센서가 멈췄다고 단정하지 않는다.

설치된 Replicator의 단발 `step()`은 완료 시 STEPPED 상태가 된다. 일반 `app.update()`에서 연속 writer를 평가하는 상태는 STARTED/STEPPING이며, STEPPED에서 PLAY 이벤트를 받아야 STARTED로 돌아간다. `captureOnPlay=True` 설정만 유지해도 이 상태 전환을 대신하지 못한다. q024의 `pause_timeline=False`와 재생 유지 정책은 필요한 PLAY 이벤트를 없앴다. 추가 GMO annotator 연결은 기존 writer를 detach/recreate하지 않는다.

수정은 main loop를 확대 변경하지 않고 촬영 경계의 pause/resume을 복원한다. 촬영은 `delta_time=0`, `rt_subframes=4`, `pause_timeline=True`로 수행하고, RGB와 time/frame/pose 불변 검사를 마친 뒤 outer timeline.play가 연속 writer를 다시 시작한다. 이전 q023의 미확정 중단까지 해결했다고 해석하지 않는다.

- 회귀 검사: `test_manual_capture_rearms_writer_with_deferred_timeline_commands`, `test_rgb_capture_rejects_inactive_products_and_missing_timeline`, 기존 촬영 전후 물리 상태 검사.
- 진단: `artifacts/brev/q024-writer-boundary-diagnosis-001.json` 및 `q024-installed-sdk-excerpt.txt`(SHA256 `0f19b9f2…bbf96175`).
- 원본 보존: `industrial-041-q024-canonical-001`, 741파일·501,693,840B, D/E 복사·정확한 복원·source124개 pin 검사 통과.
- 수정 완료 조건: 여러 번의 실제 촬영 이후에도 네 raw scan header가 증가하고 AMCL 입력이 유지되며, 정상 임무와 닫힌 bag 검사가 통과해야 한다. CPU 상태 모델만으로 이 조건을 충족했다고 표시하지 않는다.
- q028에서127개 촬영 시점 이후에도 네 raw→filtered scan과 AMCL 입력을4.4667~1094.9333초 범위에서 대조했다. 각 raw 입력10,901건의 대응 검사와 bag integrity가 통과했다. 정상 임무는 완료했으나 전체 QA는 별도 FOT-011에서 실패했다.

## FOT-005 — 중지 요청과 실제 상태의 구분

이전 웹·CLI 중지는 AWS `UnrecognizedClientException`·HTTP400으로 실패했고 후속 상태는 RUNNING이었다. q028 원본과 사후 검사 자료를 D/E에 보존하고 원격 컨테이너·GPU 프로세스가 없음을 확인한 뒤 industrial-049에서 한 번 재시도했다. 2026-10-19 03:08:22 UTC의 명령은 exit0으로 수락됐다. 03:14:17 UTC의 후속 CLI JSON은 STOPPING·shell NOT READY였고,03:15:34 UTC에는 실제 STOPPED를 확인했다.

수락이 가능해진 외부 원인은 검증되지 않았다. 저장소 수정이 AWS 자격증명을 복구했다고 주장하지 않는다. 중지 명령을 반복하거나 OS poweroff·인스턴스 삭제로 우회하지 않았으며, 요청 수락만으로 GPU 중지나 과금 종료를 확정하지 않는다.

- 요청: `artifacts/brev/industrial-049-stop-attempt-001.json`.
- 후속 상태: `artifacts/brev/industrial-049-post-stop-observation-006.json`.

## FOT-007 — 요청과 반영된 상태의 구분

q025는 outer `timeline.pause()` 호출 직후에도 `is_playing()`이 true인 상태에서 첫 촬영 hook에 들어왔다. 새로 넣은 즉시 정지 확인이 Replicator step 실행 전에 실패시켰다. 이전 CPU fake는 pause/play를 즉시 반영했으므로 이 차이를 검출하지 못했다. q025는 센서 출력 복원 수정의 성공이나 실패를 검증한 실행으로 세지 않는다.

상태 모델을 명령 요청과 update에서의 반영으로 나누었다. 촬영 hook은 즉시 boolean 대신 활성 render product와 timeline 존재를 확인하고, Replicator의 zero-delta step과 기존 time/frame/pose 검사가 촬영 동안의 물리 불변을 검사한다. 원래 신선도·정지·예약 조건은 바꾸지 않는다. q026에서 실제 촬영11회와 네 raw scan의55.6초까지 연속 수신을 확인했다. 전체 임무 승인은 남아 있다.

- 진단: `artifacts/brev/q025-deferred-pause-diagnosis-001.json`.

## FOT-008 — 읽기 전용 SQLite도 실행 중에는 잠금을 만든다

q026은 센서 입력이 복구되어 네 raw scan을 약10Hz로 수신했고, 주행 액션4개·정지4개·시각 상태 발행/적용5개를 확인했다. 그러나 에이전트의 active-bag 진단 조회 시점에 기록기가 `SqliteException: SQLite error (5): database is locked`로 종료했다(exit−6). 임무는 의존 프로세스 종료를 감지해 중단됐다. 이는 AMR·LiDAR 수정의 실패를 증명하지 않으며, 전체 성공으로 해석할 수도 없다.

진단 연결의 `mode=ro`와 빠른 fetch/close가 충분한 보호라는 판단이 잘못됐다. writer가 SQLITE_BUSY를 처리하지 못하면 짧은 reader lock도 실행을 중단시킬 수 있다. 세 live SQL 조회기를 실행 차단하고 원래 코드와 해시를 별도로 보존했다. 이 규칙은 저장소 `AGENTS.md`에도 넣었다. 이후 실행 중에는 원장·상태 저널과 bounded ROS 구독을 사용하며, SQL 검사는 기록기 종료 후에 수행한다. 잠금을 우회하거나 쓰는 중인 DB 사본을 검증 근거로 사용하지 않는다.

- 진단: `artifacts/brev/q026-live-sqlite-lock-diagnosis-001.json`.
- 조회기 차단·원본 해시: `artifacts/brev/industrial-047-retired-live-sqlite-001.json`.
- 차단 검사: `artifacts/brev/industrial-047-operational-guard-tests-001.json`, 네트워크 호출 이전 차단3개 통과.
- q027은 같은 물리·주행 생산자 소스에서 JSON/JSONL만 관찰했고 기록기가 정상 종료했다. 닫힌 bag으로 FOT-009를 분석했다. q027의 별도 주행 실패와 q026의 원래 판정은 유지한다.

## FOT-009 — 확인된 XY 도착 상태를 현재 위치 검사로 해제

q027에서 두 AMR은 S1/S3 픽업 위치에 도착한 뒤 방향을 반복해서 바꿨다. carter1의 FollowPath는 약85초에 status6·error105(`FAILED_TO_MAKE_PROGRESS`)로 종료했다. goal의 map yaw0이 odom 좌표계에서 약−1.59rad로 변환된 것은 일관되며, 고정된 방향 변환 오류를 원인으로 삼지 않는다.

carter1은55.55초에 XY 오차0.059832m로 기본 SimpleGoalChecker의 XY 도착 상태를 확인했다. 이후 stateful 검사 상태는 유지됐지만, RPP wrapper가 현재 추정 XY가6cm 밖일 때마다 회전 도착 상태를 해제했다. 이 액션에서 현재 XY는 경계를13회 오갔고 RPP latch는12회 전환했다. 경로 방향 추종과 최종 yaw 회전이 교대로 선택됐다. 또한82.25초에는 기본 판정이 true이고 yaw 오차가−0.143rad였지만 현재 XY0.063551m 때문에 정지 후보와 최종 성공이 모두 false였다. wrapper만 수정해도 정지 판정이 같은 문제를 남길 수 있다.

수정 범위는 확인되지 않은 stale RPP latch만 해제하고, 정지 후보·최종 성공을 기본 판정 결과와 실제 속도에 결합하는 것이다. 현재 XY와 현재 자세의 범위 판정은 진단 값으로 남긴다. 최초 XY6cm·yaw0.15rad와 속도 한계, 독립 station0.25m/0.25rad, HOLD ACK·신선한0.5초 연속 정지·예약 조건은 유지한다.

- 재현 검사의 핵심: 확인된 XY 이후 현재 XY가 경계 밖으로 이동해도 stateful 회전과 정지 후보를 보존한다. 확인 전 stale latch·stateless XY 거부·yaw 거부·reset은 별도로 검사한다.
- 판정·trace 검사의 핵심: 기본 판정이 false이면 정지 후보와 성공을 허용하지 않는다. 기본 판정이 true여도 움직이는 동안 성공을 막고, 정지 후보의0 Twist와 trace 순서·소스 결속을 대조한다.
- 원본 보존: `industrial-047-q027-canonical-001`, 777파일·900,960,026B, D/E 정확한 복원·source124개 pin·Git fsck 통과. 보존 성공은 임무 성공을 뜻하지 않는다.
- 진단: `artifacts/brev/q027-stateful-xy-latch-diagnosis-001.json`(SHA256 `c9e5eae5…993ce2`), 닫힌 bag 집계 `artifacts/brev/q027-latch-aggregate-001.jsonl`.
- 수정: RPP wrapper는 `!xyConfirmed()`인 경우에만 상태를 해제한다. 정지 후보는 `enabled && base_result && !velocity_stopped`, 성공은 `base_result && (!enabled || velocity_stopped)`로 판정한다. trace 검사기도 같은 기본 판정과 결속한다.
- 회귀 검사: `ReleasesOnlyUnconfirmedStaleLatchAndPreservesConfirmedRotation`, `RequiresStoppedVelocityAfterStockAcceptanceAndPreservesStatefulXy`, `StatelessCheckerRejectsCurrentXyOutsideTolerance`와 `test_controller_trace_validates_mixed_stock_and_terminal_stop_cycles`.
- CPU C++12개·Python34개가 통과했다. 검증 기록·전체 C++ 로그 해시는 `artifacts/brev/terminal-stop-049-validation-receipt.json`에 남겼다. q028은 수정 소스012dc57에서62개 주행·정지,8회 인계,2건 출하·최종 주차를 완료했다. 전체 QA는 FOT-011에서 실패했으며 원래 판정을 보존한다.
- CPU 하네스의 첫 시도는 `set -u`와 ROS `setup.bash` 충돌로 빌드 전에 종료했다. ROS 환경 로딩에는 `set -eo pipefail`을 사용하고 최종 소스를 새로 빌드해 위 검사를 통과했다. 주행 실패 원인과 구분한다.

## FOT-010 — 이벤트 수와 출하 주문 수의 혼동

관찰 도구가 `orders_completed`라는 전체 주문 종료 이벤트의 개수를 개별 출하 건수처럼 표시했다. 첫 주문은 이미 OUT 인계를 마쳤지만 전체 종료 이벤트는0개였으므로 진행 보고가 늦었다. 생산자나 원본 원장의 오류가 아니다.

필드를 `all_orders_completed_events`로 바꾸고, `shipped_orders`는 outbound·OUT·drop 인계 중 accepted=true·duplicate=false인 고유 order ID로 집계한다. 기존 JSONL 원장에서 출하1건과 최종2건을 대조했으며, 주행 중 SQL 연결을 추가하지 않았다.

- 근거: `artifacts/brev/q028-first-shipping-observation-001.json`, `q028-observer-shipping-count-fix-001.json`.

## FOT-011 — 감속 정지와 stock RPP 관측의 계약 불일치

q028의 mission과 두 worker는 정상 종료했으나, 원래 전체 QA는 `FollowPath terminal has no adjacent lookahead sample`에서 실패했다. 검사는 첫 예외에서 종료하므로 오류 문구1개가 영향받은 액션1개를 뜻하지 않는다. 닫힌 bag에서62개 액션을 조사한 결과13개의 마지막 lookahead와 종료 사이 간격은 약0.53~0.92초였다.

이 구간의 `terminal_stop_candidate`는 stock RPP를 호출하지 않고0 Twist를 반환한다. 따라서 stock RPP가 발행하는 lookahead/rotation pair도 새로 생성되지 않는다. 제어 trace는 계속 기록됐고 해당 액션들의 최종 goal check는 `effective_result=true`, `velocity_stopped=true`였다. 원래 Nav2 trace 검사는 stock 호출/관측의1:1 대응과 두 로봇의 terminal-stop 호출286개·405개를 통과했다. 이 동작에 종료 직전 stock lookahead를 필수로 요구한 검사가 충돌했다.

검증기는 주행 중 stock 관측의 신선도를 유지하면서 감속 구간을 같은 액션·epoch의 제어 trace, 기본 도착 판정,0 Twist, 신선한 정지 성공과 결속한다. 기존 heavy trace 검사의 결과를 재사용하며 중복 실행하지 않는다. 신선도·주행 허용치나 원본 결과를 바꿔 통과시키지 않는다. 누락·nonzero·위조 확인·다른 액션/epoch·오래된 승인·순번 간격을 포함한 CPU35개와 pyright 오류0개, 별도 검증 패스를 통과했다.

같은 닫힌 원본을 새 판정 코드로 읽은 사후 검사는62개 경로·오류0개였고 그중13개는 terminal-stop trace로 확인했다. bag 전체와 원래 보고서를 전후 대조해 입력 불변을 확인했다. 이 검사는 제어·RPP 관측 범위이며 전체 qualification의 원래 false를 변경하지 않는다. 후속 전체 검증은 남아 있다. 사후 검사 도구의 첫 시도는 파일을 `/validator.py`에 마운트해 모듈의 `parents[2]` 경로 계산에서 bag을 읽기 전에 실패했다. `/postprocessor/scripts/ros/g3_evidence.py`로 마운트 위치를 고친 별도 시도에서 검사를 완료했으며 생산자 코드는 바꾸지 않았다.

- 진단: `artifacts/brev/q028-terminal-observation-diagnostic-001.json`.
- 사후 검사: `artifacts/brev/q028-terminal-observation-recheck-051-002.json`, 공개용 사본은 `docs/images/factory-v4-q028/observation-recheck.json`.
- 원본 보존: `industrial-049-q028-canonical-001`,1,164파일·6,356,506,539B. D/E 복사·정확한 복원·source124개 pin·Git fsck와 증명 파일8개 해시 대조를 통과했다.

## FOT-012 — 웹3D의 중첩 React DOM 루트 종료 오류

첫 웹 검수에서 상태 패널과 API는 동작했으나3D 영역이 로딩 표시를 유지했다. Chrome 콘솔에 `Attempted to synchronously unmount a root while React was already rendering`와 `NotFoundError: removeChild`가 기록됐고 스택은 Drei `Html.useLayoutEffect`를 포함했다. 이를 물리 시뮬레이션 또는 API 실패로 해석하지 않는다.

장면의 Drei Html 라벨을 CanvasTexture sprite로 바꿨다. 이후 페이지를 새로 열어 공장 geometry와 AMR raycast 선택·목록 선택·키보드 선택을 확인했고, 기존 오류의 마지막 기록 이후 새 오류는 관측하지 않았다. 모바일390px·중간 폭900px의 가로 넘침도 없었다. React/R3F 전체 호환성이 보장됐다는 판정으로 확대하지 않는다. 재발 검사는 실제 브라우저에서 장면 로드·선택 변경·주기적 상태 갱신·콘솔을 함께 확인한다. 타입 검사와 빌드 성공만으로3D 표시 완료를 판정하지 않는다.

## FOT-013 — 웹 상태·명령의 실행 경계와 표시 오류 격리

API 정적 검수에서 서로 다른 실행의 동일 명령 ID가 ACK와 결합될 수 있는 경로, ACK 적용 버전 누락, 구조가 손상된 상태 파일의 캐시 가능성을 확인했다. 선택형 ROS 웹 발행기에도 직렬화/projection 오류가 물리 임무까지 전파될 수 있는 경로가 있었다. 이들은 실제 GPU 사고로 관측한 내용이 아닌 코드 검수·CPU 오류 주입으로 확인한 문제다.

명령을 실행 ID와 제출 버전에 결속하고, 실행 전환 시 이전 요청을 만료시키며 이전 실행 ID 재사용을 거절한다. ACK의 실행·버전 범위를 검사한 뒤 적용 버전을 반영한다. 프런트가 소비하는 상태 구조·숫자·null 계약을 입력에서 검사하고 손상된 파일은 마지막 정상 관측을 disconnected로 유지한다. NaN 속도는 미관측 null로 투영하며, 발행 경계의 예상 가능한 파일·자료 오류를 진단 후 비활성화해 제어 경로와 격리한다.

회귀 근거는 `tests/test_dashboard.py`의 실행 전환/ACK·손상 snapshot·NaN/projection 격리·읽기 전용 ROS 투영 검사다. 관련59개 테스트 통과, 신규 Python 모듈·테스트의 Pyright 오류0개를 확인했다. G3 main의 분석 복잡도 오류1개와 yaml source 경고1개는 기준c67c2da에서도 재현되며 이번 기능의 신규 오류로 기록하지 않는다. 실제 ROS/GPU 웹 연동과 물리 명령 handler는 아직 검증하지 않았다.

## FOT-014 — 웹 운영의 재사용 토트와 제품 ID 혼용

웹 CPU 모델은 생산 원장의 제품 payload와 재사용 토트에 같은 ID를 사용했고, 자재 소비 후 빈 용기를 회수하는 운송 작업이 없었다. 토트 풀이 소진되면 후속 주문을 받을 수 없었으며, 생산 원장의 제품 ID 유일성 조건 때문에 같은 토트를 새 주문에 재등록할 수도 없었다. 이는 Isaac Sim 실행에서 발생한 물리 오류가 아닌 CPU 모델의 운영 연결 누락이다.

제품 LOT와 토트 ID를 분리했다. 소비 후 CI에서 원래 자재 위치로 가는 회수 작업을 공급·출하와 시간순으로 배정하고, 반납 경로가 끝난 뒤에만 용기를 가용으로 바꾼다. source별 초기 토트 위치를 설정하고 다른 자재 위치의 토트를 예약하는 요청을 거절한다. 회수 중인 차량은 제품 배차에서 제외하며, 이벤트 ID는 제한 길이 로그에서도 중복되지 않도록 별도 순번을 쓴다.

재발 검사는 `tests/test_dashboard_model.py`의16개 토트·20개 주문 실행, 매 tick의 중복 배차/적재·수량 보존, 반납 중 pause와 목적지 도착 후 가용 전환이다. 관련135개 CPU 테스트와 변경 Python 모듈의 Pyright 오류0개를 확인했다. 실제 웹에서도 빈 토트 반납·적재 ID·목적지·경로를 확인했다. 이 모델의 이동은 경로 보간이며 교통 회피·물리 적재·충전·반납 중 재시작 복구는 완료 범위가 아니다.

## FOT-015 — 4대 개발 시험의 시각 설정·센서 실행 결속 누락

4대 정지 센서 시험의 실행 전 소스 검수에서 두 공백을 확인했다. 준비된 입력 묶음에 시각 설정을 보존했지만 실제 시각 자산 생성기는 저장소의 설정을 다시 읽었다. 센서 결과 검증기도 로봇·센서 이름을 확인하면서 실행 ID·시나리오·센서 prim 경로를 대조하지 않았다. 실제 GPU 오류로 관측한 사건은 아니다.

선택 설정 디렉터리를 시각 자산 생성까지 전달하고 실제 사용한 visual/render 입력 해시를 실행 결과와 입력 묶음 사이에서 대조한다. 기존2대 경로는 이전 설정 선택을 유지한다. 센서 관측과 완료 사건은 같은 실행·시나리오·정확한 센서 prim 및4대 제어 표본 집합에 결속한다. 다른 실행·prim, 시각 설정 해시 불일치, 부족한 센서 메시지와 진행하지 않는 GMO 헤더를 거절하는 CPU 검사를 수행했다. 관련108개 회귀검사와 신규 Python 모듈의 Pyright 오류0개를 확인했다. 실제4대 GPU 센서 시험·운송·물리 적재·충전 검증은 별도 상태로 기록한다.

## FOT-016 — 4대 개발 실행의 출력 폴더 준비 누락

`fleet4-stationary-smoke-001`은 공유 worker가 기존의 빈 출력 디렉터리를 요구하는데 개발 진입점이 해당 디렉터리를 만들지 않아 `FileNotFoundError: /workspace/fleet4-results/run`으로 종료됐다. 컨테이너 종료 코드는1이며 Isaac SimulationApp 초기화 전 실패다.4대 차체·센서나 유효한 렌더 자료를 생성한 실행으로 집계하지 않는다. 생산자60513d9와 실패 로그·소스 묶음은 D/E에 같은 해시로 보존했다.

개발 진입점은 출력 경로의 절대경로·심볼릭 링크 여부와 입력 씬 일치를 검사한 뒤 폴더를 준비하고 기존 공유 worker의 빈 디렉터리 검사를 유지한다. 새 출력 생성, 기존 파일 보존과 비어 있지 않은 출력 거절, 상대경로 거절을 실제 경로 helper로 검사했다. 이 수정의 관련65개 CPU 테스트와 타입 검사를 통과했다. 재시도는 새 실행 ID·출력 경로를 사용하며 원래 실패를 덮지 않는다. 최종 성공은 컨테이너 종료 코드와 실행 ID가 맞는 센서 검증 결과 파일을 함께 확인한다.

## FOT-017 — SDK 종료 뒤 같은 프로세스에 둔 후속 검증

`fleet4-stationary-smoke-002`는4대·8개 센서를 구성하고 공유 worker의 `status=validated`, `cleanup=worker_exit_requested`, 컨테이너 종료 코드0을 남겼다. 그러나 `SimulationApp.close()` 뒤에 둔 검증 파일 생성은 실행되지 않아 외부 수집기가 파일 누락을 기록했다. NVIDIA [SimulationApp API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/index.html)는 fast shutdown이 프로세스를 종료하는 경로를 설명한다. 또한 새 검증기는 공유 worker의 실제 완료값 `validated` 대신 `passed`를 가정했고 CPU 양성 fixture도 같은 잘못된 값을 사용했다.

후속 검증을 SDK 밖의 CPU CLI로 옮기고 실제 worker 상태·정리 상태를 대조한다. 원래 수집기의 파일 누락 실패와 원본은 유지했다. 별도 폐쇄 원본 검증에서 입력·시각 설정·실행·센서 prim 결속,8개 GMO 헤더 진행과 ROS 메시지, 차량별273개 표본의 정지 한도와0인 적용 명령을 확인했다. 실제 원본의 재검증은 통과했으며 GPU를 다시 실행하지 않았다. 초기·종료2개 시점의 RGB6장과 source438e44d를 D/E에 같은 해시로 보존했고 실제 Stop도 확인했다. 이 판정은4대 정지 센서 시험이며 운송·물리 적재·충전의 완료 근거는 아니다.

## FOT-018 — 네 대 운송 후보의 시각 계약에 남은 두 대 가정

네 대 입력을 연결하는 CPU 검사에서 화물 위치 검증이 `robot_carter1/2`만 허용하고, 새 배치 revision이 기존 생산 설비 recipe·바닥 표시 분기를 선택하지 못하는 문제를 확인했다. 바닥 글꼴에는 P4의 숫자4도 없어 새 시각 장면 생성이 실패했다. 이는 새 개발 후보를 GPU에 올리기 전에 발견한 오류이며 실제 GPU 운송 실패로 집계하지 않는다.

시각 상태 문서와 화물 배치 검증에 선언된 차량 ID를 전달하고, 미등록 차량·다른 차량의 화물 부모를 거절한다. 기본 두 대 계약은 유지한다. 새 배치에도 생산 설비 recipe를 적용하고 안전선·공정 표시는 생산 구역의 이동량에 맞추며 P1~P4 표시와 실제 마커 수를 생성한다. 물리 적재 검증 표시는 계속 false로 유지한다. 관련 시각 검사48개 중47개 통과, OpenUSD 런타임이 필요한 기존1개는 로컬 환경에서 건너뛰었으며 새 배치의 실제 렌더·센서·운송은 후속 GPU 검증 대상이다.

## FOT-019 — 개발 실행기의 종료 확인·Stop·보존 시간 경계

네 대 운송 실행기 검수에서 종료 파일의 존재만 보고 활성 bag 압축에 들어갈 수 있는 경로, nohup 응답 유실로 PID가 없을 때 종료 확인을 건너뛰는 경로, 로컬 기록/상태 조회 오류가 Stop을 생략하는 경로를 모의 재현했다. 보존 작업의 개별 timeout 합계도 선언한 전체 시간 여유보다 길었다. 이 단계에는 Brev를 시작하지 않았으며 모의 테스트의 START/STOP 문자열은 실제 클라우드 상태 근거가 아니다.

원격 PID와 실행 ID label을 기록하고, 중단 플래그·소유 프로세스·실제 컨테이너 상태를 새로 확인한 뒤에만 압축한다. 원래 worker 종료와 cleanup 실패를 따로 기록한다. Stop은 보존·보조 로그와 분리한 최외곽 finally에서 수행하며 일시적인 상태 조회 실패가 정지 요청을 막지 않게 한다. 단일 절대 시간 창 안에서 개별 작업 timeout과 파일 해시 계산을 제한하고 Stop에 쓸 시간을 남긴다. 공급자의 실제 정지 처리 시간이나 청구 상한을 보장한다고 표시하지 않는다.

초기 입력 검사와 응답 유실·활성 컨테이너 종료 기록·상태 조회 실패·로컬 기록 실패·느린 보존·시간 경계의 모의 재현을 합친10개 테스트가 통과했다. 생성된 원격 Python 문법과 Bash 문법도 확인했다. 실제 컨테이너 종료·D/E 사본·Brev STOPPED와 종료 원본 QA는 새 GPU 실행에서 확인할 항목이다.

## FOT-020 — VM 기본 Python과 프로젝트 실행 환경 불일치

`fleet4-transport-normal-001`은 VM 기본 Python3.10에서 입력 묶음을 준비하다가 `enum.StrEnum` import 오류로 종료됐다. traceback의 `/usr/lib/python3.10` 경로로 실행 환경을 확인했다. Isaac/ROS worker는 시작하지 않았고 소유 컨테이너·runner PID도 없었다. 이 시도를 네 대 운송이나 렌더 성공으로 집계하지 않는다.

프로젝트를 import하는 준비·입력 검증을 기존 ROS Jazzy 이미지의 Python3.12 CPU 컨테이너로 옮긴다. 준비 도구는 프로젝트 import 전에 버전을 검사하고 실제 Git archive·raw commit·읽기 전용 소스가 일치한 뒤 새 입력과 provenance를 만든다. VM Python은 표준 라이브러리만 사용하는 조회·정리에 쓴다. 재시도는 새 실행 ID와 출력 경로를 사용한다.

실패 원본90,540,186B는 D/E에 SHA256 `743df3424f2936fca7133e5d86d5d2a5000840cfaf6160b7d5b68a819ad1623a`로 보존했다. 실제 Brev STOPPED 확인 시각은2026-10-19 19:12:23 KST다. 실제 임시 Git archive/raw commit 양성·변조·기존 출력 보존과 버전 거절을 포함한 준비 검사9개, 실행기 오류 주입10개가 통과했다. 다음 실제 컨테이너 실행 결과는 별도 기록하며, 버전 수정만으로 운송 성공을 주장하지 않는다.

## FOT-021 — 시작 응답과 실제 실행 상태의 분리·CLI 인증 오류

`fleet4-transport-normal-002`의 시작 요청은 정상 반환했으나 CLI와 웹 화면은 계속 Stopped를 표시했다. 같은 환경의 웹 Start와 일반 CLI start도 확인했지만 실제 Starting/Running 전환이나 새 worker를 관측하지 못했다. 대기 중 CLI `/api/me`가401 Unauthorized를 반환해 실행기가 종료했다. 인증 오류는 확인된 사실이며, 앞선 무상태 전환의 원인이 같은 인증 문제인지는 확정하지 않는다. 이전 공급자400 security-token 오류와도 구분한다.

이 시도에는 원격 stage·Isaac/ROS·bag·미디어를 만들지 않았다. 생산자411670c의 소스 archive/raw commit과 host·시작·비용 관측 기록은 D/E의 `fleet4-transport-normal-002-host-proof`에 같은 해시로 보존했다. 실행기는2026-10-19 19:45:35 KST에 실제 STOPPED를 확인했다. 재로그인 화면은 Continue가 Brev·NVIDIA 약관 수락을 겸해 실행 시점 동의를 요청한 상태다. 재로그인으로 접근해야 하는 경우 약관 동의 승인 뒤에 진행한다. 재실행은 새 실행 ID와 신선한 잔액·상태를 사용하며, 응답 코드0을 실제 GPU 실행으로 집계하지 않는다.

20:00 KST의 기존 CLI 조회는 다시 정상 반환했고 같은 환경의 STOPPED를 확인했다. 새 약관 수락은 수행하지 않았다. 기존 인증 경로가 동작하면 원래 승인된 환경의 시작을 사용할 수 있으며, 인증 갱신의 약관 수락과 구분한다. 후속 실행기는 일반 start 응답과 실제 상태 전환을 별도로 보존하고 일시적인 상태 조회 오류는 세 번 연속 발생할 때 종료한다. 단일 실행 창은10800초로 줄여 앞선 두 시도의 전체 대기 시간을 보수적으로 계산해도 이번 묶음의15달러 추정 범위에 남도록 했다. 공급자의 실제 청구액을 보장하는 값은 아니다. 변경한 실행기의11개 모의 검사를 통과했으며 모의 START/STOP 출력은 실제 상태 증거가 아니다.

## FOT-022 — 충전기 분리 후 재접촉과 오래된 출차 승인

가상 충전 상태 머신의 별도 CPU 검토에서 분리 확인 후 ready_to_leave가 된 차량의 새 접촉 관측을 거절하면서 이전 승인은 유지하는 경로를 재현했다. 이 상태에서 슬롯 release가 가능했다. 실제 SDK 도킹 사고가 아닌 CPU 계약 결함이다.

출차 준비 상태도 새 접촉·움직임을 계속 처리하고, 재접촉이나 이동이 보이면 분리 대기로 되돌린다. release에는 판단 시각을 요구해 마지막 분리·정지 관측의 신선도를 재검사한다. 슬롯 소유권은 재접촉·오래된 관측에서 유지한다. 재접촉·이동·과거/오래된 release를 포함한 에너지·충전14개 검사와 타입 검사0오류를 확인했다. 이 결과는 실제 접촉 센서나 ROS 배차에 연결됐다는 증거가 아니다.

## FOT-023 — 네 번째 차량의 주차를 업무 참여로 집계할 수 있는 검증 공백

종료 JSON 검사 도구 검토에서 네 대의 성공한 경로와 최종 주차를 요구하면서도, 업무 인계는 세 대가 모두 처리한 구성을 배제하지 않는 공백을 확인했다. 네 대 모두가 승인된 업무 pickup/drop에 참여하도록 조건을 추가했다. 네 대 양성 fixture와 세 대 업무·네 번째 주차만 수행한 음성 fixture가 각각 통과/거절됨을 확인했다. 실제 운송 원본에 이 판정을 적용한 상태는 아니며, 재고·빈 토트 확장의 별도 검증을 대신하지 않는다.

## 실행 전 확인과 후속 기록

1. 깨끗한 생산자 소스·host commit·설정·실행 ID를 결속한다. 실패한 실행 ID와 원본을 다시 사용하지 않는다.
2. 진단 실행은 v4 연속 촬영 비활성화와10초 간격·상태 변화 RGB를 사용한다. 데이터 수량 확대보다 주행 완주를 우선한다.
3. 촬영 lifecycle·물리 상태 불변·compact session 검사를 수행한다. 관련 검사 통과 후에만 새 GPU 실행을 시작한다.
4. 최초 촬영과 후속 촬영 이후 네 raw scan·AMCL·액션 진행을 ROS 구독과 상태 저널로 확인한다. GMO가 정상이라는 이유로 이 검사를 생략하지 않으며 실행 중 bag SQL 조회는 하지 않는다.
5. 종료 후 이 문서의 상태와 해당 진단 파일에 실제 결과를 반영한다. QA의 연쇄 오류는 최초 실패의 영향과 구분하고, 검사가 끝나지 않은 항목은 열어 둔다.

기본 CPU 검사:

```powershell
cd "$HOME/factory-ops-twin"
python -B -m unittest tests.test_g2_scene tests.test_g3_scene tests.test_compact_session tests.test_g3_session tests.test_g3_media_evidence
```

외부 장애가 남은 동안에는 기존 인스턴스의 승인된 예산·실제 잔액·저장 공간을 확인한다. Stop 명령의 종료 코드와 VM의 실제 상태를 따로 기록하며, 서비스 장애를 프로젝트 구현 완료와 혼동하지 않는다.
