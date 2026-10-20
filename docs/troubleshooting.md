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

사용자의 실행 시점 동의를 받은 뒤 기존 계정 CLI 로그인을 갱신했고 브라우저의 로그인 성공과 잔액336.31달러를 확인했다. `fleet4-transport-normal-003`에서도 start는 정상 반환했으나16분24초의 제한된 대기 동안 실제 Running 전환이 없었다. 웹 History는19:11:56 이후 Stopped를 유지했고 새 시작 이력도 없었다. 로그인 갱신은 성공했지만 환경 시작 장애는 해결되지 않았으며, 공급자 측 내부 원인은 확정하지 않는다.

003은 원격 stage·Isaac/ROS·bag·미디어를 만들기 전에 종료했다. 실행기는20:20:28 KST에 STOPPED를 확인했고 호스트 PID63536도 사라졌다. 생산자9d9a67c의 source archive/raw commit, host·비용·재로그인 후 시작 기록8개를 D/E의 `fleet4-transport-normal-003-host-proof`에 바이트 수와 SHA256을 대조해 보존했다. 같은 실패를 반복하기 위한 새 환경·삭제·reset은 수행하지 않는다. 외부 상태가 바뀌어 실제 시작이 가능한 경우에만 새로운 실행 ID로 이어간다. 이 실패를4대 운송·충전·80% 달성으로 집계하지 않는다.

종료 후 명시적으로 수행한 `brev stop factory-ops-twin-g0`는 종료 코드1과 AWS `400 UnrecognizedClientException: The security token included in the request is invalid`를 반환했다. 이후 `brev ls --json`은 같은 환경의 `STOPPED / COMPLETED / NOT READY / UNHEALTHY`를 반환했다. Brev 계정 로그인 성공, 현재 정지 조회, 공급자 Stop 요청 실패를 별도 사실로 기록한다. AWS 제어에 사용된 토큰의 유효성 오류는 관측됐지만 토큰 발급·보관 주체나 시작 실패의 내부 원인까지 확인한 것은 아니다. 재로그인을 다시 반복하거나 새 환경을 만드는 해결책을 실행하지 않는다.

## FOT-022 — 충전기 분리 후 재접촉과 오래된 출차 승인

가상 충전 상태 머신의 별도 CPU 검토에서 분리 확인 후 ready_to_leave가 된 차량의 새 접촉 관측을 거절하면서 이전 승인은 유지하는 경로를 재현했다. 이 상태에서 슬롯 release가 가능했다. 실제 SDK 도킹 사고가 아닌 CPU 계약 결함이다.

출차 준비 상태도 새 접촉·움직임을 계속 처리하고, 재접촉이나 이동이 보이면 분리 대기로 되돌린다. release에는 판단 시각을 요구해 마지막 분리·정지 관측의 신선도를 재검사한다. 슬롯 소유권은 재접촉·오래된 관측에서 유지한다. 재접촉·이동·과거/오래된 release를 포함한 에너지·충전14개 검사와 타입 검사0오류를 확인했다. 이 결과는 실제 접촉 센서나 ROS 배차에 연결됐다는 증거가 아니다.

## FOT-023 — 네 번째 차량의 주차를 업무 참여로 집계할 수 있는 검증 공백

종료 JSON 검사 도구 검토에서 네 대의 성공한 경로와 최종 주차를 요구하면서도, 업무 인계는 세 대가 모두 처리한 구성을 배제하지 않는 공백을 확인했다. 네 대 모두가 승인된 업무 pickup/drop에 참여하도록 조건을 추가했다. 네 대 양성 fixture와 세 대 업무·네 번째 주차만 수행한 음성 fixture가 각각 통과/거절됨을 확인했다. 실제 운송 원본에 이 판정을 적용한 상태는 아니며, 재고·빈 토트 확장의 별도 검증을 대신하지 않는다.

## FOT-024 — AWS 공급자 서비스 중단 공지와 대체 환경 준비

2026-10-19 밤 Brev 콘솔에 AWS 공급자를 일시 중단했다는 공식 배너가 표시됐다. 공지는 기존 AWS 환경의 start·stop·delete가 모두 불가능하다고 안내한다. 기존 환경은 계속 Stopped이며, 후속 Stop 명령은 stopped→stopping 전환 불가를 반환했다. 앞서 수집한401·AWS400·시작 미전환 기록은 보존한다. 현재 서비스 중단 공지는 확인됐지만 각 과거 실패의 내부 원인을 동일하다고 소급 확정하지 않는다.

대체 생성 화면에서 GCP L4 구성을 확인했으나 Deploy는 실행하지 않았다. 사용자가 이어서 기존 환경만 쓰도록 지시해 신규 생성과 대체 대상 실행기 변경을 취소했다. 이후 실제 CLI 조회에는 기존 `factory-ops-twin-g0` 한 개만 표시됐고 상태는 STOPPED·UNHEALTHY였다. 기존 AWS 원본을 보존하며 공급자 장애 중 반복 Start를 하지 않는다. 로컬 운영 구현을 진행하고, 기존 환경이 정상 사용 가능한 상태가 된 뒤 새 실행 ID로 GPU 검증을 재개한다.

## FOT-025 — CPU 충전 모드의 tick revision과 실행 ID 재사용

선택형 충전 모델이 매 tick command revision을 올려, 같은 사건 목록에서0.1초만 진행해도 이전 화면의 pause 요청을 거절하는 현상을 재현했다. 명령 CAS revision은 이동·SOC 갱신과 분리하고 기존 API의 명령 적용 때 갱신하도록 유지한다. 실제 loopback HTTP로 관측 사이 이동 후 pause가 승인되고 시간·위치·SOC·충전 상태가 동결되며 resume 뒤 다시 진행하는 회귀검사를 통과했다.

모든 CPU 모델이 고정된 `web-local-model` 실행 ID를 쓰던 부분도 초기 Ledger 생성 때 UUID를 부여하도록 바꿨다. 초기 사건 생성 뒤 ID를 바꾸지 않는다. API 재시작의 두 모델이 다른 ID를 쓰고 각각 원장 재생이 일치하는지 확인했다. 모델·API 관련29개 검사와 웹 타입/프로덕션 빌드가 통과했다. 이 결과는 CPU 관제 계약이며 실제 Isaac/ROS 주행 검증으로 집계하지 않는다.

후속 UI 검수에서는 일시정지된 차량에도 보존된 충전 단계가 현재 상태보다 우선 표시되는 문제를 확인했다. 목록·상세는 일시정지를 우선하고, 충전대·작업행에는 일시정지를 함께 표시한다. 브라우저에서 pause/resume 명령의 applied revision1·2를 확인했고 재개 뒤 같은 실행 ID의 충전 진행을 관측했다. 정지 중 SOC·위치·단계 보존은 HTTP 회귀검사에서도 확인했다.

## FOT-026 — 충전 동안 공용 출구를 점유하던 CPU 배차 병목

CPU 충전 슬롯 예약과 동시에 공용 출구 M_LOW를 점유해, 다른 차량의 정상 S1/S2 운송이 충전 내내 배정되지 않는 현상을 재현했다. 슬롯 소유권과 출구 홀드 획득 시점을 분리했다. 충전 중에는 공용 출구를 열어 두고, 목표 SOC에 도달하면 이미 수락된 업무·빈 토트 반납의 현재 leg가 끝날 때까지 패드에서 기다린다. 실제 출차 직전에 출구 홀드를 얻고 패드 밖 정지·분리가 확인된 뒤 반환한다.

충전 중 S1/S2 배차와 업무·빈 반납의 출구 통과를 검사하는 두 회귀검사를 추가했다. 충전·API·기존 운영 관련31개 검사가 통과했다. 이는 CPU 경로 모델의 운영 검증이며 실제 충돌·Nav2·물리 충전의 검증은 남아 있다.

## FOT-027 — 빈 토트 반납을 제품 인계 원장에 요구하던 종료 검증

기존 종료 검증은 모든 action handoff가 제품 임무의 navigation journal에 있어야 한다고 요구했다. MaterialFlow가 추가한 빈 반납 사건은 별도 원장이므로 이 계약과 맞지 않았다. 제품 task와 선언된 empty task를 분리하고, 알 수 없는 task는 거절하도록 수정했다. 유한 재고는 수락된 제품 픽업/하역, 재생 가능한 공정 시작과 두 반납 사건을 시간순으로 적용해 최종 재고·소비·토트·적재 상태와 대조한다. completed/balanced 플래그만으로 통과하지 않는다.

생산자 형식의 모의 종료 기록에서 정상·원본 불변, 누락된 반납 인계·잘못된 차량/목적지·조작된 재고와 토트를 포함한17개 검사가 통과했다. 최종 소스와 새 에너지 설정을 사용한 별도 양성 검사도 통과했다. 실제 GPU 종료 원본에 대한 검증 성공은 아직 없다.

후속 검수 ER017-001은 빈 반납의 attempt와 NavigationResult pose를 검사하지 않던 경계를 재현했다. attempt0과 map 프레임·유한 좌표·시각·목표 도착 허용치를 기존 NavigationResult/validate_goal_pose 계약으로 검사한다. 정상 및 여섯 변조 사례의 두 회귀검사가 통과했고 지정 수정 경계의 별도 작성자 재대조도 마쳤다.

## FOT-028 — 가상 SOC 표시와 실제 ROS 배차 제한의 연결

기존 ROS 경로는 오도메트리 기반 SOC를 표시했으나 새 운송의 에너지 예산을 배차에서 사용하지 않았다. 업무와 빈 토트 배차의 cost callback에 전체 픽업·하역·충전대 복귀와 회전/대기/예비 에너지 검사를 연결했다. 관측 누락·오래됨·미래 시각·고정된 fault와 에너지 부족은 새 배차를 막는다. 기존 적재 임무와 이동/예약/정지 조건은 유지하며, 주차 위치를 물리 충전 접촉으로 간주하지 않는다.

신선한 관측 시각은 사건에 기록하되 판단의 의미가 바뀔 때만 로그를 추가한다. 에너지·ROS 연결30개와 입력/SDK 입구/기존 임무32개 검사가 통과했다. 변경 모듈과 검사 파일의 타입 오류는0건이며, 기존 run_mission의 복잡도 진단1건은 남아 있다. 실제 SDK/ROS 충전 접촉·전력 전달과 새 배차의 GPU 실행은 미검증이다.

## FOT-029 — 로컬 SDK4.5의 USD API 차이와 종료 코드0 오인

기존 PC의 Isaac Sim4.5에서 NovaCarter 참조 로딩 뒤 `Usd.Stage.GetCompositionErrors` 호출이 AttributeError를 냈다. 실제 바인딩을 조회한 결과 Stage에는 해당 메서드가 없고 `Pcp.PrimIndex.localErrors`가 존재했다. 합성 prim과 instance proxy의 prim index에서 오류를 수집하도록 바꾸었으며, 없는 API를 빈 목록으로 처리하지 않는다. 원래 실패 source는 전체 SHA를 대조해 보존했다.

동시에 SDK의 fast shutdown이 실패를 기록한 스크립트에서도 프로세스 종료 코드0을 반환하는 현상을 확인했다. host 실행기는 결과의 passed 상태·run ID·source SHA와 실제 프로세스 종료·잔여 PID를 함께 확인한다. 새 실행 전 source/config/launcher bytes를 `.inputs`에 고정하고, timeout 정리는 소유한 cmd와 자식 PID만 대상으로 한다.

수정 후 `asset-admission-002`에서 실제 구성 검사·180개 물리 표본·3초의 진행·연속 정지·640×480 RGB와 host closed/PID없음을 확인했다. 이 시험은 바퀴 명령이 없는1대 준비 시험이며 Nav2·적재·충전·4대 운송을 증명하지 않는다. [요약](evidence/local45-asset-002.json)에 원천 SHA와 백업 위치를 보존했다.

## FOT-030 — Windows SDK에 동봉된 ROS 미들웨어 DLL 경로 누락

`ros-transport-001`은 bridge 시작 중 `rmw_fastrtps_cpp.dll`의 LoadLibrary126과 후속 rclpy import 실패를 기록했다. 해당 DLL과 rclpy는 기존 설치에 존재했다. 설치된 bridge의 안내와 대조해 실행 프로세스의 PATH 앞에 SDK의 `exts/isaacsim.ros2.bridge/humble/lib`를 추가한 뒤 bridge를 활성화했다. 전역 환경·방화벽 변경과 추가 설치는 하지 않았다.

새 run ID의 `ros-transport-002`에서 native4.5/Humble 발행 메시지19건 중3건을 기존 WSL ROS2 Jazzy가 수신했다. 동일 run ID·증가하는 순번·원본 JSON payload의 완전 일치와 양쪽 종료를 확인했다. domain143의 run별 String 시험 토픽만 사용했다. 로봇 명령·센서 시계·QoS 전체·Nav2 운영 성공으로 확대하지 않는다. [요약](evidence/local45-ros-002.json)과 실패001을 함께 D/E에 보존했다.

## FOT-031 — 로컬 RTX LaserScan 초기·불완전 버퍼 거절

`rtx-scan-001`의 다중 빔 예제와 `002/003`의 설치된 평면 RPLIDAR_S2E에서 ROS 시계는 WSL에 전달됐지만, 정적8×8m 방의 벽 거리 검사에 필요한95% 유효 반환을 충족한 스캔을 확보하지 못했다. `002`의 첫 스캔은3,200칸 중1,252개가 유효했고, `003`의 첫 스캔은timestamp0·전체 -1이었다. 첫 프레임 원본만으로 이후 모든 스캔의 실패 원인을 단정하지 않는다.

수신 검사는 -1을 무효 칸으로 보존하면서 유효 비율·기하 거리 오차·증가하는 stamp·sim clock 근접성을 함께 검사한다. 첫 부분 스캔을 성공 자료로 쓰거나 합성 거리로 빈 칸을 채우지 않는다. 세 attempt는 failed로 보존했으며 source/config/helper/receiver snapshot과 실제 host 종료·잔여 PID 확인을 결속했다. 다음 진단은 전체 회전 이후의 최선·마지막 버퍼와 render/scan 주기 정합을 확인한다. 이 경계가 통과하기 전에는 로컬4대 Nav2의 센서 연결을 완료로 표시하지 않는다. 후속004에서 최선·마지막 native 버퍼는2,851/3,200칸 유효였고 무효 칸은 단일 칸으로 분산됐다.006은 별도 구성요소 기준으로720개0.5도 방위 sector의 실제 반환·최대 gap0.225도·벽 거리p95오차 약2.5~2.7cm와 수신 시계50ms 이내를 확인했다. raw95% fill은 false로 유지하며 약10.9% dropout의 원인은 미확정이다. 보간이나 원본 대체는 하지 않았다. [연결·품질 범위](evidence/local45-scan-006.json)를 분리해 보존했다.

005는 RTX 초기화가 약66.5초에 이르러 먼저 시작한35초 수신 창이 끝났고0 scan/0 clock으로 실패했다.006은 World.reset 뒤 sensor-stream-ready와 receiver-armed를 같은 run ID로 확인하고 측정 창을 시작했다. 초기화 대기120초·arm30초·측정35초와 host180초를 각각 제한했다. 수정 후3개 실제 scan과4개 clock 수신, native·WSL 종료 및 잔여 프로세스 없음을 확인했다.

## FOT-032 — 로컬 종료 확인 범위와 수신 ACK 결속

한정 검수에서 이전 host의 종료 탐색은 launcher와 run ID가 일치하는 한 단계 자식만 포함하므로 전체 후손 트리의 강제 종료를 보장하지 않는다는 지적을 받았다. 해당 실행 기록의 `closure_scope`와 `full_descendant_closure_verified=false`, `timeout_full_tree_cleanup_supported=false`에 범위를 명시했다. 정상 종료 관측을 임의의 손자 프로세스까지 추적한 증명으로 확대하지 않는다.

후속 실행기는 Windows Job Object에 대기 중인 worker를 먼저 배정한 뒤 SDK 실행 gate를 연다. breakaway를 허용하지 않고 kill-on-close를 설정하며, 종료·시간 초과 뒤 같은 Job의 active PID 목록이 비었는지 확인한다. 실제 Windows의 정상 종료·부모가 종료한 손자 프로세스·시간 초과3개 검사를 통과했다. `contact-003`은 실제 SDK 접촉 시험과 전체 Job 종료를 통과했고, SDK 종료 뒤 남아 있던17개 보조 프로세스도 같은 소유 Job에서 정리했다. 이전 기록의 종료 범위는 소급해서 바꾸지 않는다.

`contact-002`는 새 Job의 종료와 물리 시험은 통과했지만 기존 접촉 probe에 프로젝트 의존성의 빈 집합 필드가 없어 host의 source 집합 검사가 실패했다. producer에 명시적인 빈 `project_source_sha256`를 추가해003에서 대조했다. 숨긴 worker의 표준 출력이 사라지는 현상은 자식에 stdout/stderr 핸들을 명시해 수정했고,003의 SDK 로그를 보존했다.

한정 검수 WG026-001은 원본 source/config/dependency와 job spec가 snapshot 뒤 실행 전에 바뀌어도 SDK 호출 자체는 차단되지 않는 경로를 지적했다. 부모가 spec bytes를 한 번 읽어 SHA를 child에 전달하고, child는 gate가 열린 뒤 실제 입력의 SHA를 검사해 통과한 경우에만 SDK를 호출하도록 보정했다. source/config/dependency 및 spec 변조에서 SDK 호출이 없음을 확인한2개 회귀 검사와 기존 Windows3개 검사를 통과했다.004의 실제 SDK 실행에서 같은 run/spec/input preflight와 Job 종료도 확인했다. 검수자는 launch-time 경계에서 해결된 것으로 재검수했다. 실행 중 파일시스템 불변이나 권한 있는 악의적 변조까지 보장하지 않으므로 native 실행 중 담당 소스의 동결 조건은 유지한다.

또한 ACK의 passed 필드만으로 성공을 받지 않도록 최소3개 scan·같은 frame·증가하는 양의 stamp·방위 coverage·clock 오차와 config/receiver source SHA를 대조한다. receiver source도 host 입력에 고정하고 producer/host의 전체 프로젝트 source 키 집합과 값이 일치해야 한다. 빈 메시지·틀린 source/config·누락된 clock ACK를 거절하는 경계 검사를 통과했다. 최신007에서 엄격한 ACK/source 집합 검사와 실제3개 scan·4개 clock·정상 종료를 확인했다. 이전 원본의 판정은 바꾸지 않는다.

## FOT-033 — Kit 종료 후 IExt 객체와 Workspace 캐시 오인

Kit 관제 시험001은 선택·명령 처리·촬영 뒤 종료 검사에서 `_client` AttributeError가 발생했다. 설치 SDK는 `on_shutdown()` 뒤 IExt 객체의 `__dict__`를 비우므로 종료된 인스턴스의 검사 메서드를 다시 부를 수 없다.002는 manager와 활성 module 인스턴스 해제를 확인했지만 `Workspace.get_window()`의 visible 값이 true여서 실패했다.

003은 종료 전 실제 소유 ui.Window 참조와 숨김 callback을 확보하고 확장을 해제한 뒤 검사했다. manager 비활성·module 인스턴스 없음·소유 창의 visible=false·hide callback을 확인했으며, 종료 후 PNG에서도 패널이 사라졌다. 같은 시점의 Workspace handle만 true로 남았다. 제품을 강제로 숨기는 검사를 추가하지 않았으며, 캐시 값은 진단 필드로 보존한다.001/002의 실패 판정도 유지한다.

클라이언트의 Nullable 경계를 명시한 뒤004에서도 선택·세 명령·같은 원장 반영·창 종료를 재확인했다.004는 새로운 Job/prelaunch 검사와 전체 후손 종료까지 확인했으며 최신 촬영·소스 명세는 `docs/evidence/kit-console-004/`에 둔다.

## FOT-034 — SDK4.5 Python3.10의 운영 enum import 실패

로컬 SDK에서 fleet transport 모듈을 import할 때 Python3.11에 추가된 `enum.StrEnum`이 없어 실패했다. Python3.11 이상은 표준 StrEnum을 유지하고,3.10에서는 문자열 값·출력·포맷·JSON·동등성에 필요한 str/Enum 호환 클래스를 사용한다. 실제 SDK3.10과 CPU3.12에서 전체 운영 enum의 직렬화·값 복원 검사를 통과했고, SDK3.10의 fleet worker import도 성공했다. 이 결과는 SDK6.2 전용 센서·물리 API의 호환성을 증명하지 않는다.

## FOT-035 — Canvas 대체 문구를 렌더 실패로 오인

기존 Chrome 관제 탭은 도구의 focus-emulation 요청이 timeout되어 조작하지 않았다. 새 작업 탭은 API 상태·4대 목록·원장·충전 정보를 수신했다. 접근성 트리와 Canvas 자식 DOM에 있는 WebGL fallback 문구를 실제 렌더 실패로 오인했으나, 해당 문구는 Canvas 내부의 대체 콘텐츠였다. 화면 촬영에서 공장3D와 이동 중 차량·선택 상세가 정상 표시되는 것을 확인했다. 따라서 DOM의 fallback 텍스트 존재만으로 WebGL 실패를 판정하지 않는다. console 오류 목록도 비어 있었다.

브라우저 내부 `chrome://gpu/` 진단 페이지는 허용 프로토콜 정책이 거절했으며 우회하지 않았다. 제품의 HTTP 화면을 촬영하는 더 작은 확인으로 표시 여부를 판별했다. 사용자 Chrome 재시작이나 전역 그래픽 설정 변경은 하지 않았다. 기존 탭의 도구 연결 timeout과 새 탭의 실제 렌더 성공은 서로 다른 관측으로 보존한다.

## FOT-036 — World 초기화 전에 만든 RTX 센서 자원 미동작

4.5 facade의 native001~003은 바퀴 이동·정지와420스텝의 실제 시계 진행을 통과했으나 차량 앞뒤 LiDAR의 ROS 콜백이 없었다.003에서 실효 domain146·BEST_EFFORT 구독은 확인했지만 발행자는0이고 native flat-scan 버퍼도 비어 있었다. 렌더 갱신만 늘려도 해결되지 않았다.

004는 같은 실행에서 World 초기화 뒤 만든 정적 기준 LiDAR가3,200칸 native 버퍼와74개 ROS 메시지를 내는 반면, 초기화 전에 만든 차량 센서는 계속 비어 있음을 확인했다. facade에서 World 생성·reset을 센서·RenderProduct·writer 생성 앞으로 옮기고 초기화 순서의 CPU 회귀 검사를 추가했다. domain·QoS나 주행 로직은 바꾸지 않았다. 정확히 어느 SDK 내부 자원 초기화가 기존 자원을 무효화했는지는 확정하지 않는다.

수정 뒤005의 단일 차량 앞뒤 센서와006의4대·8개 차량 센서가 실제 발행했다.006은 정적 기준 센서를 포함하지 않으며 각 차량 약0.585m 이동·실측 정지, 센서별53~54개 스캔과3,200칸 native 버퍼, source/config/prelaunch/전체 Job 종료를 확인했다. 이 시험은 센서 ID·발행 진행과 짧은 물리 주행만 판정하며 거리·방위·ROS clock 정합·Nav2·자재 운송·충전 qualification은 미완료로 유지한다. 원래001~004 실패와 source snapshot을 보존한다.

## FOT-037 — 4대 배치의 위치 표시 개수와 고정 장면 검사 불일치

로컬 통합001은 `open_factory`에서 위치 표시 기대8개·실제10개로 실패했다. 4대 개발 배치의 추가 주차 지점은 입력에 선언돼 있지만 공통 장면 검사에는 기존 개수8이 고정돼 있었다. 공통 함수의 기존 기본값은 유지하고, 운송 진입점이 배치의 stations·pockets 개수 합을 넘기도록 수정했다. 추가 표시가 있는 배치는 기본8 검사에서 거절되고 선언10 검사에서 통과하는 회귀 검사를 포함해21개 검사와 변경2개 파일 타입 검사가 통과했다. 실제 통합 재실행 결과는 후속 기록으로 판정한다.

001의 소스·설정·로그는 새 실행으로 덮지 않는다. 약41초 뒤 Windows Job과 WSL 소유 cgroup의 잔여 프로세스가 없음을 확인했다. SDK 실행기 종료 코드0만으로 성공을 판단할 수 없다. 이 실행의 Job 검사는 통과했어도 장면 JSON과 ROS 임무가 실패했으므로 쌍 전체는 실패로 보존했다. [축약 근거](evidence/local45-transport-001.json)를 참고한다.

## FOT-038 — SDK4.5 ROS 배포본의 tf2_ros Python 도우미 누락

로컬 통합002는 장면 검사·4대 로딩·World 초기화와 초기 RGB2장 촬영 뒤 `ros_and_lidars`에서 `No module named tf2_ros`로 실패했다. 기존 SDK 배포본에는 rclpy와 tf2_msgs 메시지는 있지만 tf2_ros Python 도우미가 없었다. WSL에 설치된 공식 tf2_ros의 동적 depth100·정적 TRANSIENT_LOCAL/depth1 계약을 확인했고, 4.5 facade에서 포함된 TFMessage로 발행하도록 연결했다. 동적 odom→base_link와 정적 base_link→앞뒤 센서 관계는 namespace 안에 유지한다. 기본6.2 경로의 tf2_ros 사용은 유지한다.

정적 메시지 누적·동적 한 건 전달·상대 토픽·내구성 검사를 포함한15개 facade 검사와 변경3개 파일 타입 검사가 통과했다. native TF 수신·AMCL localization의 성공은 후속 실행으로 확인한다.002도 Windows Job과 WSL cgroup의 잔여 프로세스가 없음을 확인했고, 이전001의 marker 수정이 실제 장면 검사에서 통과한 근거로 보존했다. [축약 근거](evidence/local45-transport-002.json)를 참고한다.

## FOT-039 — Windows 읽기와 WSL 원자적 JSON 교체 충돌

통합003은4대·8개 센서 준비와4개 Nav2 스택 기동, 첫 ComputePath/FollowPath 수락·실제 가속·라이브 관제 발행까지 진행했다. 이후 carter2의 authorization.json을 WSL이 교체할 때 PermissionError로 임무가 실패했다. 별도 무해한 시험에서 Windows가 대상을 읽기 위해 열면 WSL os.replace가 같은 오류를 내는 것을 재현했다. 공유 충돌에만 최대0.2초 재시도하고, 원본·임시 파일을 유지하다 원자적으로 교체하도록 수정했다. 실제 재현 시험은 열린 읽기 핸들을 약50ms 뒤 닫자14회 시도 뒤 완성된 새 JSON을 게시했고, 고정 권한 오류·다른 I/O 오류는 계속 실패하도록 검사했다. ROS 승인·상태와 시각화의 교체 경로에 적용한다.

003의 최초 실측은 carter1 약3.306m 이동·최대 적용0.4m/s이며, 다른3대는 명령0으로 대기했다. 종료된 bag에서8개 raw scan 토픽과4개 정적 TF,671개 clock 메시지를 확인했다. 주문 완주·4대 혼합 운송 성공은 아니다. 데이터 대부분은 raw/filtered/localization 스캔 사본이므로 다음 local45 개발 bag은 원본8개 센서와 제어·TF·이벤트를 남기고 전처리·합류 사본을 제외한다. 실제 필터 발행·Nav2 소비는 유지하고6.2 평가 기록 정책은 바꾸지 않는다.

ROS가 실패 종료한 직후 외부 실행기가 아직 정리 중인 Windows Job runner를 종료해 최종 Job receipt가 남지 않았다. Linux cgroup 종료는 확인됐고 이후 SDK 프로세스가 없는 것을 관측했지만,003의 전체 Job 종료 증명은 미완료로 보존한다. 실행기에 ROS 종료 뒤 최대30초의 SDK 정리 유예를 추가했다. 별도 검수 PAIR031-001의 한쪽 정리 예외가 다른쪽 정리를 건너뛰는 문제도 수정했고,2개 오류 주입 검사와 control 재검수로 해당 경계를 확인했다. 새 native 실행으로 최종 종료 기록을 다시 확인해야 한다. [통합003 근거](evidence/local45-transport-003.json)와 [파일 충돌 재현](evidence/crossos-atomic-001.json)을 참고한다.

## FOT-040 — RGB 갱신 중단 뒤 첫 촬영 버퍼가 비어 있음

통합004에서3개 RGB 제품을 촬영 사이에 중단하는4.5 개발 최적화를 시험했다. 첫 촬영이1차원 빈 배열을 반환해 RGB 저장에서 실패했고, 유효한 운송 촬영으로 등록하지 않았다. 당시2.05초·물리123프레임과4대 pose는 촬영 전후에 유지됐으며8개 native 스캔 버퍼도 존재했다. RGB 재활성화 후 Kit 적용·annotator 준비 중 어느 세부 단계가 부족했는지는 확정하지 않는다. zero-delta 단계가 물리를 멈춘 뒤 Kit update5회와 두 번째 zero-delta 갱신을 추가해 다음 실행에서 확인한다. 빈 버퍼·잘못된 채널은 파일을 만들기 전에 카메라 파일명·shape를 포함한 오류로 거절한다.4.5의 RGB 중단 최적화 성공이나 속도 향상은 미완료다.

이번004는 실패 뒤에도 새로운 실행기가 Windows Job과 WSL cgroup의 완전 종료를 확인하고 최종 기록을 남겼다.003에서 누락된 종료 증거를004로 소급 채우지는 않는다. 이후 ROS context 오류는 SDK가 먼저 촬영 실패로 종료한 다음의 연쇄 오류로 구분한다. [004 근거](evidence/local45-transport-004.json)를 참고한다.

통합005는 paused Kit update5회와 추가 zero-delta 갱신 뒤에도 첫 overview 버퍼가 shape(0,)으로 남았다.004·005 모두 유효 이미지 검사를 거절했고 양쪽 프로세스 종료는 확인했다. RGB 중단 방식을 더 반복하지 않고 개발 lifecycle의 중단 override를 제거해003의 연속 RGB 갱신으로 복구했다. 빈 버퍼 검사·원자적 파일 교체 재시도·축소된 개발 기록 정책·실행기 종료 보완은 유지한다. [005 근거](evidence/local45-transport-005.json)에 실패와 제외 결정을 남겼다. 물리60Hz/렌더10Hz 후보는 별도 짧은 시계·센서 시험을 통과한 뒤에만 운송 루프에 적용한다.

## FOT-041 — SDK 승인 JSON 읽기 공유 충돌과 후속 위치 관측 만료

통합006의 SDK stdout에서 carter2 authorization.json 읽기의 PermissionError가 최초 오류로 확인됐다. ROS 임무의 fresh map pose unavailable은 뒤이어 발생했다. 종료된 bag의 시간 대조에서는8개 센서의 원본 시각이 약10Hz로 증가하고 clock 역행은 없었지만, SDK odometry/TF 발행이13.083초에서 끝난 뒤 clock이13.6초까지 진행했다. pose TTL0.5초를 늘려 숨기지 않는다. 공유 교체 중 파일 열기 경합에 짧은 재시도를 추가하고 SDK 결과 JSON 교체도 같은 원자적 재시도를 사용한다. 영구 권한 오류나 잘못된 JSON을 빈 값·캐시로 바꾸지 않는다.

006은3개 카메라의 유효 촬영3시점을 남겨 연속 RGB 복구를 확인했다. raw 스캔별 약6개 메시지가 같은 원본 시각을 반복하는 것도 확인했고, 물리 시계와 렌더 주기 분리 시험의 별도 확인 항목으로 유지한다.006에서도 외부 Job runner 종료 때문에 최종 Windows Job 증명이 빠졌다. 이제 실행기는 run ID가 일치하는 취소 파일을 받아 소유 Job 전체를 정리하고 최종 기록을 저장한 뒤 종료하도록 보완했다. 실제 자식·손자 프로세스를 포함한 취소 검사와 관련8개 검사가 통과했다. [006 근거](evidence/local45-transport-006.json)에 최초 오류와 후속 오류를 분리했다.

## FOT-042 — 수동 PhysX cadence 시험의 첫 native 호출 충돌

분리 시험 cadence001은 첫 wheel action,002는 첫 수동 PhysX simulate에서 native access violation로 종료됐다. 두 실행은 result.json이 없어 거절했고, vendor launcher 종료 코드0을 성공 근거로 사용하지 않았다. Job의 잔여 프로세스가 없는 종료는 확인했다. 물리 장면이 play 상태로 준비되기 전에 render-only 호출로 첫 Kit update를 억제한 순서가 관련됐는지 다음003에서 확인한다.003은 측정 구간 밖에서 정상 Kit update1회를 거친 뒤 수동 스텝을 시작하고 실제 시작 frame/time을 별도로 남긴다. 첫 두 실패에서 시계 정합·성능 향상을 주장하지 않는다. [실패 원본 명세](evidence/local45-cadence-failures-001-002.json)를 보존한다.

분리 시험003도 native 충돌로 종료됐다.004에서는 렌더 간격을 World 생성 때 설정하여 native 충돌 없이 실제 물리120프레임·2.000000104초, 렌더20회와 유효 RGB3개를 얻었다. 다만 두 rear 센서 시각이 World보다 약27ms 앞서 기존 한 물리 tick 허용치를 벗어났다. 원본 시각을 이동하거나 허용치를 늘리지 않았고 전체 시험은 실패로 유지한다. 이 후보는 운송 루프에 적용하지 않으며 후속 최적화 실험을 보류하고 기존 연속 렌더의 운송 구현을 우선한다.

## FOT-043 — 짧은 개발 실행 한도에 도달한 부분 운송

통합007은 공유 파일 읽기 재시도와 협력 종료를 적용한 소스1c5bd81로 실행했다. PermissionError는 관측하지 않았고, AMR2대가 각각8.45m·5.93m를 주행하며3개 구간을 마쳤다. 작성된450초 개발 한도에 도달해 Job이 종료됐고, 후속 ROS ExternalShutdownException은 종료 과정의 영향으로 구분한다. Windows Job·WSL cgroup 모두 종료를 확인했으며 정리 오류는 없었다. 주문4건이 투입됐지만 완료는0건이다. 시간 제한에 의한 부분 결과를 경로 실패나 운송 완주로 바꾸지 않는다. 물리 시간은40.033초까지 진행했고, 전체 임무를 이 짧은 입장 시험으로 완료할 수 있다고 가정하지 않는다. 반복 검수·별도 렌더 실험을 추가하지 않고 구현을 우선하며, 후속 실행은 실제 주행 진행량과 필요한 임무 시간을 고려해 범위를 정한다. [부분 실행 근거](evidence/local45-transport-007.json)를 보존한다.

007 이후 개발 운송은640×360 RGB를 선택할 수 있게 하고 최종1280×720 설정을 보존했다. 물리·센서 시각을 바꾸지 않는 설치 ROS helper의 PostProcessDispatchIsaacSimulationGate를 각 LiDAR 제품에 적용해10Hz 스캔의 중복 발행을 줄인다. 실제 gate 설정 읽기가 요청과 다르면 실행을 실패로 처리한다. 실행기와 Job은 명시된 상한을 최대7200초까지 허용하며 짧은 기본450초와 구분한다. 관련 runtime·Job 검사24개 및 촬영 해상도 검사3개가 통과했다. 성능 향상과 장시간 운송 완주는 다음 실제 실행으로 판정한다.

## FOT-044 — 촬영 뒤 일부 ROS 센서 발행 중단과 위치 추정 만료

008은8개 SimulationGate 읽기6과640×360 RGB를 확인했다. 종료된 bag에서는 활성 구간의8개 원본 토픽이 약10Hz 고유 시각으로 발행됐고 clock 역행은 없었다. 마지막 촬영은42.033초였고4개 토픽은 그 직전41.817~41.833초 이후의 발행이 없었다. 다른4개 토픽·clock·odom은 이후에도 진행했다. carter3 map→odom 마지막 시각은42.233초이며 마지막 정상 위치 검사42.700초에서는 나이0.467초였다. 이후 기존0.5초 위치 기준을 넘겨 ROS 임무가 실패했다. 발행 간격 부족으로 단정하지 않으며, 촬영 후 Replicator 연속 실행 복구를 보완하는 다음 실행에서 인과를 확인한다. SDK4.5의 문서화된 비차단 Start(start_timeline=False)를 촬영 뒤 제출하고 외부 PLAY와 함께 사용한다. 센서·ROS 시각을 보정하거나 TTL을 늘리지 않는다. SDK 종료의 기존30초 유예를 넘긴 후 host 취소로 양쪽 소유 트리 종료를 확인했다. 후속 유예는90초로 바꾸어 실제 실패 기록과 SDK 정리를 기다린다. [008 근거](evidence/local45-transport-008.json)를 보존한다.

## FOT-045 — 실행 중 WSL 재부팅과 수명 유지 연결 누락

009는 WSL user manager 조회 실패로 host가 중단했다. Windows Job은 종료됐지만 최초 Linux 종료 확인도 실패했다. 이후 Linux 부팅 시각이 해당 실행 시작보다 늦음을 확인했고, 원 unit은 not-found/inactive, cgroup과 해당 실행 경로를 사용하는 프로세스는 없었다. 별도 복구 기록으로 Linux 종료를 확인했으며 최초 host 기록을 성공으로 고치지 않았다. 재부팅을 촉발한 원인은 확정하지 않는다. Microsoft 문서는 systemd 서비스가 WSL 인스턴스 수명을 유지하지 않는다고 명시한다. 기존 detached systemd-run은 명령 제출 뒤 반환해 지속적인 WSL 클라이언트가 없었다.037에서 systemd-run --wait의 foreground 클라이언트를 소유해 유지하고, 실제 unit/cgroup과 클라이언트를 함께 종료하는 경로를 보완한다. 전역 설정이나 배포판 재시작은 이 복구에서 수행하지 않았다. [009 근거](evidence/local45-transport-009.json), [Microsoft 문서](https://learn.microsoft.com/en-us/windows/wsl/systemd), [systemd255 문서](https://www.freedesktop.org/software/systemd/man/255/systemd-run.html)를 참고한다.

037은foreground WSL 클라이언트를 유지하는 방식으로 보완했고12개 관련 CPU 검사를1회 통과했다. 실제 WSL 단위1회에서는70.016초 동안 추가 WSL 조회 없이 같은 boot ID와 살아 있는 클라이언트를 유지했다. 이후 부모·setsid 자식·unit/cgroup·Linux waiter·Windows 클라이언트가 모두 종료됐고 강제 클라이언트 종료는 없었다. 전체 fixture는71.875초였다. [수명·종료 근거](evidence/wsl-lifetime-037.json)에 소스와 원본 해시를 남겼다. 이 결과를009 재부팅 원인이나4대 운송 성공으로 확대하지 않는다.

## FOT-046 — 촬영 뒤 Replicator Start가 동결된 물리를 전진시킴

010은 촬영 뒤 `orchestrator.run(start_timeline=False)`를 호출한 첫 시점에 물리 프레임 불변 검사를 통과하지 못했다. SDK 오류는 `industrial capture hooks advanced the frozen simulation state`였다. Python mock 검사의 비차단 호출 확인은 native 물리 불변의 근거가 아니므로 해당 후보와 mock 검사를 제거한다. SDK 정리가 끝나지 않아 같은 run ID의 취소 파일로 소유 Job을 종료했고 Windows·WSL 종료와 정리 오류 없음이 확인됐다. 정리 정체의 native 내부 원인은 확정하지 않는다. [010 근거](evidence/local45-transport-010.json)를 보존한다.

후속 구현은 설치 SDK의 SimulationTime FlatScan 원본 버퍼를 읽고 고유 원본 시각마다 rclpy LaserScan을 한 번 발행한다. RGB 촬영은 기존 pause/resume 정책을 사용하며 별도 Replicator Start/Resume을 제출하지 않는다. 센서 시각 이동·TTL 확대·주행 위치 대입은 하지 않는다. 변환기는 도·m·Hz·첫 빔 시각이라는 설치 OGN 메타데이터를 사용하고 원본 거리 순서와 유효하지 않은 거리 값을 보존한다. warmup 빈 버퍼는 미발행, 역행 시각과 잘못된 populated 메타데이터는 오류로 처리한다. 촬영 이후8개 ROS 스캔의 연속성·AMCL·실제 운송은 새 실행에서 확인할 항목이다.

## FOT-047 — 센서 준비 구간 첫 native 버퍼의 ROS 시간 표현 오류

011은 `ros_and_lidars`에서 원본 시각이 유효한 nonzero ROS Time으로 변환되지 않아 실패했다. 처음부터 warmup 매 프레임의 버퍼를 발행 대상으로 삼은 경로였다. 해당 오류에는 원본 숫자가 포함되지 않아 값의 크기·초기화 내부 원인을 확정할 수 없다. 008의 준비 이후 native 관측에는 정상 초 단위 시각이 기록돼 있다. 후속 변경은 기존120스텝 준비 구간을 마친 뒤 readiness 루프에서 발행을 시작하며, 이후 중복·역행·범위 검사는 유지한다. 오류에는 센서 정체성과 원본 시각 값을 포함한다. 원본 시각을 고치거나 표현 오류를 성공으로 처리하지 않는다.

011은 SDK 오류 확인 뒤 해당 run ID의 취소 요청으로 정리했고 Windows·WSL 종료와 정리 오류 없음이 확인됐다. 준비 완료 후의8개 ROS 스캔·촬영·운송은 다음012의 확인 항목이다. [011 근거](evidence/local45-transport-011.json)를 참고한다.

## FOT-048 — 충전 접근 경유점이 작업장 목표 사전에 없음

012는 센서 준비 이후8개 native→ROS 스캔 수신·첫 촬영·Nav2 초기 위치 추정과 가상 충전대 예약까지 진행했다. 이후 자기 충전대에서 출발하는 carter1의 접근 경유점 M_LOW를 작업장 목표 사전에서 조회해 KeyError로 실패했다. 경로 그래프에는 M_LOW가 있지만 작업장·주차면의 최종 자세 사전에는 없는 것이 원인이다. 충전 예약이 있는 빈 차량의 경유 leg만 경로 마지막 edge 방향으로 최종 자세를 계산하고, 일반 업무 목표는 기존 작업장 자세를 사용하도록 수정한다. 이후에도 ComputePath/FollowPath와 정지·도착 판정을 거치며 이동을 생략하지 않는다.

012의 Windows·WSL 종료는 확인됐다. 초기 native header는8개 모두3200거리값, 약0.1초 scan_time이며 각도 범위와 원본 시각을 보존했다. 이전011의 준비 구간 변경이 초기 발행을 통과한 근거이며, 긴 촬영 연속성·운송 완주·충전 완료를 뜻하지 않는다. [012 근거](evidence/local45-transport-012.json)를 참고한다. ROS 조정기 CPU 검사는 Python3.12에서 실행하며, SDK4.5 Python3.10은 renderer adapter 검사 범위다.

## FOT-049 — 경유점을 지나도 이전 구간을 포함하는 예약 요청

013은 첫 공정 투입과 자재 소비·빈 토트 반납 작업 생성까지 진행했지만, 후속 차량들의 대기가 길었다. 기존 request 경로는 각 경유점의 정지 확인 후에도 최초 leg 전체의 타일을 다음 예약에 사용했다. 지나온 영역을 포함하는 보수적 예약은 소스에서 확인됐으며, 이 범위가 실제 처리량에 미치는 크기는 측정하지 않았다. 현재 실행은 소스 f06597c를 동결한 기준 실행으로 유지한다.

041은 기존 종료 성공·HOLD ACK·신선한 zero-odometry·도착 검사로 anchor가 갱신된 경계에서만 남은 경로를 계산한다. 이전 hold는 원자적 successor transfer가 승인될 때까지 유지하고, 현재 관측된 차량 점유가 포함되지 않거나 peer와 충돌하면 기존 manager가 거절한다. 승인된 기록에 실제 예약 타일과 남은 경로 출처를 남기며 원래 leg 식별자도 보존한다. 관련 CPU 검사4개가1회 통과했다. 실제 ROS 적용·대기 시간·처리량 개선은 다음 실행에서 확인할 항목이다. [041 결과](../coordination/collab/results/remaining-route-reservation-041.json)를 참고한다.

013은 기준 관측을 마친 뒤041 적용 실행으로 전환하기 위해 부분 종료했다. 종료 상태를 성공으로 바꾸지 않았다. 종료 후8개 raw 스캔은 각각2108~2441개의 고유 시각을 기록했고, 약274초까지 스캔과 clock의 시각 역행은 없었다. 원본198파일은 D·E에 보존해 해시를 대조했다. 전체 주문 완료·빈 토트 반납 완료와 센서 품질 합격은 아니다. [부분 기준 실행 근거](evidence/local45-transport-013.json)를 참고한다.

## FOT-050 — endpoint 발견과 실제 clock 수신의 차이

014는 예약 개선 함수에 진입하기 전 evidence_discovery 단계에서 실패했다. recorder는 약300초 동안 clock을 기다렸고 일부 lifecycle manager도 서비스를 기다렸다. 단일 읽기 진단 노드에서는 clock 발행자1개와 해당 서비스 endpoint가 보였지만 실제 표본 수신·서비스 응답을 증명하지는 않았다. Nav2 PID는 살아 있었고 WSL 가용 메모리는 약38GiB, swap 사용은0이었다. 뒤의 exit -9는 임무 실패 이후 정리 과정과 구분한다. 실제 발견·전송 실패의 세부 원인은 미확정이다.

다음 실행은 다른 조건을 유지하고 이 프로젝트의 ROS domain을72에서73으로 분리한다. 이것은 재사용된 통신 그래프 영향 여부를 가리는 한 변수 시험이며 원인 확정이나 해결 선언이 아니다. Windows와 WSL은 같은 입력 config를 사용하고 전역 환경·배포판·다른 작업 프로세스는 바꾸지 않는다. ROS domain은 통신 그룹을 나누지만 실제 통신 성공의 충분조건은 아니다. [공식 domain 설명](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html)을 참고한다.014의 Windows·WSL 종료와 정리 오류 없음은 확인했다. [014 근거](evidence/local45-transport-014.json)를 보존한다.

015의 domain73에서는 clock 수신·기록기 전체 구독·4개 Nav2 ACTIVE와 실제 주행이 시작됐다. 2번 차량은 정지 확인25.916668초 뒤25.933335초에 새 예약을 요청했고, P2에서 시작한60타일을 M_HIGH부터의48타일로 줄였다.12개의 지나온 타일을 제외한 실제 경계 관측이며 전체 처리량 개선은 아니다. [정지 후 갱신 근거](evidence/remaining-reservation-native-015.json)를 보존했다.014의 세부 발견 실패 원인은 여전히 미확정이다.

## FOT-051 — 호스트 상태 조회 timeout이 실행을 종료함

015의 최초 host 오류는 `systemctl --user show`를 실행하는 WSL 클라이언트가20초 안에 응답하지 않은 TimeoutExpired였다. 기존 감시 루프는2초마다 inspect를 호출했고, 이 읽기 조회 예외를 전체 실행 실패로 처리해 finally에서 Windows·WSL을 정리했다. 뒤의 ROS context-invalid 오류는 이 정리 과정과 구분한다. 종료 전 실제 시뮬레이션은 약40초까지 진행했고, 양쪽 소유 프로세스의 종료와 정리 오류 없음은 확인했다. 원본 상태를 성공으로 바꾸지 않았다. [015 근거](evidence/local45-transport-015.json)를 참고한다.

042는 이미 소유권 admission을 마친 foreground `systemd-run --wait` Popen 클라이언트를 관찰하도록 수정했다. 실행 중 반복 WSL 조회에 의존하지 않고, 시작·최종 종료의 unit/cgroup/PID 소유권 확인과 전체 시간 상한·SDK 종료 유예는 유지한다. Windows 클라이언트 종료를 Linux 완전 종료 증명으로 사용하지 않는다. 관련 CPU 검사9개를1회 실행해 통과했으며 총괄의 한정 diff 검수를 마쳤다. 실제 적용은 다음 실행에서 확인할 항목이다.

## FOT-052 — native FlatScan 입력의 한 물리 스텝 시각 역행

016은 약200.433초까지 진행해 자재 픽업2건·재고60→56과 가상 충전 복귀·예약 해제를 확인했다. 첫 CI 도착·정지 뒤 하역 대기 중 carter2/rear의 native timeStamp200.3333437821436이 마지막 ROS 발행200.350010449보다 약16.666667ms 이전이라 strict converter가 실패했다. 종료된 bag의 마지막24개 후방 스캔과 clock 표본만 읽어 이 경계를 확인했다. 거절된 native 거리 버퍼 전체는016에서 저장하지 않아 같은 버퍼의 재조회인지, 내부 계산·전달 순서 문제인지는 확정할 수 없다.

이후 fresh map pose unavailable과 SDK 종료 유예90초 초과는 최초 센서 오류 이후의 결과로 구분한다. Windows·Linux 소유 트리 종료와 정리 오류 없음은 확인했다.042의 반복 WSL 상태 조회 timeout은 재발하지 않았으며 최초·최종 소유권 확인도 통과했다. 픽업·충전의 부분 결과를 주문 완료·빈 토트 반납 완료나80% 완료로 표시하지 않는다. 원본171파일737,547,621바이트를 D·E 저장소에 보존해 해시를 대조했다.

044는 strict 변환기의 기본 실패를 유지하고, 로컬4.5에만 명시적 거절 어댑터를 연결한다. 한 스캔 기간 이내의 역순 입력을 발행하지 않고 원본 bytes·dtype·shape·시각·해시를 보존한다. 회복 한도는 스캔 기간을 물리 스텝으로 올림한 값이며 현 구성은6프레임이다. 새 시각이 돌아오지 않거나 큰 역행·World clock/frame 초기화·원본 저장 실패·센서별16건의 보존 한도 초과가 생기면 실패한다. 유효한 마지막 시각을 낮추거나 기존 관측 유효시간을 늘리지 않는다. 같은 시각의 반복 입력은 버퍼를 다시 읽지 않는 기존 처리 방식을 유지한다. 초기22개 검사와 해당 처리 보완 후 영향받은15개 검사를 각각1회 통과했으며, SDK Python3.10.15의 두 소스 구문도 확인했다. 실제 적용 검증은 다음017 실행에 남아 있다.

[SDK4.5 FlatScan 명세](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/py/source/extensions/isaacsim.sensors.rtx/docs/ogn/OgnIsaacComputeRTXLidarFlatScan.html)는 출력 시각을 첫 빔의 초 단위 시각으로 정의한다. SDK 내부의 역행 원인이나 반환 버퍼의 원자적 일관성은 이번 자료로 확정하지 않는다. [016 부분 실행](evidence/local45-transport-016.json)과 [044 결과](../coordination/collab/results/bounded-native-scan-admission-044.json)에 구현·검사·미확인을 구분했다.

## FOT-053 — clock 수신과 차량 데이터 기록·초기화의 불일치

017은 기록기의 구독 생성까지 진행했지만 planner/controller/TF/odometry/scan 기동 조건에서 실패했다. 종료된 bag에서 clock2,852개·4.667~51.950초·시각 역행0을 확인했고, 선택한8개 raw scan·odometry·dynamic TF는 기록0건이었다. static TF는 이 검사 대상이 아니다. Windows SDK의8개 센서 자체 구독 헤더는 존재한다. AMCL에도4.817/7.317/7.517초의 일부 스캔 수신·TF cache/queue 거절 로그가 있어, bag0건을 모든 WSL 구독자의 미수신으로 확대할 수 없다.

동결 소스는 SDK 환경을 SimulationApp 생성 전에 설정하고 있었다.017의 실제 Python context domain은 기록이 없어 domain0이라고 소급 판정하지 않는다. FastCDR ParticipantEntitiesInfo 경고는 기동에 성공한016에도 있어 단독 원인으로 보지 않는다. 초기 pose의 ROS 시각은016에서4.233초,017에서0초였고017의 map TF는 성립하지 않았다. 정확한 전달·기동 실패 원인은 미확정이다. G3 기동 TimeoutError가 최초이며 SDK90초 종료 유예 초과는 후속이다. 양쪽 소유 프로세스와 기록기 종료를 확인한 후 bag을 읽었고 정리 오류는 없었다.

046은 실행별 UDP participant 파일과 부모 환경을 고정하고 실제 SDK·WSL domain/RMW를 기록·대조한다. 로컬 기록기의 raw scan은 BEST_EFFORT, odometry·dynamic TF는 해당 발행자의 RELIABLE로 명시하며 기존 latched 주제와6.2 경로는 유지한다. 기동 동안 raw 수신과 전처리 후 계수·원래 시각·odometry·pose·마지막 lifecycle 상태를 별도 저널에 남긴다. 기존 기동 조건·관측 유효시간·시간 상한은 유지한다. [017 근거](evidence/local45-transport-017.json), [045 진단](../coordination/collab/results/nav2-startup-045.json), [UDP 설정 참고: NVIDIA5.1 설치 안내](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)를 함께 보존한다. 실제 효과는 다음018에서 확인한다.

018은 SDK 네 context와 WSL context에서 domain73·rmw_fastrtps_cpp를 관측했고, 네 차량의 planner/controller/bt_navigator ACTIVE·pose·odometry·전후방 스캔 기동 조건을 통과했다. 실제 운반·하역·공정 소비와 같은 실행의 웹 관측도 진행됐다. 이 결과는 새 구성의 기동 성공이며017 실패 원인의 소급 확정은 아니다. 이후의 센서 실패는 아래 FOT-054로 분리한다.

## FOT-054 — 반복된 native 역순 입력과 보존 한도 초과

018은 약235.5초에 carter1/front의 native 역순 입력 보존 한도16건을 초과해 실패했다.234.267~234.917초의 짧은 구간에8개 센서의 거절111건이 보존됐다. 스냅샷과 range 원본의 해시를 확인했으며 일부 거절 사이에도 더 새로운 발행은 이어졌다. 한 번의 회복 기한 초과가 최초 오류는 아니다. 뒤의 ROS pose 유효시간 오류와 SDK90초 종료 유예 초과는 후속으로 구분한다. 양쪽 소유 프로세스는 종료됐고 정리 오류는 없다.

종료된 bag의 각 raw scan과 clock 마지막128건을 한 번 확인했다. 선택 구간의 발행 시각 역행·동일 시각 중복은0이지만, 실패 직전 raw scan에는 약10마이크로초의 양수 시각 간격도 나타났다. 이는10Hz 완성 스캔의 발행 사건과 출력 폴링을 구분할 필요가 있는 관측이며, SDK 내부 원인이나 반환 버퍼의 원자적 일관성을 확정하지 않는다.23회 촬영의184개 센서별 전후 쌍에서는 같은 물리 프레임 내 native 시각 변화가 없었다. 촬영을 원인으로 단정하거나 시각 보정·관측 유효시간 확대·보존 한도 증가로 통과시키지 않는다. 다음 수정은 완성 스캔의 execution 출력과 읽기 경계를 확인하는 범위로 좁힌다.

실행 중 첫 하역·공정 소비2개와 빈 토트 회수 작업1건 생성, 가상 충전·예약 해제,4번 차량의 출하 작업 배정을 확인했다. 재고56+운반 중2+소비2=초기60개다. 완료 주문·빈 토트 반납은0건이고4번 차량의 실제 운반도 아직 확인되지 않았다.80% 완료로 표시하지 않는다. [018 근거](evidence/local45-transport-018.json)에 원본·시각·상태 범위를 남겼다.

후속의 작은 두 센서 시험에서는 Python Writer 캐시 방식이20초 촬영 후0.3초 신선도 조건에서 실패했고, execution 값의 단순 폴링도1.5초 지점에 한 센서의 새 출력을 놓쳐 실패했다. 두 방식은 적용 후보에서 제외했다. 두 시험의 프로세스 종료 성공과 센서 시험 실패를 구분하며,018의 원인 재현이나 수정 성공으로 표시하지 않는다. [후보 시험 근거](evidence/native-boundary-probes-049-050.json)를 보존하고 지원되는 사건 수신 API를 확인한다.

## FOT-055 — 완성 스캔 실행 수신과 운항 중 RGB 촬영 경계

051은 FlatScan의 `outputs:exec`를 같은 그래프의 SDK ScriptNode에 연결해 원본 시각·버퍼를 복사했다. 단순 execution 값의 폴링을 사용하지 않았고, 완료 사건 수신은 시작됐다. 그러나 기존 Replicator 수동 촬영 뒤20.217초에 새 완료 사건이0.3초 동안 오지 않아 실패했다. 오류 문자열의 writer-cache 표현은 기존 시험의 문구이며051에는 Writer를 붙이지 않았다. 프로세스 종료 결과와 이 실패를 구분한다.

052는 같은 완료 사건 수신기에서 `World.render()`4회로 물리 상태를 고정한 RGB를 읽었다. 작은 RGB 제품을 함께 사용한 두 센서 시험으로270초·촬영13회·시각 역행0, 센서마다 새 시각2,708회와 이전 복사본 보존을 확인했다. 촬영 전후 실제 World 시각·프레임은 모두 같았다. SDK 함수는 렌더 갱신 중 시뮬레이션 실행을 막고 다시 복원한다. 이 결과는 적용할 수신·촬영 경계의 격리 근거이며,018의 세부 내부 원인이나8센서·4대 운송을 입증하지 않는다.051과052에는 RGB 제품 구성 차이도 있어 단일 변수 원인 확정으로 표시하지 않는다.

로컬4.5 운송에만 같은 렌더 hook과 완료 사건의 소유 버퍼를 연결한다. 기존 변환·중복 처리·6프레임 회복·보존 한도·ROS 유효시간과6.2 경로는 유지한다. 초기 소스 확인에서 동적 속성 생성 함수의 bool 반환을 Attribute로 취급한 결함1건을 고쳤고 해당 행·해시만 다시 확인했다. 전체 검토나 문자열 검사는 반복하지 않았다. [052 근거](evidence/native-completion-052.json)와 [설치 API 근거](../coordination/collab/results/native-complete-boundary-048.json)에 범위와 미확인을 남겼다.

두 probe의 ScriptNode 제거 과정에는 `state:omni_initialized`를 찾지 못하는 SDK release 콜백 오류 로그가 있다. Python 정리 호출은 반환됐고 소유 Job의 남은 PID는0이었다. 로그 오류가 없었다고 표시하지 않으며, 운송 데이터 실패와 종료 콜백 로그를 분리해 후속 실행에서도 보존한다.

## FOT-056 — 완료 복사본의 반복 조회와 회복 기한 실패

019는 네 차량의 ROS 초기화를 통과했으나54.217초에 새 성공 스캔이6물리 프레임 안에 돌아오지 않아 실패했다. 보존 한도 초과가 최초 오류는 아니다. 뒤의 ROS pose 유효시간 오류와 SDK90초 정리 유예 초과를 후속으로 구분한다. 양쪽 소유 프로세스 종료와 정리 오류 없음을 확인한 뒤 원본499파일366,460,014바이트를 D·E에 보존하고 해시를 대조했다. [019 근거](evidence/local45-transport-019.json)에 실패 판정과 원본 범위를 남겼다.

완료 callback이 만든 같은 복사본을 물리 프레임마다 재조회하면서 동일 거절 버퍼도 반복 보존되고 있었다.051은 소유 callback ID를 붙여 처리한 사건의 재조회가 발행·획득 계수·보존을 늘리지 않도록 한다. 모든 호출의 실제 World 시각·프레임 검사와 최초6프레임 회복 기한은 유지하며, 서로 다른 거절 사건16개는 기존 한도를 적용한다. 발행·보존 실패는 완료 ID를 소비하지 않아 같은 사건의 재시도를 숨기지 않는다. 새 CPU 경계10개와 SDK3.10 구문 검사를 한 번 통과했다.

진단 관측은 발행과 같은 소유 완료 복사본을 빌려 읽도록 연결했다. 별도 annotator 연결·가변 출력 재조회는 native 발행 경로에서 제거하고, 기존 비native 경로는 유지했다. 관측 종료가 생산자를 닫지 않는 소유권·오류 전파 검사1개와 작은 소스 검토를 한 번 수행했다. 두 변경의 실제 효과와019 회복 실패 해결은 다음 실행에서 확인한다. 설치 앱의 asyncRendering 기본값은 이미false이며 새 플래그를 추가하지 않았다. SDK 내부 원인이나 시계 정합은 아직 확정하지 않는다.

## FOT-057 — 네 번째 차량의 TF 유효시간 실패와 센서 시각 비교

020은63.5초에 중단됐다. 최초 오류는 carter4의 map→base_link TF 나이0.533302초가 기존 허용0.5초를 넘은 것이다. SDK의 mission-failed와90초 정리 유예 초과는 후속이다. native 거절 사건은 소유 완료 ID별 한 번씩 보존됐고, 회복 기한이나16건 한도 초과가 최초 실패는 아니었다. 양쪽 프로세스가 종료됐으며 정리 오류는 없다.

종료된 bag의 carter4 raw scan·odometry·TF와 clock 끝부분을 한 번 읽고 파일 해시 불변을 확인했다. 마지막 odometry·odom→base_link는63.450초이고, 전후방 raw header는63.916701초였다. 마지막 map→odom은62.916701초였다. raw 수신 순서상 가장 가까운 선행 clock은63.450초로 약0.467초의 차이가 있다. 이는 같은 취득의 원자적 시계 대조나 SDK 내부 원인 확정은 아니다. 수신은 이어졌으므로 단순 센서 전체 중단으로 표시하지 않으며, TF 유효시간 확대나 원본 header 이동으로 통과시키지 않는다.

물리 충돌 raycast를 사용하는 설치 PhysX 센서를 별도 로컬 개발 후보로 준비했다. 거리 배열·각도·자기 차체 반환·타깃 이동 반응과 World 관측 시각을 작은 보정 장면에서 확인한 뒤 사용 여부를 결정한다. 원래 RTX/RPLIDAR/FlatScan 모델과 동등하다고 표시하지 않으며,6.2 경로와 과거 실패 자료는 유지한다. 아직 제품 경로에는 연결하지 않았다. [020 근거](evidence/local45-transport-020.json)에 최초·후속 실패를 남겼고, 원본264파일383,730,316바이트를 D·E에 보존해 해시를 대조했다.

## FOT-058 — PhysX 평면 후보의 각도 해석과 차체 가림

첫 후보054의 원래 판정은 실패다. 시험 코드는 수평 zenith를π/2로 가정했지만, 설치 API의 실제 원본은720×1 거리·intensity와720개 방위, 수평 값0을 반환했다. 종료된 원본의 별도 CPU 해석에서 앞 센서의 동쪽 표적 거리가3.700000→4.700000m로 한 실제 물리 스텝 뒤 바뀌는 것을 확인했다. 새 해석 파일을 원래 실패 결과와 구분해 보존했다.

앞 센서의 뒤쪽과 뒤 센서의 앞쪽은0.1m 반환이 있으며, 모든 센서가 모든 방향의 벽을 볼 것이라는 시험 기대도 맞지 않았다. semantics를 끈 구성의 prim path는 비어 있어 반환 물체의 prim identity는 미확인이다. 기존 ROS 소비 경로는 각 센서의 앞쪽 반평면만 사용하므로 새 backend에서도 원시 가림 값과 이 소비 정책을 유지한다. 가려진 방향에 뒤쪽 벽의 거리나 자유 공간을 만들어 넣지 않는다.

18개 물리 스텝과 정지 상태 렌더 전후 World 시각·프레임 불변, 소유 복사본 보존은 확인했다. 정적 보정을 위해 rigid body를 비활성화한 자산에서는 static body 사이 joint를 생성할 수 없다는 SDK 오류 로그가7건 있다. Python 정리 오류는 없고, SDK close 반환 기록은 없지만 호스트 Job의 잔여 PID0·종료를 확인했다. 호스트 성공을 센서 입장 조건 통과로 확대하지 않는다. 원본79파일2,337,745바이트를 D·E에 보존했다. [054 근거](evidence/physx-planar-admission-054.json)에 실제 확인과 미확인을 남겼다.

제품 후보는 별도 opt-in PhysX 순간 평면 모델이다. 실제 World 관측 시각으로 표기하고 native 절대 센서 시각·RTX/RPLIDAR 동등성은 주장하지 않는다. 실제 운항의 거리·TF·센서 소비와 자기 차체 반환은 새4대 통합 실행에서 확인해야 한다.

## FOT-059 — Python 임무 시계와 bag·플래너 시계의 진행 불일치

021의 PhysX 모드는 네 차량의 Nav2·AMCL·odometry·8개720방위 스캔 기동을 통과했고, carter1의 주행을 시작했다. 실제 prim 경로로 자기 차체 반환도 관측됐다.14.95초의 최초 오류는 carter2가 받은 경로의 유효시간 초과다. 요청·임무 측 관측 시계는14.933334초인데, 서버 수락 시각과 반환 경로 header는5.450000초여서 나이9.483334초가 기존 한도0.5초를 넘었다. SDK mission-failed와90초 정리 유예 초과는 후속이다.

종료된 bag을 한 번 확인했더니 `/clock`91건은5.450000초에서 끝났고, raw scan·odometry는14.85~14.9초까지 이어졌다. Python 임무 노드의 ROS 시계는14.933334초까지 진행했다. 이는 소비자별 시계 관측 불일치이며 모든 발행이 멈췄다는 증거는 아니다. SDK graph 수명·QoS·전달 경계의 세부 원인은 미확정이다. 이 bag의 수신 시각 정합은 검증되지 않았으므로 최종 운영 시간·정책 비교 근거로 사용하지 않는다.

오래된 경로를 재시도하는 후보057은 읽기 단계에서 보류됐고 제품·테스트 변경은 없다. 새 local45 PhysX 모드는 같은 실제 World 물리 시각을 지속되는 rclpy 발행자 한 개에서 제공한다. 발행 QoS를 명시하고, 실제 physics app.update 뒤에만 시계를 발행하며 정지 촬영에서는 새 시각을 만들지 않는다. 초기 endpoint 개수·QoS와 SDK 자체 수신을 기록하고,1시뮬레이션 초 간격의 현재 상태 파일과 종료 보고를 남긴다. 전체 WSL 소비자 전달은 이 자체 수신과 별개로 확인해야 한다. 기본 RTX·6.2 시계 경로와 기존 TTL·시각·실행 상한은 유지한다.

[021 근거](evidence/local45-transport-021.json)에 확인 범위와 미확인을 남겼다. 양쪽 소유 프로세스 종료를 확인한 뒤 원본130파일166,419,389바이트를 D·E에 보존하고 해시를 대조했다. 새 시계의 촬영 이후 전달·플래너 시각·실제 운송은 다음022에서 확인한다.

## FOT-060 — 초기 HOLD 응답과 수신 시계의 확인 경계

022는 네 차량의 Nav2·pose·odometry·8개 스캔 기동 조건을 통과했다. 초기 DDS 목록에서 World 시계를 발행하는 노드 한 개와 RELIABLE QoS를 확인했고, 종료된 bag의 마지막128개 clock은5.7500003초까지 이어졌다. 이전021의5.45초 정지 지점은 넘었으나, 주행 목표와 운항 중 촬영에 도달하지 않아 플래너와 촬영 이후의 전달은 미확인이다.

최초 오류는 초기 예약 HOLD 적용의 확인 기한 초과다. 네 차량 모두 seq20의 유효한 HOLD·0속도 명령을30물리 프레임씩 적용한 궤적이 있지만, 임무 노드의 확인 장벽은 통과하지 못했다. 첫 평가 시각은 허가 발행 시각보다1ns 미만 앞서며 기존 엄격한 비교에서는 거절 대상이다. 수신 노드가 각 응답 파일을 읽은 순간의 ROS 시계는 남아 있지 않다. 최신 파일을 반복 읽을 때 시계보다 한 프레임 앞선 응답을 계속 쫓을 가능성은 가설로 유지한다.

후속 수정은 계약에 맞는 최초 응답의 원본 복사본을 보관한 뒤 실제 수신 시계가 그 표본에 도달할 때 확인하는 범위다. 잘못된 scope·sequence·시각의 응답은 보관하지 않고, 기존 허가0.5초·시간 상한·추가 다음 시계 틱 장벽을 유지한다. 후보·확인·장벽 상태가 바뀔 때 작은 상태 파일을 남겨 다음 실패의 수신 시각을 확인한다. 이 수정의 실제 효과는 다음 실행에서 확인한다.

SDK mission-failed와90초 정리 유예 초과는 후속 오류다. 양쪽 소유 프로세스 종료와 정리 오류 없음을 확인한 뒤 원본101파일130,919,317바이트를 D·E에 보존해 해시를 대조했다. 운송 목표·완료 주문·빈 토트 반납은0건이며, 자재 상태 파일의 부재를 재고0으로 해석하지 않는다. [022 근거](evidence/local45-transport-022.json)에 원본과 미확인 범위를 남겼다.80% 판정은 보류한다.

023에서는 최초 적격 응답을 실제 수신 시계5.916667초에 보관하고, 네 차량 모두5.933334초에 확인한 뒤 추가 시계 틱5.950000초에서 초기 장벽을 통과했다. 운항 중 촬영 이후에도 플래너 시각이 현재 시계 안에서 확인됐다.022의 정확한 수신 순서는 소급 확정하지 않으며,023의 후속 실패는 아래 정차 유지 문제로 분리한다.

## FOT-061 — 0속도 명령 중 차량 밀림과 출발점 이탈

023은320.383초의 경로 확인에서 carter4의 관측 시작점이 작성된 P4 시작점의 허용0.15m를 벗어나 실패했다. 마지막 ROS 관측은(16.996617,20.146460)m로, P4(17,20.5)m에서0.353556m 떨어져 있다. 반환 경로의 시각 나이는 약0.033초로 기존0.5초 안이며, 이 실패를 시계 유효시간 오류로 분류하지 않는다.

차량의19,112개 물리 표본에서 두 바퀴 목표 속도는 모두0이었다. 그러나 SDK 평가 위치는2.067~320.583초 사이에0.354442m 이동했고, 바퀴 엔코더 적분도 약0.353m를 나타냈다. 표시나 odometry만의 차이가 아니라 물리 장면의 정차 유지 문제다. 정차 중 힘·접촉·drive 설정 중 정확한 발생 원인은 아직 확정하지 않았다. GT는 이 평가에만 사용하며 제어 입력으로 넣지 않는다.

설치 SDK는 바퀴 두 축에 한정한 position/velocity drive target과 유한 PD gain을 지원한다. 수정 후보는 0속도 구간의 최초 엔코더 위치를 보관해 유지하고, 이동 명령에서는 원래 velocity gain을 복원한다. 원래 drive effort·caster joint·가감속 프로파일·주행 허가·시각·시작점0.15m 기준을 유지한다. wheel wrapper가 다른 축에도0 target을 채우는 점과 mode 변경의 gain cache를 고려해 indexed action을 사용한다. 즉시 joint state 설정·base 고정·pose 보정·encoder 초기화는 사용하지 않는다. 전체 운송 재실행에 앞서 두 차량의 정차·재출발·회전 비교 한 번으로 후보를 확인한다.

이 실행에서 초기 정지 확인, 실제 주행과 연결된 자재 인수2건, 공정 소비2개, 빈 토트 회수 작업1건 생성, ROS 도착·정지 기반 가상 충전 예약 해제를 확인했다. 재고56+운반 중2+소비2=60개다. 완료 주문·빈 토트 반납은0이고,4번 차량의 업무 주행은 확인하지 못했다. 적재물은 운영 원장과 시각화 모델이며 접촉·인양·하중 물리 검증은 아니다.

양쪽 소유 프로세스 종료·정리 오류 없음을 확인하고 원본과 파생 자료233파일1,003,893,377바이트를 D·E에 보존해 해시를 대조했다. SDK 보고서는 원래 `running` 상태로 끝나 미완성이며, host 종료를 SDK 통과로 확대하지 않는다. SDK mission-failed와90초 정리 유예 초과는 후속 오류다. bag SQL은 열지 않았다. [023 근거](evidence/local45-transport-023.json)에 범위를 남겼다.

종료된 원본26회 촬영으로25.4초·1280×720·254프레임의 타임랩스를 만들고 전체 decode와 원본 해시를 확인했다. 화면에 실패 상태와 명목12배속·원본 정지 프레임 유지·보간 없음을 표시한다. 연속 실시간 주행 영상이나80% 완료 영상으로 승인하지 않는다.

비교 시험의 실행 전 소스 확인에서 NumPy float32 목표값이 JSON 직렬화에 남는 결함을 발견했다. 값을 Python float로 보존해 기록하고, 메타데이터 저장 실패에도 SDK 종료를 시도하도록 고쳤다. 이 결함은 SDK 시험을 시작하기 전에 수정했으며, 해당 행과 소스 해시만 다시 확인했다.

063의 두 차량 비교는 원래 바닥·같은 yaw에서392초 명목 구간·23,520물리 스텝을 진행했다.320초 정차의1Hz 평가 표본에서 기존 차량은 최대0.353154m, 위치 유지 차량은 최대0.000017216m 이동했다. 약1.2m 재출발·회전·두 번의 재정차 조건도 통과했다. 다른 joint gain과 모든 effort limit/mode의 보존을 확인했다. 원래 힘·접촉 원인이나 하드웨어 정확도를 확정하는 결과는 아니다.

측정된 native 경과 시각은 원본 그대로 남겼다. 응답0.2초는 gain 계산의 설계 입력이며 실측 정착 시간이 아니다. SDK 로그에는 `_MarkSprimDirty` 경고57건이 남아 있다. SDK close 반환 기록은 없지만 호스트 Job 종료·잔여 PID0·timeout 없음을 확인했으며 정리 오류는 없다. 원본45파일122,999,338바이트를 D·E에 보존했다. [063 근거](evidence/local45-parking-hold-063.json)에 실제 gain readback과 확인 범위를 남겼다.4대 공장 운송의 효과는 새 실행에서 확인한다.

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
