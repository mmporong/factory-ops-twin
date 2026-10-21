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

## FOT-062 — 첫 HOLD 응답이 부적격일 때 같은 허가의 후속 응답 누락

024는 네 차량의 정차 유지가 활성화된 상태로 초기화를 통과했지만 초기 HOLD 확인에서 실패했다. seq23의 발행 시각은5.783333635초, 첫 물리 평가 시각은5.783333634957671초다. 첫 표본은 기존 `issued<=evaluation` 조건에 맞지 않는다. 네 차량 모두 같은 허가의 유효 HOLD·0속도 물리 표본30개가 있고, 두 번째 이후 표본은 엄격한 후보 조건을 통과하지만 수신 체크포인트의 후보·확인 목록은 끝까지 비어 있다.

생산자 `after_control`은 첫 응답을 기록하면 같은 authorization sequence의 모든 후속 표본을 무시하고 있었다.060의 수신 버퍼는 이미 적격인 응답을 보관하는 기능이므로, 생산자가 내보내지 않은 후속 적격 표본을 얻을 수 없다.024의 원본 궤적과 동결된 생산자 소스로 이 동작을 확인했다. 첫 응답 파일은 마지막 emergency 응답으로 덮였으므로 당시 수신 파일 읽기 시각을 관측했다고 주장하지 않는다.022의 수신 순서가 기록되지 않았다는 한계도 유지한다.

수정은 첫 진단 응답을 기존대로 발행하면서, 같은 허가의 나중 물리 표본이 유효 HOLD·0입력·발행 이후 평가 조건을 만족하면 최초 적격 응답으로 한 번 갱신하는 범위다. 이후 같은 적격 sequence의 반복 기록은 막고, navigation/emergency의 기존 한 번 기록 동작은 유지한다. 원본 평가·표본 시각을 사용하며 시각 반올림·offset·epsilon 추가·허가 갱신·유효시간 확대는 하지 않는다. 최초 진단 표본은 원본 궤적에 남아 있다. 관련 CPU6개·설치 Python3.10 구문 검사를 한 번 통과했고, 실제 효과는 다음025에서 확인한다.

최초 오류는6.283334초의 초기 HOLD 확인 만료이며 SDK mission-failed와90초 정리 유예 초과는 후속이다. 양쪽 소유 프로세스 종료·정리 오류 없음을 확인한 뒤 원본102파일131,707,062바이트를 D·E에 보존했다. bag SQL과 새 영상·SDG 생성은 하지 않았다. 업무 주행·완료 주문·빈 토트 반납은0이며, 자재 상태 파일 부재는 재고0을 뜻하지 않는다. [024 근거](evidence/local45-transport-024.json)에 실제 표본과 재구성 범위를 남겼다.

## FOT-063 — ROS 미션 시작 시각이 같은 프레임의 World 시각보다 앞서는 경계

025의 초기 HOLD 확인은 네 차량 모두 통과했다. seq11의 첫 평가 시각4.600000239908695초는 발행 시각4.60000024초보다 작아 부적격이었으며, 생산자는 실제 다음278프레임의 적격 응답으로 갱신했다. 네 차량의 확인 시각4.633333575초 이후 실제 추가 tick을 거쳐4.650000243초에 미션을 시작했다.065 수정의 효과가 원본 표본과 수신 체크포인트 양쪽에서 확인됐다.

이때 ROS의 정수 나노초로 표현한 원본 미션 시작 시각4.650000243초가 같은279물리 프레임의 native World 시각4.650000242516398초보다 약0.484나노초 크다. `BarrierLifecycle.step`이 미래 시각으로 거절해 `G3 mission-start time is invalid`를 출력했다. 시계 소유자·발행률 문제가 아니라 두 표현의 경계에서 같은 프레임의 시각을 비교한 실패다. 시각 반올림·offset·epsilon으로 값을 바꾸지 않고, 원본 marker를 보관한 뒤 첫 실제 후속 물리 프레임에서 World 시각이 따라왔는지 확인하는 수정으로 대응한다. 그 프레임에서도 미래이거나 대기 중 marker가 변경되면 실패를 유지한다.

SDK는 최초 오류를 출력했지만 native 정리 단계에서 반환하지 않았다. ROS도 멈춘 시뮬레이션 시각을 보고 있어 호스트가 즉시 실패를 감지하지 못했다. 소유 Windows Job의 중단 요청으로 종료한 뒤 pair owner가 해당 Linux 단위를 닫았다. 이후 기록된 ROS context 오류는 이 정리의 후속이며 최초 원인으로 바꾸지 않는다. SDK의 원래 보고서는 `running`으로 끝났으므로 통과 자료로 취급하지 않는다.

재발 방지를 위해 로컬 SDK는 오류를 정리 전에 원자적 파일로 기록하고, 호스트 감시기는 같은 실행 ID의 실패를 확인하면 기존 양쪽 종료 경로로 진입한다. 관련 CPU11개를 한 번 통과했다. 실제 SDK의 실패 감지·종료 효과는 새 실행에서 확인한다. 재고는60개·소비0·적재0이고 주문 완료·빈 토트 반납은0이다. 소유 프로세스 양쪽 종료·정리 오류 없음을 확인한 뒤110파일218,306,272바이트를 D·E에 보존해 해시를 대조했다. bag SQL·새 영상·SDG는 생성하지 않았다. [025 근거](evidence/local45-transport-025.json)에 원본 시각과 확인 범위를 남겼다.

026에서는 실제 물리 시간이60초를 넘도록 미션 시작 오류 없이 주행·가상 충전이 이어졌다. 세 차량의 주행과 충전대 도착·정지에 따른 가상 충전 전환을 확인했다. 주문 완료·빈 토트 반납은0이며4번 차량은 업무 주행을 확인하지 못했다. 실행 속도를 개선하기 위한 별도 센서 시험에 앞서026을 의도적으로 종료했고, 양쪽 소유 프로세스 종료·정리 오류 없음을 확인했다. 원본과 선택한 웹 화면147파일334,623,047바이트를 D·E에 보존했다. [026 근거](evidence/local45-transport-026.json)는 host의 중단 결과를 SDK 오류나 완주 결과로 바꾸지 않는다.

## FOT-064 — 전체 실행 속도와 센서 관측을 보존하는 렌더 호출 축소 후보

026의 한 관측 구간에서89.744초 동안 World 시간은7.000초 진행했다. 이 구간에는 렌더링·ROS·기록을 모두 포함하므로 단일 원인을 확정하는 성능 측정은 아니다. 후보는 기존 `Local45Stepper`의 실제 물리 스텝과6스텝마다 수행하는 렌더 갱신이며, 도입 전 작은 장면에서 움직인 충돌 벽에 실제 PhysX 거리 배열이 반응하는지 확인한다. 정적인 배열 모양이나 World 시각 증가만으로 새 센서 취득을 입증하지 않는다. native 절대 취득 시각이 없다는 한계를 유지하고, 후보 시험과 전체 공장의 실측 속도를 구분한다. 실행 중인026의 동결 소스는 수정하지 않았으며, 시험에 앞서 해당 실행을 정리했다.

067의 작은 실제 SDK 시험은 baseline과 후보에서 각각36물리 스텝·실측0.600000031초를 진행했다. 충돌 벽의 중심을6·4·7m로 두 번 반복해 옮겼으며 두 방식 모두 실제 전방 거리가5.9·3.9·6.9m로 반응했다. 후보는6물리 스텝마다 수행한 렌더 뒤에 native 배열을 읽었다. 스텝 처리 구간은 baseline0.391초·후보0.077초로 약5.08배 차이가 있었으며, 시작·기록 시간을 제외한 작은 장면의 비교다. native 절대 취득 시각이나 전체 공장의 성능을 입증하지 않는다. SDK close 반환 기록은 없으나 host Job 통과·종료·잔여 PID0·timeout 없음과 정리 오류 없음을 확인했다. 원본30파일284,386바이트를 D·E에 보존했다. [067 근거](evidence/local45-physx-cadence-067.json)에 실제 거리와 범위를 남겼다.

임무 적용은 로컬4.5 PhysX 모드에 한정한다. 초기화 경로는 유지하고, 임무 중 물리 스텝마다 제어·시계를 갱신하며 예정된 렌더 뒤에만 센서를 읽어 발행한다. 센서 읽기를 렌더 갱신과 같은 반복에 두며, native 취득 시각 미확인 표시는 유지한다. 시작 후 첫 스캔 간격은 위상 전환으로 달라질 수 있다. 진단은 기존 소유 캐시만 읽고, RGB 촬영은 물리 동결을 확인하는 별도 경로를 사용한다. 바뀐 경계의 CPU4개·Python3.10 구문3파일을 한 번 통과했으며, 전체 공장 속도와 완주는027에서 확인한다.

027은 임무 전환 직후 native 물리 스텝 간격 검사에서 실패했다. 임무 시작은123프레임·2.050000107초이며 마지막 완료 궤적은124프레임·2.066666774초다. 실패한 스텝의 전후 쌍이 원래 오류에 없으므로 프레임 증가량·0시간 스텝·다른 간격 차이 중 원인은 확정하지 않는다. 오류에 실제 전후 frame/time과 차이·설정 dt를 덧붙이는 진단만 추가하고 검사·허용 오차·실패 상태를 유지한다. 관련 기존 CPU1개·구문1파일을 한 번 확인했다. 새 SDK 오류 체크포인트와 감시기는 실제로 동작해 양쪽 소유 실행을 자동 종료했으며 정리 오류는 없다. host 총 실행 시간은63.016초다. 미완성인 SDK 최종 보고서와 종료 중 ROS context 오류를 최초 step 검사 오류와 구분한다. 종료 후 원본84파일126,412,097바이트를 D·E에 보존했으며 bag SQL·영상·SDG를 생성하지 않았다. [027 근거](evidence/local45-transport-027.json)에 미확인 범위를 남겼다.

028의 추가 진단은 실패한 스텝 전후가 모두124프레임·2.0666667744517326초이며 frame 증가0·시간 증가0임을 보였다. 같은 프레임에서 촬영0의 물리 동결 검사는 완료돼 있다. 원래 촬영 래퍼는 재생 요청을 보낸 뒤 Kit 갱신 없이 반환하고, 설치 SDK의 렌더 없는 World.step은 재생 상태일 때만 물리를 수행한다. 따라서 촬영 후 대기 중인 pause/play 이벤트를 처리하는 경계가 수정 후보다. 실제 효과는029에서 확인하며,027의 원인 미확정 기록은 당시 한계로 보존한다. 후보는 물리를 진행시키지 않는 World.render 한 번으로 이벤트를 처리하고 실제 frame·시각·차량 pose가 그대로인지 확인한 뒤 다음 스텝을 수행한다. 실제 스텝 증가 검사와 시각·허가 기준은 유지한다.028도 SDK 체크포인트 감지로 양쪽 실행이 자동 종료됐고 정리 오류는 없다. 원본84파일126,436,366바이트를 D·E에 보존했다. [028 근거](evidence/local45-transport-028.json)에 원본 오류와 후보의 미확인 범위를 남겼다.

촬영 후 재개 수정은 관련 CPU6개·구문2파일을 한 번 통과했다. 성공한 촬영에서 기존 pause/resume 뒤에만 render 전용 callback을 수행하며, 해당 경로는 로컬4.5 PhysX 임무에 한정한다. 최신 체크포인트에 실제 render 전후 시각·프레임·재생 상태·차량 pose 일치 여부를 기록한다. 실제 SDK 복구 효과와 이후 촬영·운송은029에서 확인한다.

029의 실제 첫 촬영에서 render 전 재생 상태는false·갱신 후true였으며124프레임·2.066666774초·네 차량 pose가 정확히 같았다. 이후 실제 물리 시간42초와 후속 촬영까지 진행했다. 마지막 촬영7의 체크포인트도2560프레임·42.666668892초에서 같은 조건을 통과했다. 이 실행에서는 촬영 재개·물리 스텝 검사가 재발하지 않았다. 후속 실패는 아래 ROS 명령 수신 경계로 분리한다.

## FOT-065 — WSL에서 발행한 주행 명령의 SDK 미수신

029는 carter1 FollowPath의 진행 검사에서 status6·error_code105로 종료됐다. SDK의 모든 차량 명령 수신 sequence는0·수신 시각은없음·바퀴 목표는0이었다. 소유 Windows·Linux 종료를 확인한 뒤에만 원본 bag을 read-only SQL과 Jazzy 공식 Twist 역직렬화로 확인했다. carter1 명령1469건 중1468건, carter2 명령985건 모두0이 아닌 명령이며 최대 선속도는0.4m/s다. WSL 기록기의 수신은 SDK 수신을 입증하지 않는다. 당시 SDK의 DDS writer match·거절 callback 계수는 기록되지 않아 정확한 원인은 미확정이다.

SDK에는 ParticipantEntitiesInfo의 FastCDR·Badalloc 오류도 있으나 앞선 주행 성공 구간에서도 관측됐으므로 최초 원인으로 단정하지 않는다.4초짜리 보조 peer 조회는 소유 실행 종료 후 수행돼 빈 endpoint를 보였으며, 실행 중 상태로 인용하지 않는다. 다음 수정은 노드별 고정 executor로 콜백 소유 경계를 유지하고, 최신 진단에 발행자 정보·수신·거절 계수를 함께 남기는 범위다. 주행 허가·watchdog·시각·물리·시작점 기준은 유지한다. 원인을 해결했는지는 새 실제 실행에서 확인한다.

SDK mission-failed는 ROS FollowPath 실패의 후속이다. 양쪽 실행은 자동 정리됐고 정리 오류는 없다. 재고60·소비0·적재0이며 주문 완료·빈 토트 반납은0이다. 원본155파일211,402,568바이트를 D·E에 보존해 해시를 대조했다. 새 영상·SDG를 생성하지 않았다. [029 근거](evidence/local45-transport-029.json)에 원본 명령과 수신 미확인 범위를 남겼다.

071은 로컬4.5 PhysX의 첫 spin 전에 각 소유 노드를 해당 context의 고정 SingleThreadedExecutor에 등록한다. 기존4라운드·비차단 호출 예산을 유지하고 노드/context 정리 전에 executor를 정리한다. 기존1시뮬레이션초 체크포인트에서 명령 수신·거절·마지막 거절 사유·정확한 토픽·발행자 정보를 최신 파일로 기록한다. 설치 Humble Subscription에는 공개된 matched publisher 계수 API가 없어 해당 값은null·지원 불가로 남긴다. 제공되는 node.count_publishers 값은graph_publisher_count로 별도 표기하며 실제 DDS match로 바꾸지 않는다. 관련 CPU8개·구문2파일을 한 번 확인했다.029의 최초 원인 해결 여부는030의 실제 수신과 주행으로 확인한다.

030에서는 전용 executor가 동작했으나 최신 수신 진단은 수신0·거절0·토픽별 graph 발행자4를 보였다. 실제 DDS match는 지원 불가로null이며 이를 연결 성공으로 해석하지 않는다. 종료된 bag의 공식 역직렬화 결과 carter1 명령1550건 중1549건, carter2 명령1015건 중1014건이0이 아니고 최대 선속도는0.4m/s였다. 전용 executor만으로는 수신이 복구되지 않았다. FollowPath105가 다시 최초 오류이며 촬영 재개와 물리 스텝 검사는 재발하지 않았다. 보조 live 조회는 실행이 이미 끝났다는 사전 조건에서 중단돼 ROS 노드를 초기화하지 않았다. 종료 후152파일211,127,121바이트를 D·E에 보존했다. [030 근거](evidence/local45-transport-030.json)에 수신 진단과 한계를 남겼다.

072의 명시적 physics-cadence 인자는 동일 센서·시계·executor·허가 조건에서 기존 app_update와 owned_step6를 비교하기 위한 경로다. 기본값은 유지한다. 관련 CPU6개·구문4파일을 한 번 확인했다.031은app_update로 명령 수신과 첫 실제 주행부터 확인해 앱 이벤트 갱신 의존성을 좁힌다. 시각·물리·시작점 허용 기준을 바꾸지 않는다.

031은 동일 PhysX·시계·전용 executor에서 app_update로 진행했지만 명령 수신0·거절0·graph 발행자4와 FollowPath105가 재현됐다. 실행 중 사전·사후 host running을 확인한 독립 WSL 구독기는4초 동안 carter1 명령39건·최신 선속도0.4m/s를 받았다. 해당 graph에는 SDK cmd 구독기가 없었으나 graph 목록은 실제 DDS match를 입증하지 않는다. 종료 후 원본 bag은 carter1 명령2687건 중2686건·carter2 명령1753건이0이 아니었다. 재개·콜백 소유 경계 외의 데이터 전달 원인은 여전히 미확정이다. 종료 후149파일239,030,127바이트를 D·E에 보존했다. [031 근거](evidence/local45-transport-031.json)에 유효한 live 관측과 한계를 남겼다.

073은 프로젝트별 UDP 프로필의 interfaceWhiteList에 Windows와 WSL의 해당 로컬 IPv4를 각각 지정하는 선택지를 추가했다. 한쪽 주소만 주거나 숫자 IPv4가 아니면 파일·프로세스를 만들기 전에 거절한다. 두 XML의 경로·해시와 양쪽 환경을 실행 근거에 결속하며 기본 설정은 유지한다. [Fast DDS2.6 공식 문서](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/whitelist.html)로 문법을 확인했고 관련 CPU3개·구문2파일을 한 번 확인했다.032의 실제 프로필 적재와 명령 수신은 미확인이다. OS 방화벽·보안 정책은 변경하지 않았다.

032에서는 인터페이스를 지정한 DDS 설정으로 네 차량 모두 SDK 명령을 수신하고 물리 주행·작업 인계를 수행했다. 이는073 변경의 분리된 인과 검증은 아니며, 초기 수신0 스냅샷을 전체 실행의 수신 실패로 해석하지 않는다. 현재 상태를 읽지 않은 종료 판단과 잘못된 경로 조회를 반복하지 않도록, 실행 ID·최신 물리 시각·명령 계수·운영 진전을 함께 확인한다. 중간 종료 요청은 정확한 실행 ID 형식과 종료 조건을 확인한 뒤에만 작성한다. 이번의 잘못 작성된 요청은 실행 ID와 일치하지 않아 적용되지 않았고, 감시 경로 밖에 보존했다. 실제 종료 원인은 아래 예약 교착이다.

032는 네 차량의 작업 인계·주문1건 출하·자재4건 픽업·공정 투입3건·소비6개까지 진행했다. 빈 토트3건 중2건이 배정됐으나 반납 완료는0이다.681시뮬레이션초 부근에서 `persistent full-leg wait cycle; preserve reservations and fail-stop`이 최초 오류로 기록됐다. CI의 carter1은 P1 복귀를 위해 carter2를 기다렸고, S1의 carter2는 CI 운송을 위해 carter1을 기다렸다. 나머지 두 차량도 CI 진입을 기다렸다. 최종 진입 경로 E_CI→CI의28타일과 W_LOW→S1의36타일이 남아 있는 같은 조건에서 CPU 상호 거절을 재현했으며, 4대 개발 자재 임무의 최종 작업 지점에서 기존 HOLD ACK·신선한 연속 정지·도착·goal-finished를 확인한 뒤, 기존 shrink_hold로 정류장 차체 점유만 남기도록 수정했다. 반경·지도·차체 점유·주행 허가·원자적 예약 이전은 유지한다. [관련 CPU 회귀 검사](../tests/test_g3_business_station_hold.py)6개는 상호 거절 재현·현재 차체 충돌 유지·후속 진행·미입증/이동/오래된 정지/도착 불일치의 거절·관리자 거절 시 기존 점유 보존을 통과했다. 후속033에서 S1·S2·S3·CI·CO·OUT의 정지 후 점유 축소와 첫 빈 토트 반납을 확인했다. 전체 정상 종료는 다른 경로 계획 오류로 미완료다.

SDK mission-failed는 위 ROS 교착 실패의 후속이다. 양쪽 소유 실행은 자동 종료됐고 정리 오류는 없다. 재고52·운반 중2·소비6의 합계60은 보존된다. 원본과 표시된 부분 타임랩스350파일1,630,080,581바이트를 D·E에 보존해 해시를 대조했다. 실제 RGB54시점의24.7초 영상은 명목28배속의 정지 프레임 유지 방식이고, 새 SDG를 생성하지 않았다. [032 근거](evidence/local45-transport-032.json)에 실제 네 차량 주행과 미완료 범위를 남겼다.

## FOT-066 — Nav2 계산 경로와 예약 통로의 불일치

033에서는 네 차량 모두 물리 주행·작업 인계에 참여했고, 자재4건 픽업·공정 투입3건·소비6개·주문1건 출하와 사용한 빈 토트1건의 S3 반납을 확인했다. 반납 뒤 carter3이 C_HIGH(15,17)에서 C_LOW(15,5)로 복귀 경로를 계산하는 단계에서 `computed path swept envelope leaves reserved chunk tiles`로 종료됐다. SDK의 mission-failed는 후속 오류다. 두 OS 소유 실행은 자동 종료됐고 정리 오류는 없다.

종료 확인 뒤에만 bag을 열어 공식 Jazzy 역직렬화로 해당 `/carter3/plan`을 읽었다.899개 pose의 x범위는9.040928021683214~15m이고, 작성된 중앙 통로 x15에서 최대5.959071978316786m 왼쪽으로 우회했다. 같은 예약 타일과 평가 반경으로 원래 거절을 재현했다. 계산 결과가 주행에 사용되기 전에 거절됐으며, 단순 시작점 오차로 설명할 수 있는 크기는 아니다. Navfn이 이 경로를 선택한 비용 원인은 아직 확정하지 않았다.

지도·통로·반경·시계·도착 허용값·경로 입장 검사는 유지한다. 후속 수정은 같은 지도에서 작성된 경유점을 요구하는 Nav2 action을 비교하며, 통과한 정적 경로 비교를 실제4대 주행 성공으로 바꾸지 않는다. 실패 경로의 원문을 입장 검사 전에 기록하도록 보완해, 이후 거절 시 bag 분석 없이도 원인을 확인할 수 있게 한다. 원본과 파생 기록377파일1,819,654,947바이트를 D·E에 보존해 해시를 대조했다. [033 근거](evidence/local45-transport-033.json)에 실패·재고 보존·첫 반납·종료를 기록했다.

## FOT-067 — 빈 토트 렌더 확인용 스크립트의 초기화·촬영·정리 순서

076의 첫 정적 fixture는 로봇을 불러온 뒤 공장 콜라이더20개 검사를 수행해 거절됐다. 본 임무와 같이 공장 시각 구성 검사를 로봇 로딩 전에 수행하도록 순서를 바꿨다. 다음 시도는 paused World의 render 호출만으로 유효 이미지 버퍼를 얻지 못했다. 기존 촬영 경로와 같은 Replicator의 delta_time=0 호출로 바꾸고, 버퍼 모양·유효값·촬영 전후 물리 프레임과 시각을 결과에 남겼다. 이후 RGB 저장과 배치 기록은 통과했으나 `rgb.detach([HydraTexture])`가 문자열 경로를 기대하는 API에서 실패했다. 기존 SDK 사용 경로와 같은 인자 없는 detach로 수정했다.

수정된 네 번째 시도는640×480 RGB, 차량 위 반납 토트와 CI 대기 토트의 USD 배치, 촬영 전후 동일한 물리 프레임2·World시각0.03333333507180214초와 소유 Job 종료·잔여 PID없음을 확인했다. 앞선 실패3건은 원래 판정을 보존한다. 프로세스 exit0만으로 성공을 인정하지 않고, SDK 결과의 passed와 호스트의 closed·잔여 PID를 함께 검사한다.4개 시도93파일69,164,174바이트를 D·E에 보존했다. [076 근거](evidence/local45-material-visual-076.json)는 종료된033의 운영 스냅샷을 작성된 정적 차량 위치에 적용한 렌더 확인이며, 실제033의 RGB·주행 중 동기화·접촉/승강 검증과 구분한다.

## FOT-068 — 자재 상태 전환 직후 RGB의 이전 화물 표시

034의 시뮬레이션221.86667823791504초·촬영20번에서 native 빈 토트 문서 revision4가 CI의 F4-TOTE-002를 표시했다. 제품 시각 문서는 아직 sequence6의 carter2 적재 상태였고,21번 촬영221.88334490545094초에는 sequence8의 공정 처리·차량 화물 숨김으로 바뀌었다. 그러나 RGB20·21번에는 차량 위 파란 화물이 남았고,22번222.05001158080995초에는 푸른 잔상이 관측됐다. USD 적용 메타데이터와 물리 프레임 불변만으로 해당 순간의 픽셀 상태까지 일치한다고 판정하지 않는다.

렌더 버퍼 갱신 지연과 시간 누적 효과 중 어느 것이 원인인지는 미확정이다. 원본과 상태 전환 기록은 유지하고, 대표 미디어는 표시가 안정된 시점을 선택한다. 현재 실행의 소스·설정·주행은 바꾸지 않는다. 후속079는 local45 PhysX의 촬영 부분만 명시적 zero-delta Replicator 캡처로 보완하고, 같은 정지 물리 상태의 화물 표시 전환에서 기존 호출과 비교한다. 079 정적 비교는 같은 물리 프레임2·World시각0.03333333507180214초에서 현재 빈 토트 RGB를 확인했고 소유 Job도 종료됐다. 기존4회 render에서도 이번에는 현재 상태가 표시되어 지연이 재현되지 않았다. 결과는 limited이며 개선 효과를 입증한 것으로 기록하지 않는다. 실제 주행 전환의 후속 확인은 남아 있다. [079 근거](evidence/local45-visual-flush-079.json)를 보존한다. 이 실행에서 새 학습용 라벨은 생성하지 않았다.

## FOT-069 — 성공한 planner 결과의 수신 시점 유효 시간 초과

034는 모든 차량의 작업 참여·공정 소비8개·주문3건 출하·빈 토트2건 반납 이후 `computed path is stale`로 종료됐다. carter1의 C_LOW→P3_MERGE 경로는 Nav2 성공 결과였으나, 원본 stamp1507833411973ns와 입장 시계1508350078667ns의 차이가0.516666694초로 기존 path TTL0.5초를 넘었다. 거절된 경로 원문과 UUID를 입장 검사 전에 보존했으며 FollowPath에 전달하지 않았다. 외부7200초 제한 만료가 최초 실패가 아니고 SDK mission-failed는 후속 오류다. Windows Job과 Linux 단위·자식·클라이언트는 자동 종료됐으며 잔여 PID와 정리 오류는 없다.

같은034 실행에서 이전033의 중앙 통로 이탈 사례는 작성 경유점 기반 경로431개 pose로 통과했고, 실제 FollowPath 성공과 정지를 확인했다. 지도·예약 구역·반경·시계·TTL을 늘리지 않았다. 재고52·소비8의 합계60과 원래 실패 판정을 보존한다. 원본·파생 미디어610파일3,208,555,650바이트를 D·E에 보존하고 해시를 대조했다. [034 근거](evidence/local45-transport-034.json)에 성공한 부분과 미완료 주문·반납을 구분했다.

083은 오래된 결과를 폐기한 뒤 정지·예약 소유권을 유지하며 최대3회 새 계산을 요청한다. 원래 주행 구간 시작 시각과 제한 시간을 유지하고 새 경로에도 기존 입장 검사를 적용한다. 종료된034의 원본 Path를 사용하는 CPU 검사6건과 Python3.10 구문 검사를 통과했다. CPU의 예약·속도 상태는 시험 입력이며 실제034 전체 관측 재생으로 표시하지 않는다. TTL 증가·원본 stamp 변경·오래된 경로의 주행 허용은 하지 않는다. 036에서는0.500000026초의 만료 경로2건을 거절한 뒤 각각 새 UUID로 계산했다. 두 경로 모두 실제 FollowPath 성공(status4/error0)과 엔코더 정지를 확인했다. TTL0.5초·시계·통로 경계는 유지했다. 운송·주차는 완료됐으나 별도 종료 기록 경계 실패로 전체 실행은 실패다.

## FOT-070 — 정적 촬영 통과와 운행 촬영의 상태 불변 검사를 구분

035는 별도3주문·4대 입력과083 경로 복구로 준비했으나, 첫 촬영의 `industrial capture hooks advanced the frozen simulation state` 검사에서 거절됐다. SDK 오류 기록은38.484초에 생성됐고 등록된 촬영 시점과 작업 인계는0건이다.079의 zero-delta Replicator 호출이 정적 fixture에서 동일 World 프레임·시각을 유지한 사실은 운행의 SimulationManager 시계·프레임·차량 pose 전체 불변을 보장하지 않았다. 어떤 비교 항목이 달라졌는지는 별도 값으로 남지 않아 확정하지 않는다.

촬영 hook을034와 동일한4회 World.render 호출로 복원하고 바깥의 엄격한 상태 불변 검사는 유지했다.079 정적 원본은 limited 그대로 보존하며, 운행 개선 성공으로 쓰지 않는다. ROS context 종료 오류와 SDK 내부 정리 실패 표시는 원본에 남겼고, 호스트가 소유한 Windows Job·Linux 단위·자식·클라이언트의 종료와 잔여 PID없음은 별도로 확인했다. [035 근거](evidence/local45-transport-035.json)와 원본95파일134,158,122바이트를 D·E에 보존했다. 별도036 실행에서는 첫 촬영부터 최종134번째 촬영까지 엄격한 물리 시각·프레임·pose 불변 검사와 마지막 capture resume을 통과했다. 이후 최종 장면 확정에서 실패했으며 RGB 픽셀 전환 품질 문제의 해결로 표시하지 않는다. 정적 렌더 확인만으로 운행 촬영 경로를 수용하지 않는다.

## FOT-071 — 운송 완료 뒤 최종 장면을 확정하지 못한 종료 대기

036은3주문 출하·3빈 토트 반납·4주차 후 World1850.250096498초에 scenario_complete를 기록했다. 최종 RGB는 World1851.000096537초·프레임111060에서 촬영됐고, frozen state/capture resume 검사는 통과했다. SDK 장면 파일은 running/mission에 남았으며 최종 stamp와 validated/worker_exit_requested가 기록되지 않았다. ROS terminal recording60초 검사와 이후 SDK 종료 유예90초가 각각 만료됐다. 네 차량의 마지막 WSL odometry 누락이 원인이라고 확정하지 않는다. 장면이 running이면 검사는 odometry 대조 전부터 대기한다.

설치된 Replicator1.11.35의 wait_until_complete는 STEPPED만 자동으로 stop하고, 그 외에는 STOPPED까지 app.update를 반복한다. CaptureSession.close는 stop 요청 없이 drain을 기다렸으며 Windows에는 POSIX SIGALRM 제한이 적용되지 않는다. 종료된036에는 호출별 정체 위치가 없어 정확한 blocking call은 미확정이고, 생성 상태 종료 누락은 소스 근거가 있는 후보 원인이다. 원래 실패 판정은 보존한다.

로컬4.5 전용 stop_before_drain을 추가해 stop 뒤 기존 drain·annotator detach·product destroy를 유지한다. 기본6.2 경로의 옵션은 false다. [NVIDIA 현재 Replicator API](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/latest/source/extensions/omni.replicator.core/docs/API.html#omni.replicator.core.orchestrator.stop)는 stop 요청과 완료 대기를 구분한다.1.11.35 문서 URL이 최신판으로 이동하므로 세부 대기 조건은 설치된1.11.35의 orchestrator.py 배포 소스와 대조했다. 세 종료 checkpoint를 남기며 시간 제한·TTL·시계·물리 입장 조건은 늘리지 않는다. CPU 종료/오류/기본 경로와 기존 촬영 검사15건을 통과했고, 088 fixture는 세 번째 RGB priming 누락으로 종료 경로 도달 전에 실패했다. 이를 보완한089는 STEPPED 상태의 호환 종료를 확인했고,090에서는 공개 API로 작성한 PAUSED 상태에서0.016초 만에 STOPPED가 됐다. World0.133333340초·프레임8과3개 RGB 제품의 종료를 확인했다. Windows Job 종료와 잔여 PID없음도 확인했다. 이는 정적 생성 종료 시험이며 네 대 ROS의 최종 stamp/종료 경계 검증은 남아 있다. 이 수정과 fixture는036의 정상 재판정이나80% 완료 근거가 아니다.

## FOT-072 — WSL 속도 명령 발행과 SDK 수신의 불일치

수정 생산자 e3ea8a8의037은 같은3주문·4대 입력과9000초 외부 상한으로 시작했다. 첫 촬영·네 차량의 초기 정지/기동·Nav2 액션 수락 뒤 SDK의 cmd_vel 수신 계수는 네 대 모두0에 머물렀다. World37.316668613초에 carter1의 FollowPath가 status6/error105로 실패했다. 설치된 Jazzy nav2_msgs/FollowPath.action에서105는 FAILED_TO_MAKE_PROGRESS다. SDK mission-controller-failed는 그 뒤의 오류이며, 종료 stop/drain 수정이 이 수신 실패를 만들었다는 근거는 없다.

양쪽 소유 트리와 Job 종료를 확인한 뒤 bag의 토픽별 개수만 조회했다. WSL bag에는 carter1/cmd_vel1399건·carter2/cmd_vel937건이 기록됐지만 SDK 수신은0건이다. 이 개수는 명령의 값이0인지와 SDK 전달 성공을 입증하지 않는다. 그래프의 publisher4개와 설치된 Subscription.get_publisher_count 미지원/null도 실제 endpoint 매칭이나 전달 성공으로 해석하지 않는다. paired NIC 주소는 Windows192.168.96.1·WSL192.168.98.11로 유지됐다. 정확한 손실·타입/QoS·discovery 원인은 미확정이다.

원본147파일222,395,023바이트를 D·E operations-079에 보존해 SHA를 대조했다. 운송 인계·출하·반납은0건이며 새로운 대표 영상이나 SDG는 만들지 않았다. 전체 실행은 추가 반복 전에 작은 네 토픽 양방향 통신 재현으로 확인한다. ROS/Humble·Fast DDS2.6과 설치 SDK의 공식/배포 소스를 기준으로 버전을 구분하고, 최신6.2의 설정을4.5의 검증 완료로 옮기지 않는다. 수신 경계와 시작 전 준비 조건을 확인하며 물리·경로·시각·TTL 조건은 유지한다.

참고: [Fast DDS2.6 SIMPLE discovery와 initial peers](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/discovery/simple.html)는 participant 발견과 endpoint 발견을 나누고, 명시적 peer 및 예상 participant 범위를 설명한다. [NVIDIA 최신 ROS 설치 지침](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)의 WSL NAT/명시적 peer 설명은 후속 시험 후보의 근거이며 설치4.5의 동일 동작을 보장하지 않는다. 버전별 공식 근거와 재현 결과를 확보한 뒤 원인·수정을 갱신한다. [Humble QoS 상류 원문](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)과 [2.6.12 initial peers 설명](https://fast-dds.docs.eprosima.com/en/v2.6.12/fastdds/use_cases/wifi/initial_peers.html)도 확인했다.4.5 아카이브 페이지의 접근 실패로 지원표는 재확인하지 못했으며, 설치 확장4.1.15의 기본 distro가 humble인 사실만 확인했다.

기존 설정의093 양방향 네 토픽과094 공유 context 시험은 통과했다.095는 CPU fixture의 tf2_ros 미연결로 데이터 시험 전에 실패했고, 실제4.5 경로와 같은 Runtime45 TF 어댑터를 사용한096에서 네 G2 callback이 각각234건을 받았다.097의 실제 SDK 앱 안에서도 네 G2 callback이 각각234건 수신·거절0건을 기록했다.099는 설치된 Nav2 C++ TwistPublisher를 사용해 네 callback의237건 수신·거절0건을 확인했다. [Nav2 Jazzy 상류 소스](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_util/include/nav2_util/twist_publisher.hpp)의 기본 Twist 발행 경로와 설치 헤더를 대조했다. 모든 소유 트리 종료와 잔여 PID없음을 확인했다. 이 시험은 별도 probe 토픽과 작성된 watchdog 시계0을 사용하며 공장 시계·주행·실제 전체 Nav2 노드 구성을 검증하지 않는다. [짧은 전달 시험 근거](evidence/local45-dds-delivery-097.json)는037 오류를 재현하지 못한 범위로 기록하며, peer·QoS·방화벽 설정은 변경하지 않았다.

098은 기존 local45/PhysX4 시작 제한 안에서 초기 HOLD·정지·시작 위치를 유지하며 실제 여섯 성분이 모두0인 Twist를 네 운영 토픽에 보낸다. 같은 실행 SDK의 수신 계수 증가·수락한0속도·기존 TTL 안의 실제 수신 시각을 확인하고 임시 발행자를 모두 정리한 뒤 임무 원점을 설정한다. 그래프 개수·다른 실행·누락·오래된 문서·0이 아닌 값은 준비 근거가 될 수 없다. 기존 예약·주행 명령 입장 검사는 유지한다. CPU8건과 Python3.10 구문3파일을 통과했으며 실제 공장 실행 확인은 남아 있다. 이는 초기 전달 실패를 조기에 구분하기 위한 조치이며037의 원인을 해결했다고 표시하지 않는다. 반복 오류는 같은 실패 ID·재현 조건·확인된 원인/가설·수정·후속 결과에 누적하고, 관련 공식 문서는 버전과 적용 범위를 함께 기록한다.

043에서 같은 SDK 수신0·거절0이 초기0속도 probe에 재현됐다. 네 차량의 HOLD·정지·시작 위치는 모두 유효했지만 실제 명령 계수는 증가하지 않아 기존 시작 기한에서 종료했다. 최종 요약의 sdk_receipt_clock_not_current는 추가 신선도 조건이며, 계수0이라는 전달 실패를 대신 설명하는 원인으로 확정하지 않는다. 양쪽 실행 종료·정리 오류없음·162파일 C/E 보존을 확인했다.60fps 연속 촬영은 시작되지 않았고 새 주행 GIF를 만들지 않았다. [043 원본](evidence/local45-transport-043.json)

133은 같은 설치 SDK의 ROS와 WSL Jazzy에서4개의 격리 Twist 채널·Linux44/Windows5 participant를 사용해 명시적 peer 경로를 시험했다.16.828초에 Windows 수신180/179/177/179건과 Linux150/150/150/149건을 확인했고 양쪽 소유 트리·Job 종료와19파일 C/E 보존을 통과했다. GPU·로봇 명령·방화벽은 사용/변경하지 않았다. 실제 공장 장애의 유일한 원인이나 복구 성공으로 확대하지 않는다. [133 결과](evidence/local45-dds-explicit-133.json)

다음 실행은 기존 UDP·NIC 제한을 유지한 `--dds-explicit-peers` 옵션으로 두 로컬 주소와 maxInitialPeersRange120을 실행별 XML에 기록한다. [Fast DDS2.6.12 문서](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/discovery/simple.html)의 initial peers·participant 범위 설정을 적용한 후보이며, 기존 기본 동작은 유지한다. 관련 설정/기동 검사4건은0.143초에 통과했다. 실제 기동 장벽·네 차량 운행·60fps 촬영은 다음 새 실행에서 확인한다.

## FOT-073 — 장시간 운행 중 위치 TF의 수신·처리 신선도 실패

038은 시작 전 네 차량의 실제0속도 수신과 임시 발행자 정리를 통과했다. 네 차량의 제품 또는 빈 토트 인계, 출하2건·빈 토트 반납2건·가상 충전을 기록한 뒤 carter1 위치 TF 검사에서 종료됐다. 원본 TF stamp1399.000072964초와 검사 시계1399.550072992초의 차이는0.550000028초로 기존 TTL0.5초를 넘었다. 외부9000초 상한 만료가 아니며 실제 호스트 실행은6553.312초였다. Windows Job과 Linux 소유 트리·클라이언트 종료, 잔여 PID없음과 정리 오류없음을 확인했다. 전체 실행과80% 판정은 실패/보류 그대로 유지한다.

기록기 종료를 확인한 뒤 마지막2초 구간만 SQL·Jazzy 공식 역직렬화로 읽었다. 실패 시각보다 작은 기록 시각1399.500072990초의 odom→base_link TF와1399.850072982초까지의 map→odom TF가 bag에 있었다. 기존 합성 TF stamp1399.000072964초보다 새로운 원본 odom TF10건이 기록돼 있었다. 이 기록은 G3의 해당 callback 실행 여부나 정확한 도착 순서를 입증하지 않는다. callback drain21167회 모두 협력적0.05초 예산 경계에 도달한 사실도 관측됐지만 실제 backlog 크기로 단정하지 않는다. 원본 센서 중단·AMCL 지연·G3 처리 지연 중 정확한 원인은 미확정이다. 과거 q009 callback backlog 기록과 기존 FOT-003·FOT-041 계열 후속 TF 오류도 함께 확인해 같은 메시지의 최초 원인을 구분한다.

[Jazzy TF Buffer 상류 소스](https://raw.githubusercontent.com/ros2/geometry2/jazzy/tf2_ros_py/tf2_ros/buffer.py)는 Time0을 버퍼의 최신 변환 조회로 정의하며, 단일 executor를 막는 동기 대기에 주의를 둔다. [Jazzy rclpy executor 상류 소스](https://raw.githubusercontent.com/ros2/rclpy/jazzy/rclpy/rclpy/executors.py)는 spin_once가 한 callback을 처리한다고 설명한다. 최신 branch와 설치 소스의 버전을 구분한다. 다음 수정은 오래된 pose를 허용하지 않고, 제한된 callback 재처리 뒤 실제 시계를 다시 읽어 신선한 TF만 수용하는 범위다. TTL·원본 stamp·시계·예약 반경·임무 제한은 늘리지 않으며, 지속 누락은 실패로 남긴다. 원본 실패를 새 성공으로 바꾸지 않고 전체 운행 재시작 전 작은 재현으로 확인한다.

후속102는 준비 게이트를 통과한 로컬 PhysX4 경로에서만 최대3회 기존 bounded drain을 호출하고 매번 현재 시계를 다시 읽는다. 네 차량의 신선한 위치가 확보되기 전 예약 관측·허가 갱신을 수행하지 않으며, 계속 누락되면 실패한다. 각 drain의64라운드·0.05초 협력적 예산과 TTL0.5초는 유지한다. 종료된038의 시각/헤더 값을 사용하는 CPU8건과 Python3.10 구문2파일을 통과했다. 이 fixture의 geometry·callback 순서는 작성된 값이며 실제038 재생으로 표시하지 않는다. 실제 통합 검증은 남아 있다. [102 근거](evidence/local45-localization-dispatch-102.json)를 참고한다.038 원본525파일2,869,986,827바이트는 D·E에 보존해 해시를 대조했다.

041에서도102의 최대3회 bounded dispatch를 사용한 뒤 신선도 실패가 재발했다. 마지막 시계1211.766729865초에서 carter1 TF stamp1211.200063169초는0.566666696초 전이었고 기존 TTL0.5초를 넘었다. 세 번째 dispatch에서 다른 세 차량은 신선한 관측을 확보했다. 이는102만으로 전체 문제를 해결하지 못했다는 근거이며, 원본 TF 수신·executor 처리·버퍼 조회 중 원인은 아직 분리하지 않았다. 출하2/3건·빈 토트 반납1/3건 뒤 host7218.891초에서 실패했고9000초 상한 만료가 아니다. 양쪽 OS·Job 종료 뒤519파일의 원본을 C/E 무손실 압축566,719,474바이트로 보존해 모든 멤버 SHA를 대조했다. [041 결과](evidence/local45-transport-041.json)를 기준으로 다음 수정을 준비하며 같은 소스로 장시간 운행만 반복하지 않는다.

041의 기록기가 닫힌 뒤 마지막2초만 SQL·공식 Jazzy 메시지 역직렬화로 확인했다. 실패 시계 이전에 기존 G3 합성 stamp보다 새로운 odom→base_link TF11건이 기록돼 있었고 마지막 원본 stamp는1211.750063198초였다. 이는 G3 callback 실행이나 버퍼의 최신 내용을 입증하지 않는다. 후속 진단은 원본 센서 중단으로 단정하지 않고 G3 수신·dispatch·TF 버퍼 경계를 확인한다. [닫힌 기록의 최소 진단](evidence/local45-localization-window-121.json)은 기존041 보존 이후에 생성했으며 원래 압축본은 수정하지 않았다.

## FOT-074 — stateful XY 통과 뒤 정지 위치와 다음 출발 허용 범위의 불일치

039는 출하2건·빈 토트 반납1건 뒤 carter4의 P4 출발에서 `observed start is outside authored chunk start tolerance`로 종료됐다. 같은 차량의 이전 P4 도착 오차는0.1629811147m, 다음 계산 요청 시 오차는0.1629747727m다. 도착 허용0.25m는 통과하지만 다음 구간의 시작 제한0.15m는 초과한다. 오차는 도착 때 이미 있었으므로 장시간 주차 중 새 위치 점프가 발생했다고 설명하지 않는다. 시각/TTL 만료나 외부9000초 제한이 최초 원인이 아니며, 호스트7047.531초에서 양쪽 OS·Job이 닫혔다. SDK mission-failed는 후속 오류다. 원본493파일2,708,495,330바이트는 C/E operations-083에 보존했다.

프로젝트 GoalChecker는 stateful XY 통과를 유지한 채 Nav2의 방향 성공과 정지 속도만으로 최종 성공을 반환했다. 현재 XY 오차를 기록했지만 성공 조건에는 적용하지 않았다. [Nav2 Jazzy SimpleGoalChecker 상류 소스](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_controller/plugins/simple_goal_checker.cpp)의 stateful XY 동작과 구분한다. 상류 branch는 변경 가능한 자료이며 설치 배포와 동일한 릴리스라고 표시하지 않는다. 회전 중 물리 이동·위치 추정 중 어떤 효과가 오차를 만들었는지는 별도 미확정이며, 확인된 문제는 이 오차가 남은 정지를 도착 성공으로 처리한 경계다.

후속105는 terminal stop handshake가 켜졌을 때 현재 XY0.06m·yaw0.15rad와 정지 속도를 모두 요구한다. 움직이고 있으면 기존 제동을 먼저 수행하고, 멈춘 뒤 XY가 벗어나면 stateful XY를 재무장해 같은 예약·원래 목표에서 RPP가 재접근한다. 회전 중의 XY latch, handshake 비활성 시 stock 동작, 기존 시작0.15m·station0.25m·시계/TTL·지도·반경은 유지한다. 실제 C++ GoalChecker·RPP 재접근 검사2묶음과 구/신 trace 해석 검사1건을 통과했으며, 새 전체 물리 운송과 종료는 아직 확인 전이다. trace의 `terminal_pose_recheck`를 선택 필드로 결속해 과거 실행의 의미를 새 동작으로 바꾸지 않는다. [원본/수정 근거](evidence/local45-anchor-terminal-105.json)를 보존한다.

105의 첫 빌드 명령은 PowerShell에서 WSL 인라인 셸로 여러 줄을 넘기며 경로 변수가 손상돼 로그 파일 경로의 권한 오류로 중단됐다. ROS/C++ 빌드 실패와 구분한다. LF 형식의 소유 셸 파일을 생성해 WSL에서 파일 경로로 실행한 뒤 구성·빌드·관련 검사·설치를 마쳤다. 다음 WSL 빌드도 파일 실행을 사용하며 같은 인라인 전달을 반복하지 않는다.

## FOT-080 — 장착 후보 검사에서 반경 경계를 산술 여유로 넓히지 않음

115의 합성 CPU 입력에서 설정 반경이 측정 외곽보다5e-9m 작은 후보가1e-8m 산술 여유 때문에 허용되는 것을 확인했다. 반경 비교의 덧셈 여유를 제거하고 기존 반경보다 크면 거절하도록 수정했다. 관련 경계·차체/센서 간섭 검사1건은0.059초에 통과했다. 작성자의 CPU5검사와 원래3소스 SHA·receipt는 보존하며 root 수정 이후 해시를 별도로 연결한다. 실제 장착·센서 가시성·주행은 아직 미확인이다. [통합 근거](../coordination/collab/results/loaded-carrier-fixture-115-integration.json)

## FOT-078 — 합산 접촉 센서만으로 네 지지점을 입증하지 못함

접촉 시험111은720샘플 중714샘플에서 정지,717샘플에서 정렬을 기록했다. 마지막 토트 접촉 힘은58.8600N이었으나 carrier 센서의 raw actor pair는 Stand_0 한 곳만 보고했고 지지점 수는 최대1이었다. 토트가 놓인 모양이나 합산 힘만으로 네 지지대의 접촉을 성공 처리하지 않았다. 단일 센서 raw pair가 일부만 보고한 정확한 내부 원인은 아직 확정하지 않았다. 설치된4.5 ContactSensor의 raw pair 해석과 센서 생성 시 부모에 PhysxContactReportAPI를 적용하는 코드를 확인했다. 후속에서는 네 지지대 각각에 센서를 추가하고, 알려진 carrier/stand pair·힘·접촉 상태·센서 시각이 모두 유효할 때만 네 지지를 인정한다. 기존 dwell·정지·정렬 한계는 유지한다. [111 원본](evidence/local45-tote-contact-111.json)은 실패 상태로 C/E에 보존했다.

113에서는 정적 collider에 붙인 지지대 센서의 시각·frame이0에 머물렀다. SDK 로그가 네 센서 모두 rigid-body 부모가 없어 생성되지 않았다고 명시했다. 정적 collider에 ContactReportAPI를 붙이는 것만으로4.5 ContactSensor의 부모 조건을 충족하지 않는다. 후속114에서는 지지대를 움직이지 않는 kinematic rigid body로 선언한다. carrier/tote는 기존 동적 body로 유지하며 센서의 invalid 상태를 성공으로 우회하지 않는다. [113 원본](evidence/local45-tote-contact-113.json)을 보존했다.

114에서는 네 지지점·토트 접촉과 정지·정렬 dwell을 실제로 확인했고, 상향 속도를 준 뒤 분리 dwell·유효 RGB2장까지 통과했다. 실제72샘플·World0.616666699초·host22.041804초이며, Job 종료·잔여 PID없음과20파일의 C/E 보존을 확인했다. [114 결과](evidence/local45-tote-contact-114.json)는 지지대 위 독립 구성 시험이며 차량 장착·운송·승강 메커니즘의 성공 근거로 확대하지 않는다.

## FOT-079 — Windows 작업 파일과 Git 동결본의 줄바꿈 해시 차이

041 준비 시109의 빌드 입력과 Git archive의 `test_telemetry_goal_checker.cpp` SHA가 달라 사전 조건에서 중단했다. 작업 파일의 CRLF와 저장소의 LF 차이였으며 나머지13파일은 같았다. 실패한 준비 폴더와 원본109 빌드는 유지한다. 실제 운행에 사용할112 빌드는041의 Git 동결본을 원본으로 삼아 새 디렉터리에 구성했다. 관련 C++3묶음·설치가0.97초 검사에 통과했고14소스·설치 해시를 준비 metadata와 연결했다. 빌드 뒤 해시만 바꾸거나 LF 정규화를 동일 바이트 증명으로 대신하지 않았다. [112 근거](evidence/local45-goal-canonical-112.json)를 보존한다.

## FOT-077 — 토트 모듈의 USD 식별자와 Python 버전별 manifest 차이

첫 native 접촉 시험110은 `post_-1_-1`을 USD prim으로 만들며 잘못된 식별자 오류로 종료됐다. 물리 접촉·미디어 생성 전 실패이며, Job 종료·잔여 PID없음과 실패 원본의 C/E 무손실 보존을 확인했다. 모듈 생성 이름을 `post_m_m` 등 영문·밑줄 형식으로 고쳤고 형상 이름 검사를 추가했다. 같은 설정의 질량 합산은 Python3.12와 SDK3.10의 `sum` 차이로 마지막 자릿수와 manifest SHA가 달랐다. 질량·질량중심·부피 합산에 `math.fsum`을 사용한 뒤 두 실제 Python에서 manifest SHA `406aec0f70fbea4a74f1462d05d53395055b7b5182fb5dcab8749732b55307f4`가 같았다. 변경 관련 CPU1검사는0.004초에 통과했다. 이전107 receipt의 SHA는 당시 CPU 결과로 보존하며 새 native 성공으로 바꾸지 않는다. [110 실패](evidence/local45-tote-contact-110.json) 이후 새 실행으로 실제 접촉·분리·RGB를 확인한다.

## FOT-076 — 정지 후 지나치게 좁은 XY 재획득으로 재접근 반복

105를 적용한040은 carter1 FollowPath의 FAILED_TO_MAKE_PROGRESS로 종료됐다. 양쪽 OS·Job 종료와 C/E 원본 보존을 확인한 뒤 carter1의 닫힌 controller trace2847건을 분석했다. World92.750004837초에는 XY오차0.0993923060m·yaw오차0.0531873613rad,108.250005646초에는 XY오차0.0612047650m·yaw오차0.0514956354rad였고, 두 번 모두 정지 상태·기존 stateful goal 통과 뒤 XY를 재무장했다. 기존 다음 출발 한계0.15m 안이지만105의 현재 XY0.06m 조건을 만족하지 못했다. 재접근 두 번 뒤 진행 검사 실패가 이어졌다. 회전 중 위치 변화의 물리·추정 원인 전체를 이 기록만으로 확정하지 않는다.

109는 최초 XY획득0.06m와 회전 중 latch를 유지하고, 최종 정지 시 기존 다음 출발0.15m·yaw0.15rad 안인지 확인한다.0.15m 밖에서 멈춘 경우에만 XY를 재무장한다. 최초 획득·다음 출발·station·지도·반경·시간 제한은 넓히지 않았다. 실제 GoalChecker·RPP·plugin loading C++3묶음과 구/105/109 trace 해석1검사를 통과했다. trace에 terminal XY한계와 현재 terminal pose판정을 결속하며0.25m로 넓힌 기록은 거절한다. 실제 후속 운행과 정상 종료는 아직 미확인이다. [040 실패](evidence/local45-transport-040.json)·[109 근거](evidence/local45-terminal-reapproach-109.json)를 보존하고 같은105를 재실행하지 않는다.

## FOT-075 — 종료 원본의 보관 공간과 후속 실행 준비

039를 두 저장소에 보존한 뒤 E의 남은 공간은 새 정상 실행의 비압축 복사본을 담기에 부족했다. 닫힌038·039의 전체 자료를 각각597,988,966바이트·553,105,854바이트의 무손실 압축본으로 만들고, C/E의 두 압축본을 전부 풀어 읽으며 멤버 집합·크기·SHA를 원래 명세와 대조했다. 원본의 성공/실패 판정이나 파일 내용을 바꾸지 않았다.

038의 E 비압축 폴더는 원본을 보존하는 이동으로 C의 relocated-106에 옮겼으며525파일의 전체 해시를 다시 대조했다. D의038 원본·E의 압축본과039의 C/E 원본은 유지한다. 변경된 보관 위치는 [106 색인](evidence/local45-storage-106.json)에 남긴다. 후속 보존은 실제 크기를 확인한 C/E 무손실 압축으로 계획하며, 이전 raw 복사 기준이나 과거 E 경로를 현재 위치로 추정하지 않는다. 원본은 기록기·소유 실행 종료 뒤에만 압축한다.

## FOT-081 — 새 운행 제어의 명시적 활성화와 기존 outbox 동작 구분

116의 격리 후보를 수신한 뒤 기존 external-source outbox API 검사에서 pending 대신 rejected가 반환됐다. 새 `live_control` 계약의 opt-in 조건을 모든 외부 outbox에 적용한 회귀였다. 새 계약은 명시적 활성화를 유지하고, 기존 생산자의 설정된 outbox 동작은 보존하도록 고쳤다. 기존 API와 새 계약의 기본 관측 전용 경계2검사는0.583초에 통과했다. 새 모듈을 활성화된 API의 생산자 소스 해시에 포함했다.

실제 제어는 다음 예약 정차점에서 네 차량의 새 HOLD·정지 관측을 받은 뒤 ACK하는 fleet stop이다. physics·ROS·MES·가상 충전 시계까지 멈춘다고 표시하지 않도록 UI 문구를 바꾸고, 별도 제어 revision 대신 ledger revision을 상태 표시에 사용했다. 변경 뒤 TypeScript 검사는3.684초에 통과했다. 코드는 격리 브랜치8c963388에 보존했으며 main의 실행 중 API는 유지한다. 실제 ROS 정지→새 정지/HOLD ACK→재개는 아직 확인하지 않았다. [수신·수정 근거](../coordination/collab/results/live-anchor-controls-116-integration.json)

## FOT-082 — 저장소 모듈을 불러오는 수신 도구의 실행 경로

117 수신 도구를 `python artifacts/local45/receive_117.py`로 실행하며 `factory_ops` import가 실패했다. 스크립트 경로로 실행하면 import 시작 경로가 `artifacts/local45`가 되는 보조 도구의 문제이며 SDK·물리 시험 실패가 아니다. 이미 복사한 네 소스의 SHA를 보존한 채 저장소 루트에서 `python -B -m artifacts.local45.receive_117`로 실행해 입력6개·수신4소스·manifest를 대조했다. 프로젝트 모듈을 사용하는 후속 수신 도구는 저장소 루트의 모듈 실행을 사용하며, 실제 import 대상이 해당 저장소인지 확인한다. 제품 소스와041 실행에는 변경이 없다.

## FOT-083 — 장착 후보를 실제 합성 형상과 대조한118의 거절

118은 실제 Nova Carter를 불러온 뒤 네 post 지지면·차체/센서 간섭·기존 반경 검사에서 장착 후보를 거절했다. 물리 장착·운송·RGB 촬영 전 종료다. 특히 composed collision AABB 반경이71.1667m로 계산돼 caster collider의 extent/변환 해석부터 확인해야 한다. 이 값을 실제 로봇 반경으로 확정하거나, 후보를 통과시키기 위해 설정 반경을 넓히지 않는다. 기존 단독 지지대용 후보를 차체 좌표에 그대로 놓는 가정도 실제 측정과 분리해 수정한다. [118 원본](evidence/local45-loaded-carrier-118.json)은19파일·Job 종료·잔여 PID없음과 C/E 보존을 확인했다.

후속 측정은 [OpenUSD22.11 Boundable 상류 소스](https://raw.githubusercontent.com/PixarAnimationStudios/OpenUSD/v22.11/pxr/usd/usdGeom/boundable.h)의 geometry extent 계산을 사용해 실제 형상 파라미터에서 경계를 다시 계산하며 원래 cached extent와 함께 기록한다. 재질/셰이더 이름에 lidar가 들어가는 경우를 센서 프레임으로 수집하지 않도록 Xformable 조건을 적용한다.118 차체 외곽 상단에 저작된2mm 장착 간격을 더한 새 후보 설정을 별도로 작성했으며, 기존 후보와 반경은 보존한다.

122의 설치 USD22.11에서 caster Cylinder cached extent는[-50,50]였고 실제 파라미터 기반 extent는 반경0.0755m·폭0.025m였다. 새 측정의 원래/적재 외곽 반경은 모두0.6339164009m로 기존 설정0.6371090915m 안이다. 네 지지점·센서 간섭과 반경 검사에서 새 후보가 허용됐고 외부 fixed joint를 작성했다. 이후 wheel getter 오류로 종료돼 물리 안착 성공으로 세지 않는다.

독립적인119 고정 롤러 시험에서는6kg 토트를 실제 회전 조인트와 접촉 마찰로 이동시켜 적재·반납을 확인했다. World9.691667초·198개 희소 관측·RGB2장·정리 오류없음·Job 종료·21파일의 C/E 보존을 통과했다. [119 결과](evidence/local45-roller-transfer-119.json)는 고정 시험대 이송 근거이며118의 장착 거절을 해결한 것으로 표시하지 않는다.

보조 JSON 수신/편집 도구는 `read_text(encoding='utf-8')` 또는 BOM 파일의 `utf-8-sig`를 명시한다. Windows 기본 cp949로 한글 receipt를 읽으며 발생한 decode 오류는 제품·SDK 실패와 구분하고 UTF-8로 다시 읽는다. 파일 경로 실행의 import 문제는 FOT-082의 저장소 루트 모듈 실행을 유지한다.

## FOT-084 — SDK4.5 wheel velocity 편의 getter의 내부 이름 오타

122가 첫 실제 wheel readback에서 `WheeledRobot.get_wheel_velocities()`의 `_wheel_dof_indicies` AttributeError로 종료됐다. 설치된 공식 클래스는 생성·초기화·공개 property에서 `_wheel_dof_indices`를 사용하지만 해당 getter만 철자가 달랐다. 설치 SDK는 수정하지 않고 공개 `get_joint_velocities()`와 `wheel_dof_indices`로 같은 측정값을 읽도록 probe를 고쳤다.122 원본은 거절/실패 상태로 보존하고 후속 native에서 실제 정지·접촉·RGB를 확인한다.

SDK의 `python.bat -c`에 따옴표가 중첩된 코드를 넘기지 않고 UTF-8 Python 파일 경로를 사용한다. 이 배포의 `pxr`는 SimulationApp 초기화 후 로드하며, 초기화 전 import 실패를 geometry API 실패로 해석하지 않는다. 관련 보조 시도는 앱/물리 시험과 구분한다.

## FOT-087 — 장착 토트의 실제 주행과20fps 촬영

128은 실제 바퀴 목표와 기존 DriveProfile로6kg 토트를 운반했다. 전진 구간0.22447m, 곡선 yaw0.23109rad, 제자리 회전 yaw0.30651rad와 엔코더 방향을 확인했다. 적재 접촉·상대 위치·기존 각도 한계는 매 물리 스텝 확인하고 마지막0.4초 정지 dwell을 통과했다. 실제120Hz 물리에서6프레임마다 RGB를 받아 서로 다른188장·20fps·9.4초를 만들었다. 영상 노출 시계의 별도 정합과 공장 도킹·재고 인계는 미검증이다.62.08초 후 정리 오류 없이 Job이 닫혔고214파일의 C/E 보존을 확인했다. [128 결과](evidence/local45-loaded-mobility-128.json)

## FOT-088 — 완료된 주차 예약을 실행 중 승인으로 오인한 정지 판정

042의 실제 정지 요청은600wall 초 안에 applied가 되지 않았다. 관측한 차량 속도는0에 가까웠지만 연속 정지·현재 HOLD·추정 위치 조건의 전체 성공을 단정하지 않았다. 기존 exact-run Job abort로 종료하고 양쪽 OS closure·정리 오류없음·488파일의 C/E 보존을 확인했다. 정상 운행 및 운행 제어는 실패/미완료 상태로 유지한다. [042 원본](evidence/local45-transport-042.json)

예약 관리자는 목표 완료와 hold 축소 후에도 accepted_goal_uuid를 보존하고 parked=True를 둔다. live 정지 판정은 accepted UUID가 있으면 무조건 거절해 완료된 정상 예약도 통과할 수 없었다.130은 같은 UUID에 결속된 parked=True 예약을 기존 pose·연속 정지·ACK 검증으로 넘기며, 미완료/다른 UUID는 거절한다. 실제 예약 관리자의 reserve→accepted→finished→shrink 상태를 사용하는 회귀와 기존 거절 경계3검사를 통과했다. 로봇별 미충족 조건은 정지 요청당 변화16행 이내로 남긴다. 이 결함이042의 유일한 원인이라는 주장과 native 성공 판정은 보류한다. [130 수신](../coordination/collab/results/native-anchor-pause-130-integration.json)

## FOT-089 — Kit 클라이언트의 live 요청 계약과 부분 결과 인수

Kit의 기존 요청에는 native next-anchor 계약의 run_id·UTC requested_at이 없었다.132의 부분 구현을 보존한 뒤 기존 control 페인의 모델 용량 오류·idle을 확인해 root가 이어받았다. 모델 변경이나 새 에이전트는 사용하지 않았다. Kit는 지원 capability·현재 run/revision을 검사해 요청하고 API 접수와 같은 원천의 pending/applied를 구분한다. 실행 변경 시 이전 요청·확인 캐시를 지우며 재전송하지 않는다. UI는 다음 예약 지점 정지와 계속 흐르는 시뮬레이션 시계를 설명한다.

관련 Kit 클라이언트21검사·Python3.10 AST4파일을 통과했다. 새로 import하는 live_control.py도 후속 native probe의 소스 해시에 포함했다. 실제 Kit UI 및4대 실행의 적용 성공은 아직 확인하지 않았으며 독립 승인으로 표현하지 않는다. [root 완료 기록](../coordination/collab/results/kit-live-controls-132-root-completion.json)

## FOT-090 — 실제60fps 적재 촬영과40fps GIF 재생 간격

공장043의 명령 미수신과044의 공유 파일 I/O 오류로 공장60fps 촬영은 시작되지 않았다. 통합 운행 실패와 영상 인코딩을 분리해 기존 Windows 단일 SDK의 적재 주행을135에서 촬영했다. 물리·접촉·하중·조인트·정지 한계는 유지하고, 명시적60fps/660장 상한 프로파일만 추가했다. 실제60fps·서로 다른564장과 적재 주행·최종 정지 dwell을74.344wall 초에 통과했다.590파일을 C/E에 보존했다. [135 원본](evidence/local45-loaded-mobility-135.json)

5초 발췌 GIF는200프레임이며 프레임 지연20/30ms·합계5000ms, 유효40fps를 ffprobe로 확인했다. 파일은1,630,728바이트,9.4초 원본60fps MP4는257,347바이트다. 프레임 수 기준 기존10fps의4배이며 체감 품질이4배라는 수치 주장은 하지 않는다. 원본 데이터는 보간하지 않고 표시용 영상의 양자화·해시·변환 명령을 기록했다. 공장 조감 GIF는 기존 실제10fps 창을 유지하며135를 같은 공장 운행으로 표시하지 않는다. [미디어 출처](images/loaded-mobility-local45-135/provenance.json)

인코딩 도구는 기존 ffmpeg/ffprobe를 사용한다. 호스트 .venv에는 Pillow가 없어 최초 보조 도구의 import가 실패했으며, 새 패키지를 설치하지 않고 ffprobe의 실제 프레임 duration으로 재생 간격을 읽었다. 설치 SDK의 RGB 저장 기능과 이 보조 도구의 실패를 구분한다.

## FOT-091 — 공유 파일 I/O 오류와 결과 저장 오류의 분리

044는 ROS foreground에서 OSError errno5를 보고한 뒤 mission-result 임시 파일을 쓰다가 다시 같은 오류가 났다. 최초 실패의 전체 trace는 결과 파일에 남지 못했다. SDK90초 정리 대기는 ROS 종료 이후의 후속 오류이며, 실제 최초 파일/시스템 원인은 확정하지 않는다. 양쪽 소유 트리 종료·정리 오류없음과156파일 C/E 보존을 확인했다. 명시 peer·공장60fps 촬영 성공은 이 실행으로 입증되지 않았다. [044 원본](evidence/local45-transport-044.json)

136은 WSL native와/mnt/c에서 작은 JSON128회씩 쓰기·원자 교체·읽기를 수행했다. 각각0.07589초·1.12234초, 오류0건으로 실제 장애를 재현하지 못했다. 이를 공장 전체의 속도나 해결 근거로 확대하지 않는다. WSL의 과거 kernel 경고도 현재 오류와 시각 결속이 없어 원인으로 쓰지 않는다. [136 진단](evidence/local45-filesystem-136.json)

공유 checkpoint 읽기·교체의 기존0.2wall 초 한도에서 errno5를 처리하고, 임시 checkpoint 쓰기에도 같은 한도의 일시 오류 처리를 적용했다. append 저널에는 재시도를 넣지 않는다. ENOSPC·다른 영구 오류는 즉시 실패하고 지속 오류는 성공으로 반환하지 않는다. 관련 fault injection2건·기존 공유 파일5건을 통과했다. G3의 최초 예외는stderr와 native 임시 저장소의 run별 JSON에 보존하며 뒤의 파일/로그 실패로 덮어쓰지 않는다. 이 경로1검사와 Python3.10 AST4파일을 통과했다. native 임시 파일은 이후 오류 발생 시 소유 실행 종료 뒤 원본에 함께 수집한다. 새 실제 통합 운행에서 효과를 확인하며 TTL·물리·임무 기한은 유지한다.

## 실행 전 확인과 후속 기록

후속120은 local45 개발 실행에서 TF·시계 처리 노드와 scan/odom/action 처리 노드를 나누고, 기존 총64회·50ms 안에서 TF 처리에 먼저 절반을 배정한다. 한 차량의 실제 Jazzy 구성요소 시험에서는 부하 상태의 TF age가 공유 경로0.673초 이상에서 분리 경로0.031473초로 줄었고, 실제 TF 중단은 계속 거절했다. CPU22검사를 통과했으나 네 차량 통합 운행의 해결 근거로 확대하지 않는다. [120 수신](../coordination/collab/results/localization-critical-dispatch-120-integration.json)

124는 수정된 공개 wheel readback으로 실제 Carter·5.4kg carrier·자유 동적6kg tote의 정지 접촉을 통과했다. World0.600000031초·70샘플·0.4초 dwell과 종료 정리를 확인했다. 차체에 대한 carrier 위치 오차는0.0000015981m, tote rest 오차는0.0008744159m였다. 이동·도킹·공장 재고 연계와 제조사 적재 등급은 이 정지 시험의 범위 밖이다. [124 결과](evidence/local45-loaded-carrier-124.json)

123은 종료가 확인된041·038·039의 지정 원본 폴더만 NTFS 무손실 압축했다.1980개 색인 파일·11,049,331,379바이트를 기존 명세의 SHA와 대조했고 모두 같았다. 파일·경로·기존 압축본을 보존했다. 확인 시 C 여유 공간은8,675,164,160바이트, E는3,888,451,584바이트였으며 후속 실행 전 다시 확인한다. [123 보관 근거](evidence/local45-storage-123.json)

## FOT-085 — README GIF의 프레임 손실과 짧은 연속 촬영

기존 미리보기는10fps MP4를4fps GIF로 변환해 원본 프레임을 추가로 버렸다. 같은12초 발췌를10fps·120프레임 GIF로 바꾸고 원본 SHA와 변환 조건을 기록했다. 보간 프레임은 사용하지 않았다. 원본 자체의 희소 촬영 간격은 변환으로 복원할 수 없으므로 다음 normal3 개발 실행은 기존4대·관측용 경로에서 실제 이동 명령이 나온 뒤 한 번만5초·10fps를 촬영한다. CPU 관련4검사와 Python3.10 AST를 통과했다. 실제 촬영 결과가 나오기 전에는 새 연속 영상이 있다고 표시하지 않는다.

후속042의 별도5초 창은50회·3시점 촬영을 완료했다. 공개한 조감/차량 근접 두 시점은 물리6프레임·0.1초 간격이며 원속도10fps다. 창 종료 이벤트와 원본 SHA를 확인해 필요한100장만 C/E에 보존했다. 실행 중 bag SQL·복사는 하지 않았다. 긴 타임랩스 대신 이 연속 촬영을 대표 미디어로 사용한다. 사용자 추가 요청에 따라 다음 소스의 촬영 주기는 매 물리 프레임, 명목60fps로 바꿨다.4대 `_compile` 검사는 통과했으며 실행 중인042의10fps 동결 소스는 유지한다.60fps 실제 촬영은 미확인이다.2대 전용 `_validate_runtime`을4대 설정에 사용하는 보조 검사 오류는 제품 실패와 구분한다.

GIF 출력 목표는40fps, 원본 MP4는60fps다. [GIF89a 명세](https://www.w3.org/Graphics/GIF/spec-gif89a.txt)는 지연을10ms 단위로 정의한다. [현재 Chromium 구현](https://raw.githubusercontent.com/chromium/chromium/main/third_party/blink/renderer/platform/graphics/deferred_image_decoder.cc)은10ms 이하 프레임을100ms로 바꾸므로60fps GIF의10/20ms 조합은 피한다. 새 원본에서40fps GIF를 만들고 실제 지연이20/30ms·합계5초인지 검사한다.60fps 원본 수집과40fps 출력 성공은 아직 미확인이다.

## FOT-086 — SDK 준비 전 운행 제어 체크포인트 생성

116 운행 제어를 활성화하면 G3가 SDK 준비 이전에 `live-control-state.json`을 만들어 SDK bootstrap의 정확한 파일 집합과 충돌했다. 준비 대기 구간의 실제 파일 목록을 확인하는 회귀 검사로 이 추가 파일을 재현했다. 체크포인트와 명령 디렉터리 생성을 SDK ready 확인 이후로 옮겼다. 같은 검사1건이3.672초에 통과했고 기존 체크포인트 배타 생성 조건은 유지한다. 실제 SDK·ROS 통합 실행의 운행 제어 성공은 아직 미확인이다.

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

## FOT-092 — 대표 장면의 선정과 교차로 출구 공간 예약

조감과 별도 적재 시험은 물류 운행 중 양보·통과·재개를 보여주지 못했다. 대표 GIF를 같은 공장 안의4대 SDK 물리 시연으로 교체했다.139는 SDK Python3.10에서 `typing.Unpack`을 불러오다가 실패했다. 런타임에 필요하지 않은 타입 import를 `TYPE_CHECKING` 안으로 옮겼다.140은 새로 적은 잘못된 자산 URL로 참조 합성이 실패했고, articulation 초기화 오류가 뒤따랐다. 검증된 `local_asset_probe.ASSET_PATH` 상수를 재사용해 경로 중복을 없앴다. 두 실행의 소유 Job과 하위 프로세스 종료를 확인했다.

141에서는 북쪽 진입 차량이 먼저 교차로를 예약했지만 서쪽 출구를 다른 대기 차량이 막고 있었다. 양쪽의 거리 센서 정지가 World17초 이후 지속됐고, World25초 상태에서도 재개하지 못했다. 실패한 실행을 소유 Job runner 종료로 중단했으며 관측한8개 PID가 모두 사라졌음을 별도 기록했다. 정상 Job 최종 영수증이나 성공한 시연으로 분류하지 않는다.

`JunctionDemo`는 접근 단계에서 교차로·출구 공간을 함께 예약한다. 출구의 다른 차량 추정 위치와 기존 반경을 확인하고, 교차 구간에서 차체 전체가 이탈한 뒤 예약을 해제한다. 소스는 엔코더 적분과 작성된 초기 위치를 제어 입력으로 사용하며 GT 위치는 간격 평가와 미디어에만 쓴다. 관련3개 CPU 검사를 통과했다.

142는 실제 World56.650002955초·60fps3,399장으로4대 목적지 도착을 마쳤다. AMR2 양보3.966666874초 → AMR1 구간 이탈14.083334068초 → 같은 시각 AMR2 진입 허가 → 회전·복귀를 기록했다. 평가용 차체 중심 최소 간격은2.492473633m였고 소유 Job 종료와 정리 오류0건을 확인했다. 이 간격은 실제 산업 안전 인증을 뜻하지 않는다. 동적 우회 계획·ROS/Nav2·접촉 적재·MES 통합과 구분한다. GIF는 원본의 교차 구간을 고정 확대해2배속40fps로 내보냈으며 RGB 보간은 사용하지 않았다. [142 실행·보존 근거](evidence/local45-junction-hero-142.json)

## FOT-093 — 045의 시작 조건 시간 초과

045는 명시적 DDS peer와 공유 파일 일시 오류 재시도 수정 소스로 실행했지만, planner/controller/TF/odometry/scan 시작 조건을 통과하지 못했다. 최초 예외는 ROS의 startup `TimeoutError`이며 SDK의 mission 실패 보고는 후속 결과다. 명시적 peer 설정을 실제 공장 시작 문제의 해결로 판정하지 않는다.60fps 이동 촬영 창은 시작하지 않았다. 양 OS 트리와 Job을 종료한 후160개 파일·원본274,835,019바이트를 C/E의 operations-137 압축본177,902,613바이트로 보존하고 모든 구성 파일 해시를 대조했다. 실행 중 bag SQL 접근·복사는 하지 않았다. [045 결과](evidence/local45-transport-045.json)

143에서 종료된045의 최초 Nav2 오류를 좁혔다. `carter3.lifecycle_manager_fleet4`의 `planner_server/get_state` future가 정상 완료되지 않아 해당 stack의 bringup이 중단됐다. 마지막 관측에서 다른3개 stack의 planner/controller/bt가 active3이었고, carter3의 odometry1,495회와 front/rear scan 수신도 계속됐다. 전체 DDS 또는 센서 단절로 분류하지 않는다. Jazzy [service client](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_util/include/nav2_util/service_client.hpp)와 [lifecycle manager](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_lifecycle_manager/src/lifecycle_manager.cpp)를 대조했으며 정확한 CPU/DDS 원인은 미확정이다. 선택형 `FACTORY_OPS_NAV2_STARTUP_STAGGER_S`는 기존 기본값0을 보존하고 namespace별 전체 stack 시작을 최대5초 간격·마지막 최대15초까지 나눈다. timeout·생명주기 성공 조건·운영 TTL은 바꾸지 않는다. 작성자 관련5개 CPU 검사를 통과했고 실제 효과는 다음 실행에서 확인한다. [143 근거](../coordination/collab/results/factory-startup-143.json)

## FOT-094 — 전체 공장 흐름을 별도 교차로 시험으로 대체한 대표 미디어

142는 실제 양보·재개를 보여주지만 자재 공급부터 공정·출하·빈 용기 회수까지의 전체 운영을 보여주지 못했다. 사용자 최신 요구에 따라142는 보조 시험으로 내리고 전체 공장 시연을 우선한다. 다음 normal3 설정은 첫 이동5초 촬영을 끄고, 이미 적용된 원장과 빈 토트 상태에서 자재 이송·공정 처리·완제품 이송·첫 출하·빈 용기 회수·순환 완료를 골라 최대6개·각240프레임만 촬영한다. 관측된 단계만 선택하며 미완료 단계를 영상으로 채우지 않는다.

각 창은 원본 물리 프레임·적용된 상태 스냅샷·실행 ID와 연결한다. 화면용 웹 상태는 원래 임무 시계에 실행 시작점을 더해 World 시계로 대조하고,2초 이내 과거 관측만 보존한다. 누락·오래된 상태는 미관측으로 남기며 운동 권한에 사용하지 않는다. 선택기 관련3개 CPU 검사와 적용 스냅샷 해시·native lifecycle 연결1개 검사를 통과했다.1280×720 전체 공장 촬영과 이 설정의 실제6단계 운영 성공은 아직 미확인이다.

## FOT-095 — Humble·Jazzy 발견 메시지의 Gid 크기 차이

046은 시작 분산5초를 적용했지만 시작 조건에서 실패했다. Windows SDK stderr에 `ParticipantEntitiesInfo`의 Fast CDR/Bad alloc 역직렬화 오류가 반복됐고 WSL 관측 시계가 멈췄다. 실제 설치 소스의 `rmw_dds_common/Gid`는 Windows Humble24바이트, WSL Jazzy16바이트다. [Humble 메시지 정의](https://raw.githubusercontent.com/ros2/rmw_dds_common/humble/rmw_dds_common/msg/Gid.msg)와 [Jazzy 메시지 정의](https://raw.githubusercontent.com/ros2/rmw_dds_common/jazzy/rmw_dds_common/msg/Gid.msg)도 이 차이를 명시한다. 발견 스키마의 불일치는 확인됐지만 모든 과거 lifecycle 실패의 유일한 원인으로 확대하지 않는다.046은 운송0건·운영 촬영 창0개이며 두 OS 종료 후 C/E 원본 보존과 SHA 대조를 마쳤다. [046 결과](evidence/local45-transport-046.json)

146은 무료 DDSRouter3.5.1 컨테이너로 두 발견 영역을 나눈 CPU 시험이다. Linux→Windows Twist 전달은 있었으나 반대 방향 전달은0이었다. Windows에서 시계 publisher가2개여서 기존 단일 발행 조건도 만족하지 못했다. 추가 clock/static callback을 spin하지 않는 passive node에 붙인 시험 코드 오류가 있어 해당 callback0건을 전송 실패 근거로 삼지 않는다. 컨테이너·두 OS 소유 프로세스는 모두 종료했다. 이 후보를 실제 공장 해결로 표시하지 않는다.

## FOT-096 — OS 내부 DDS 격리와 허용 토픽의 응용 계층 전달

147은 각 OS의 DDS 인터페이스·초기 peer를127.0.0.1로 제한하고, 원본 CDR·실행 ID·순번·토픽 타입·SHA를 확인하는 TCP 경로로 센서와 시계만 Windows→Linux, 주행 명령만 Linux→Windows로 전달했다. 서비스·액션·발견 정보는 OS 경계를 넘기지 않는다. 주행 명령을 생성하거나 필드·시계를 바꾸지 않으며, 수신 시점의 관측 시계보다 미래이거나0.3sim 초보다 오래된 Linux 명령 봉투는 버린다. 봉투의 시계는 relay가 명령을 수신한 시각이며 원 생산자의 생성 시각으로 표현하지 않는다. 기존 watchdog·주행 권한·TTL은 유지한다.

비GPU CPU 시험은44Jazzy+5Humble fixture context와2relay context에서 통과했다. 네 차량 채널의 odometry·8개 scan, 정적 TF의 늦은 구독자 수신과 역방향 명령을 확인했다. 양 OS에서 clock publisher1개·역행/중복0건, 종료 정리 오류0건이었다. 실제 World·로봇 동작이나 전체 공장 완료의 증거로 쓰지 않는다. 원본3개 wire 검사와 loopback XML1검사를 통과했다. [147 결과·146/147 원본 보존](evidence/local45-topic-relay-147.json)

## FOT-097 — 운영 촬영의 적용 문서와 실제 RGB 프레임 결속

145 exporter 작성 중 visual snapshot이 `{schema_version,state_hash,visual_state}` wrapper인데 선택기가 wrapper 전체를 flat 원장으로 읽는 연결 오류를 발견했다. 다음 producer는 wrapper의 SHA·state hash·run·sequence를 확인하고 `visual_state`를 선택기에 전달한다. 또한 기존 G2는 적용 상태를 갱신한 뒤 물리를 한 단계 진행하고 RGB를 촬영한다. 촬영 태그를 pre-step에서 만들면 RGB와1물리 프레임 어긋나므로 선택기 관측을 실제 post-step 제어 샘플과 capture_due 경계로 옮겼다. 이 오류들은046의 최초 발견/시작 실패와 별도다.

145의 CPU exporter9검사·Python3.10 AST2검사를 수신했다. 실제 완료된 정상 실행·양 OS 정리·여섯 실제 단계·1280×720 원본과 연속60Hz 물리 프레임을 요구하며 누락 단계를 합성하지 않는다. 화이트 운영 콘솔의 재고·출하·반납·작업 값은 같은 촬영의 적용 원장과 freshness를 통과한 checkpoint에서 읽는다. 실제 미디어 렌더링은 후속 정상 실행 종료 뒤 수행한다.

## FOT-098 — 새 relay checkpoint에 공유 파일 처리 누락

047은 양 OS relay 연결과4채널 정적 TF·33개 시계 callback까지 진행한 뒤 Windows `windows-relay.tmp`→`windows-relay.json` 교체의 `WinError5`로 실패했다. Linux의 socket EOF는 그 뒤의 결과다. 새 relay의 checkpoint writer가 기존 `compat.replace_shared_file/write_shared_text`를 재사용하지 않고 단일 replace를 호출한 것이 확인됐다.0.2wall 초 한도의 기존 처리로 연결하고, relay 상태 reader와 launcher checkpoint도 같은 유틸리티를 재사용한다. 계속되는 접근 오류·ENOSPC는 실패하며 기록을 성공으로 숨기지 않는다. 관측된 일시 sharing conflict1회 뒤 실제 atomic 교체가 되는 fault injection1검사를 통과했다.

SDK·Nav2·Windows relay·Linux relay의 네 소유 트리는 모두 종료했고 잔여 PID와 정리 오류가 없었다. 운영 이동이나 여섯 단계 촬영은 완료되지 않았다. 원본은 보존하고 C의 전체 입력/결과/동결 소스 압축본과 E의 명시적으로 선별한 상태·초기 이미지·로그를 각각 해시 검증한다. E 선별본을 전체 rosbag 백업으로 표시하지 않는다. [047 오류·종료·보존](evidence/local45-transport-047.json)

## FOT-099 — 048의 같은 OS 안에서 발생한 Nav2 설정 응답 TIMEOUT

048은 공유 파일 수정 소스와 OS별 DDS 격리를 사용했다. Windows–WSL 센서·시계 전달이 시작 제한까지 이어졌고 발견 메시지 역직렬화 오류나 relay 실패가 기록되지 않았다. Nav2 세 stack은 active3으로 올라갔지만 carter2는 planner2/controller1/bt1에 머물렀다. 최초 관련 경고는 `carter2.planner_server/change_state` 응답을 전송하지 못했다는 `rmw_response.cpp:153`의 TIMEOUT이며 client가 응답을 받지 못한다는 내용이다. 모든 차량의 odometry와 앞/뒤 scan은 계속 도착했으며 carter2의 pose만 없었다. 따라서 전체 센서 단절이나 같은 발견 메시지 실패로 묶지 않는다.

기존300wall 초 시작 한도를 유지한 채 실패했고, 운영 이동과 여섯 단계 촬영 창은 시작되지 않았다. SDK·Nav2·두 relay의 네 소유 트리는 모두 종료했고 정리 오류0건을 확인했다. C 전체 입력/결과/동결 소스 압축본225,081,090바이트와 E 선별 상태/미디어3,859,466바이트의 구성 파일 해시를 확인했다. 설치된 Jazzy rmw-fastrtps-cpp8.4.4·FastDDS2.14.7의 서비스 응답 구현을150에서 확인하며, 발행 모드 환경변수가 서비스에도 적용되는지 확인하기 전에는 해결책으로 표시하지 않는다. [048 관측·오류·종료·보존](evidence/local45-transport-048.json)

## FOT-100 — 서비스 응답 reader 매칭의 유한 대기 후보

150은 공식 rmw-fastrtps8.4.4에서 해당 오류가 DDS write 전의 `check_for_subscription` 분기임을 확인했다. 응답 writer QoS의 `reliability.max_blocking_time` 동안 관련 client response-reader GUID 매칭을 확인하지 못하면 MAYBE→TIMEOUT으로 반환한다. publication mode 환경변수는 서비스 writer에도 적용되지만 ASYNCHRONOUS가 이 사전 매칭 대기를 우회하지 않는다. 내부 listener 지연·GUID·discovery 중 어느 원인인지는 미확정이다. [150 공식 근거와 한계](../coordination/collab/results/native-service-response-150.json)

151의 비GPU C++ probe는 실제 설치 FastDDS 라이브러리를 사용해 기존 XML의 기본 writer 대기0초+100,000,000ns와 다음 XML의 정확한 response profile1초+0ns·RELIABLE을 읽었다. Publisher/Participant를 삭제한 뒤 두 probe와 컴파일러가 종료됐다. 이는 XML 적용 값 확인이며 ROS 서비스나 공장 성공 시험이 아니다. [151 실측·소스·보존](evidence/local45-service-qos-151.json) · [실측 probe](../scripts/ros/probe_service_reply_qos.cpp)

다음 실행의 선택형 `--startup-service-match-wait`는 Linux loopback DDS의4대×8구성요소×change_state/get_state 응답64개 profile만 지정한다. Windows·일반 topic writer·발행 모드·메모리 정책을 바꾸지 않으며 XML QoS 전면 활성화도 하지 않는다. 기본 경로는 유지한다. Linux 이외 적용 거절·64개 정확한 응답 이름·RELIABLE·1초 값의 관련1검사를 통과했다. 기존300wall 초 시작 한도와 주행·센서·권한·TTL·반경·임무 기한을 유지하고, 실제 lifecycle 활성화와 완주는 새 동결 실행에서 확인한다.

## FOT-101 — 실패 실행·압축 해제본의 중복 누적으로 저장 공간 부족

D는 약1.05GB, E는 약2.20GB의 여유 공간만 남았다. 프로젝트 진단 경로에 실패 실행 원본·전체 압축본·복원본·코덱 비교·검수용 bag 복사본이 중복 보관돼 있었다. 사용자 지시에 따라 이 프로젝트의 명시된 경로만 정리했다. 삭제 전 절대 경로와 보호 경로를 대조하고 reparse 경로를 거절했으며, 실패 runtime은 양 OS 종료가 확인된 것만 대상으로 삼았다.

최종 G4 데이터 압축본 두 개의 SHA-256이 기존 검증 출처의 `e8f683730c21b4aa7a159f30e956b25a79b26ca699afd36d1b88ff2bb7240749`와 일치함을 확인했다. G2 원본 압축본도 기존 해시와 대조한 뒤 중복 미리보기를 지웠다. 전체 공장036 원본과 D의077 보존본, 실제 적재·접촉·회전 시험, 대표 미디어·공개 라벨 검사, 최신049와 재사용 C++ 설치물112, 설치 SDK는 남겼다. 실패 원인·종료·소스의 작은 영수증을 로컬에 보관하고316개 대상의 삭제·보존 결과를 기록했다.

세 번의 정리 구간 전후 여유 공간 관측 합계는 약233GB 증가했다. 논리 파일 크기와 실제 확보량은 압축·할당·동시 시스템 쓰기로 다르다. Git에는 코드·선별 미디어·출처·오류 기록을 남기고 대형 원본은 선별 보존한다. 과거 출처 문서의 삭제된 로컬 경로는 현재 색인에서 만료로 명시하며, 실패 실행의 전체 원본을 여러 드라이브에 계속 쌓지 않는다. [정리 결과·보존 해시·만료 경로](evidence/local45-storage-155.json)

## FOT-102 — 049의 중계 상태 파일 교체가 기존 재시도 한도를 초과

049에서는 네 차량의 Nav2와 신선한 센서·추정 위치가 준비됐고, 자재 재고가60에서56으로 줄었다. 자재 공급의240프레임 촬영 창은 끝났으며 공정 처리 창을 촬영하던 World208.07초 부근에서 실행이 중단됐다. 최초 기록 오류는 Windows 중계의 `windows-relay.tmp`를 `windows-relay.json`으로 교체하던 `PermissionError / WinError 5`다. FOT-098의 기존0.2wall 초 재시도가 적용된 상태에서도 실패했으므로, 한 번의 일시 충돌 검사 통과가 장시간 실행의 해결을 증명하지 못했다.

Linux 중계의 소켓 종료와 임무의 `ExternalShutdownException`은 그 뒤에 기록된 후속 오류다. 공유 충돌을 만든 reader 또는 OS 구성요소는 아직 확인되지 않았으며, 디스크 부족을 뜻하는 ENOSPC 기록은 없다.049는 저장 공간 정리의 보호 대상으로 유지했다. SDK·Nav2·양 중계의 소유 트리 네 개는 종료됐고 잔여 소유 PID와 정리 오류가 없다. 전체 업무나 여섯 단계 대표 영상은 완료로 세지 않는다.

이 오류는 열린 상태로 남긴다. 충돌 핸들과 상태 파일 저장 경로를 확인해 수정한 뒤 후속 실행에 적용하며, 같은 상태로 GPU 전체 실행과 대형 데이터 보관을 반복하지 않는다. [049의 실패·부분 촬영·종료 근거](evidence/local45-transport-049.json)

## FOT-103 — 중계의 주기 관측을 파일 교체에서 추가 기록으로 변경

Microsoft의 [CreateFileW 공유 규칙](https://learn.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilew)과 [MoveFileExW](https://learn.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-movefileexw)를 대조했다. 이 PC의 실제 Windows 검사에서 일반 reader가 열린 대상의 `os.replace`가 WinError5로 실패했다. 삭제 공유를 허용하는 reader 후보도 같은 조건에서 실패해 채택하지 않았다.049의 충돌 핸들 소유자는 여전히 미확정이다.

중계는 시작·연결·준비·실패·종료 시 전체 상태를 저장하고, 매초 관측은 행당 최대4096바이트의 작은 JSONL heartbeat에 추가한다. 통신 루프가 주기적으로 같은 전체 JSON을 교체하지 않는다. append는 재시도하지 않고, 부분 쓰기·ENOSPC는 실패로 남긴다. reader는 한정된 끝부분에서 줄바꿈이 끝난 행만 읽고, 완료된 행의 손상·다른 실행 ID·side·상태·역행 sequence는 거절한다. 시계·명령 본문·0.3초 획득 시각 제한·센서/추정 위치 검사·소유 프로세스와 실행 한도는 유지했다.

관련10검사와 설치 SDK Python3.10.15의2파일 구문 확인을 통과했다. CPU 실제 양 OS fixture156에서는 Windows 전체 상태 reader를 샘플링 내내 연 채44Jazzy+5Humble fixture context와 중계를 실행했고, 센서·시계·정적 TF·역방향 명령 전달이 통과했다.24.484wall 초 뒤 양 OS와 중계가 종료됐으며 정리 오류0건이다. 새 heartbeat는 Windows20회·Linux19회 기록됐고 stale drop은0건이었다. GPU·World·로봇 구동은 하지 않았다. 긴 공장 정상 완료의 증거로 확대하지 않고 다음 동결 실행에서 확인한다. [156 실측·소스 해시·공식 근거](evidence/local45-relay-heartbeat-156.json)

## FOT-104 — 인증 전 heartbeat와 미시작 자원 정리의 구분

050은 중계 연결 중인 heartbeat를 supervisor가 `ready`로 검사해 실패했다. Linux 첫 행은 `connecting`, sequence1, 시계0이었으며 예외는 `relay.start` 안에서 발생했다. 주 임무의 Linux 시작과 SDK Popen 이전이므로 World·GPU 공장·새 촬영은 시작되지 않았다. 두 중계는 정상 종료됐다. 원래 호스트 영수증에는 아직 만들지 않은 Linux unit의 정리 거절과 없는 SDK job 영수증이 정리 오류로 남았다. 원본 영수증은 수정하지 않는다.

주기 관측은 `starting/connecting` 동안 파일·sequence를 만들지 않고, 인증된 `ready`에서 첫 행을 게시하도록 고쳤다. 실행 ID·side·상태·sequence 거절 조건은 유지한다. 주 실행기는 OS별 시작 시도 여부를 기록해 미시작과 실제 종료를 구분하며, 시작을 시도한 자원의 기존 소유권·종료 증명은 계속 요구한다. 관련4검사와 SDK Python3.10.15의2파일 구문 확인을 통과했다. 이전156의 연결 완료 뒤 관측·전달 결과를 초기 연결 분기의 증거로 확대하지 않는다. [050 관측·수정 범위·검사](evidence/local45-relay-startup-157.json)

## FOT-105 — 중계 자원 종료와 통신 성공의 구분

내보내기 경로에서 `RelayPair.status=closed`가 자원 회수를 뜻하며 각 중계의 통신 성공을 보장하지 않는 조건을 확인했다. 실제051에서 새 실패가 발생했다는 뜻은 아니다. 중계 사용 실행의 미디어를 승인할 때 양쪽 endpoint의 동일 실행 ID·side·`stopped`·`closed=true`와 오류/정리 오류 없음을 추가로 요구한다. 종료되지 않았거나 실패한 endpoint를 가진 합성 영수증은 정상 미디어로 내보내지 않는다. 새 경계 검사와 기존 OS 소유 종료 검사2개를 통과했다. 실행 중인051의 동결 소스·임무·촬영 설정은 바꾸지 않았으며, 원본 종료 후 CPU 내보내기에 적용한다.

## FOT-106 — 전체 공장 영상에 쓰지 않는 추적 RGB 제품의 수집

051은 대표 영상에 overview만 사용하는 동안에도3개의 persistent RGB render product를 유지하고 매 촬영에3개 PNG를 저장한다. 촬영 중 World 진행이 느려지는 것을 관측했으나3시점 렌더만을 전체 지연의 원인으로 확정하지 않는다. 실행 중인 동결 소스·한도·촬영 설정은 바꾸지 않는다.

설치된 Isaac Sim4.5의 Replicator1.11.35와 [공식 API](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.11.35/source/extensions/omni.replicator.core/docs/API.html), [4.5 수명 관리 예제](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/tutorial_replicator_ur10_palletizing.html)를 대조하고 다음 실행용 `--capture-views overview`를 추가했다. 쓰지 않는2개 render product/annotator를 생성하지 않으며 매 프레임 Hydra 전환을 도입하지 않는다. 생성한 제품만 기존 소유자가 detach/destroy한다. 선택값·실제 소스 수·보정된 카메라 정보를 남기고 기존3시점 정식 검증과 구분한다. CPU9검사와 SDK Python3.10.15의5파일 구문 확인을 통과했다. GPU 실행과 RTF·wall time 개선은 미실측이며 다음 실행에서 확인한다.

## FOT-107 — 운반 차량과 다른 차량의 움직임으로 촬영 시작

기존 선택기는 `inbound_loaded/outbound_loaded/returning` 상태와 어느 차량이든 움직이는 조건을 함께 사용했다. 따라서 운반 차량은 기다리고 다른 차량만 움직일 때 운송 촬영이 시작될 수 있었다. 코드의 조건 오류는 확인했으며051의 각 촬영분이 실제로 부적합한지는 종료 후 원본 움직임으로 판정한다. 선속도0만으로 정지라 단정하지 않고 적재 상태의 제자리 회전도 포함한다.

다음 실행의 선택기는 운반 차량 ID와 같은 차량의 신선한 주행 허가·입력 명령·엔코더 이동/회전을 요구한다. 내보내기도 선택한 원본 구간에서 해당 화물·빈 토트의 운반 차량 GT만 평가하고, 다른 차량의 움직임으로 대체하지 않는다. 미디어용 작성 기준은 이동0.1m 또는 회전0.15rad이며 검증 근거에 측정값과 함께 기록한다. GT를 제어에 사용하거나 실패한 원본을 수정하지 않는다. 관련8검사와 SDK Python3.10.15의3파일 구문 확인을 통과했다. 네이티브 운송 완료와 대표 미디어 적합성은 별도로 보고한다.

051의 JSON 관측에서 출하 후 자재 공급 차량과 빈 토트 반납 차량의 동시 주행을 확인했다. 새 실행의 `first_shipment` 창은 첫 출하 집계 직후가 아닌 두 차량의 실제 주행 시점에 선택하고, 원본 GT에서도 두 차량의 움직임을 확인한다. 같은6개 창·1,440프레임 한도를 유지한다. 선택기·기존6단계·두 차량 원본 움직임의 관련3검사를 통과했다. 현재051의 동결 소스나 과거 촬영의 의미는 바꾸지 않는다.

## FOT-108 — 전체 운송 미완료와 Windows 중계 Job 시간 초과

051은 출하2/3건·빈 토트 반납1/3건·재고54개까지 진행했고, 운영 촬영 여섯 창 중 다섯 창을 마쳤다. `relay-job.json`의 `timed_out=true`, `elapsed_wall_s=10800.047`에서 Windows 중계 Job이 설정된 시간 제한에 도달했음을 확인했다. 호스트에는 `owned Windows topic relay exited`가 기록됐고, Linux의 peer reset과 종료 중 ROS context 오류가 뒤따랐다. 공유 파일 교체의 `WinError 5` 재발로 분류하지 않는다. 전체 실행 시간의 GPU·CPU·파일 I/O별 기여도는 측정하지 않았다.

SDK Windows Job의 남은 PID는 없고 Nav2 Linux cgroup은 비었으며, 양쪽 중계 소유 트리도 종료됐다. 정리 오류는 없다. Windows 중계의 최종 상태 파일은 `ready/closed=false`, Linux 중계는 `failed/closed=true`다. 자원 종료를 통신 정상 종료나 업무 완주로 표시하지 않는다. 원본은 현재 실행 디스크에 한 벌 보존하고 실패 실행 전체의 추가 압축·복제를 만들지 않았다. [종료·미완료 근거](evidence/local45-transport-051.json)

053은 새 실행 ID·동결 소스로 조감 카메라의 렌더 제품 하나만 생성한다.4대·3주문·운영 원장·제어/센서 계약·최대 촬영 표본 수와10,800초 한도는 유지한다. 운반 차량의 관측된 움직임과 출하 이후 병행 운송을 촬영 조건에 사용하고, 같은 실행에서 opt-in 정지·재개 결과를 수집한다.051의 소스와 제한 시간은 실행 도중 바꾸지 않았으며,053의 속도 개선·완주·정상 중계 종료는 실제 결과 전까지 미확인이다.051은 실패 판정을 유지하고 대표 전체 공장 영상으로 내보내지 않는다.

후속 실행기의 작업 기한은 `wall_bound_s-30`으로 두고 마지막30초를 기존 SDK·Nav2·중계 정리에 예약한다. 기한 검사를 중계 상태 확인보다 먼저 수행해 자식 Job 시간 초과가 상위 예산 종료 이유를 가리지 않도록 했다. 자식 Job의 hard timeout과 전체 한도는 늘리지 않는다. 최대10,800초에서10,770초의 작업 종료를 요구하는 검사는 수정 전 실패했고, 수정 후 기존 시작·종료·최초 예외 검사와 함께10건을 통과했다. SDK Python3.10 구문을 확인했으며 실제 기한 도달 시 정상 중계 종료는 다음 실행에서 확인한다. 진행 중인054의 동결 실행기는 변경하지 않는다.

## FOT-109 — 예약 거절 후 대기 중인 목표가 현재 점유의 목표를 덮어씀

053의 KitClient는 같은 실행의 배정된 작업에서 실제 엔코더 이동을 관측한 뒤 정지 요청 한 건을 보냈다. 차량 세 대는 정지 확인을 충족했지만, Carter3는 초기 예약 세대3을 유지한 채 `retained_hold_binding_or_tiles_mismatch`에 머물렀다. 초기 HOLD와 현재 런타임의 목표 UUID가 달랐다. `request_remaining_reservation`이 후속 예약의 승인 전에 `follow_uuid`를 바꾸고, 거절돼도 기존 점유는 유지하는 코드에서 이 불일치를 재현했다.

새 소스는 후보를 `pending_follow_uuid`에 보관하고 `manager.transfer`가 승인한 경우에만 현재 목표에 반영한다. 거절·재요청 동안 현재 점유의 목표·세대·타일과 후보 ID를 각각 유지한다. 회귀 검사는 수정 전 목표 불일치로 실패했으며, 수정 후 거절·재요청·차량 양보 후 승인과 대기 원인 표시·엄격한 정지 조건의 관련12검사를 통과했다. 설치된 SDK Python3.10.15에서 변경 파일의 구문도 확인했다. ACK·오도메트리·위치·시계·예약 검사를 완화하지 않았다.

053은 원본 수정이나 강제 재개 없이 실행 ID가 일치하는 SDK 중단 요청으로 종료했다. pending 요청은 종료 시 거절됐고 재개 요청은 보내지 않았다. SDK·Nav2·양 OS 중계 트리를 닫았으며 양 중계 endpoint 모두 `stopped/closed=true`, 정리 오류0건이다. 실패 원본과 [정지 진단·종료 근거](evidence/local45-native-control-053.json)를 보존하며, 수정한 소스의 실제 정지·재개 성공은 새 동결 실행에서 확인한다. 전체 공장 대표 영상은 생성하지 않았다.

## FOT-110 — 조작 요청의 시간 유효성 거절과 진단 정보 부족

수정 소스의054에서 두 개의 서로 다른 정지 요청이 `request_stale_or_invalid_time`으로 거절됐다. 첫 요청의 거절·running·revision0을 확인한 뒤 새 요청 한 건만 보냈고, 두 번째 거절 후 추가 요청은 보내지 않았다. pending/적용 상태를 모르는 명령을 재전송한 것이 아니다. 네이티브 물류 실행은 running을 유지하며 정지·재개 성공으로 표시하지 않는다.

기존 거절 이유만으로는 요청이3초보다 오래됐는지, 수신기의 현재 시각보다 미래였는지, 시각 파싱이 실패했는지 구분할 수 없다. 별도 읽기 전용 표본에서 Windows 전후 시각1792601080743/1792601080925ms 사이의 Linux 시각은1792601080558ms였다. Linux가 Windows 중간값보다276ms 느렸지만 이 표본은 실제 요청 처리 순간의 age가 아니므로 거절 원인으로 확정하지 않는다. OS 시각·요청 타임스탬프·0~3초 허용 구간을 바꾸지 않았다.

다음 소스의 시간 거절 진단에는 요청 ID·requested_at·수신기 wall 시각·유한한 request_age_s를 남긴다. 음수 age·4초 경과·파싱 실패가 계속 거절되고 상태/revision이 바뀌지 않는 검사와 기존 요청 거절 검사를 통과했다. 이 진단은 진행 중인054에 소급 적용하지 않는다. 앞선 두 거절의 실제 age는 미확인으로 남긴다.

054의 조작 API를 ROS 수신기와 같은 WSL에서 실행하고 웹의 `OPS_API_URL`을 이 API로 연결했다. 큐 생성과 수신이 같은 OS wall 시계를 사용하며, 기존 Windows API·웹 소유 프로세스는 닫고 새 API·웹을 짝으로 종료하도록 소유자를 연결했다. 네이티브 소스·OS 시각·시간 허용 구간은 바꾸지 않았다. 이 구성에서 새 정지와 재개가 모두 `applied`됐으며 [동일 실행의 기록](evidence/local45-native-control-054.json)을 남겼다. 이는 앞선 거절의 원인을 소급 확정한 결과가 아니다.

## FOT-111 — 초기 미리보기와 실제 조감의 차이, 공장 경계 잘림

`factory-overview.png`는 `configure_cameras` 이전에 생성하는 초기 미리보기다. 실제 운영 영상은 카메라 override 후 `media/overview-*.png`에 저장한다. 초기 미리보기의 큰 회색 여백을 실제 운영 영상의 프레이밍으로 판단하지 않는다.054의 실제 운영 원본과 카메라 readback을 확인했으며, 기존24mm 조감에서는 공장의 남쪽·옆 경계 일부가 프레임 밖으로 나갔다.

후속 fleet4와 normal3 profile의 overview focal length만17mm로 조정했다. 시점·대상점·1280×720 해상도는 유지하고 이미지 crop·AI 보완·GT 제어는 추가하지 않는다. 실제 readback의 변환 행렬·aperture로 외벽을 포함한 보수적인 외곽 상자의8꼭짓점을 투영했다.24mm는4꼭짓점이 프레임 밖이었고,17mm는 모두 들어오며 최소24.09픽셀 여유를 확보했다. 이는 CPU 광학 계산이며 새 설정의 GPU 렌더 완료를 뜻하지 않는다. [계산·설정 해시](evidence/local45-overview-framing-168.json)

진행 중인054는 기존 동결 camera/profile을 유지한다. 기존 영상의 calibration·픽셀 좌표·프레임 원본을 수정하지 않고, 새 프레이밍의 실제 카메라 readback·전체 공장 가시성은 다음 실행에서 확인한 뒤 README 이미지를 교체한다.

## FOT-112 — 정지 적용 기록과 현재 관측 신선도의 구분

054에서 Linux API를 통한 정지는 네 차량의 예약 지점·새 HOLD 응답·요청 이후 연속 정지 표본을 모아 적용됐다. 첫 조작 도구는 이후 관측의 일시적인 신선도 부족을 즉시 실패로 처리해 재개를 보내지 않았다. 정지 적용 자체가 실패한 것은 아니며, 실행은 정지 상태와 기존 예약을 유지했다.

도구가 적용된 정지 요청 ID를 보존하고 현재 정지 관측이 다시 신선해질 때까지 기다리도록 수정했다. 복구 모드는 같은 실행의 이미 적용된 정지 요청만 받으며, 현재 네 차량의 정지·capability·revision을 확인한 뒤 새 재개 한 건만 보낸다. 다른 실행·알 수 없는 요청·미적용 정지는 거절하고 정지 요청을 반복하지 않는다. 시간 허용 구간·HOLD·예약 조건은 완화하지 않았다. 정상 흐름·신선도 회복·재개 복구·잘못된 요청 거절을 포함한 CPU8검사를 통과했다.

실제054에서는 World296.367초에 정지를 접수하고307.717초에 네 차량의 정지를 적용했다. 같은 정지 ID로 관측을 다시 확보한 뒤362.050초에 재개를 적용했고 업무 운송이 이어졌다. [원천 이벤트 순번·행 해시·차량별 정지 근거](evidence/local45-native-control-054.json)를 보존한다. Kit와 같은 클라이언트의 네이티브 API 결과이며, 브라우저/Kit 패널 클릭이나 전체 운송 완료의 근거로 확대하지 않는다.


## FOT-113 — 충전 단계가 같으면 관측 파일과 SOC가 갱신되지 않음

054의 충전 보조 파일은 관측 시각491.300초를 유지하는 동안 주 원장과 차량의 SOC·위치는 계속 갱신됐다. 파일의 갱신 시각도02:31에 머문 상태를02:49의 새 원장과 함께 관찰했다. 원인은 충전 단계·소유자만 포함한 fingerprint가 바뀔 때에만 파일을 발행한 조건이다. 단계가 같아도 SOC와 관측 시각은 바뀌므로 파일의 신선도를 유지할 수 없었다.

후속 소스는 단계 변경 시 즉시, 같은 단계에서는1시뮬레이션 초 간격으로 새 관측 파일을 발행한다. 단계 변경 저널은 기존처럼 변화가 있을 때만 기록해 반복 데이터를 늘리지 않는다. 실제 임시 파일을 사용하는 CPU 검사에서 같은 단계의 SOC·시각 갱신과 즉시 단계 변경, 중복 lifecycle 사건이 없는 결과를 확인했다. 실행 중054는 수정하지 않았다. 이후054가 충전 단계로 전환되며714.100초의 새 보조 파일을 발행한 사실도 [순차 관측 기록](evidence/local45-charging-diagnostic-178.json)에 보존한다. 새 코드의 실제 SDK 확인은 남아 있다.

## FOT-114 — 큐에 있는 오도메트리보다 최신 위치만 사용해 접촉 대기를 초기화

충전 경계는 현재 tick의 최신 추정 위치만 큐의 모든 오도메트리와 결합했다. 이미 관측한 위치가 충분히 신선하고 오도메트리보다 과거여도, 더 최신 위치로 덮어쓰면 그 이전 오도메트리에 위치를 제공하지 못했다. 지연된 엔코더와 새 위치가 매 tick 들어오는 CPU 재현에서는1초 접촉 유지가 누적되지 않고 contact_pending에 머물렀다. 원래 시각을 보존하는 재현2검사가 수정 전에 실패했다.

후속 소스는 차량별 최근 추정 위치를 최대64개 보존하고 기존 신선도 한도 밖의 표본을 버린다. 각 오도메트리에는 그보다 미래가 아닌 실제 관측 위치 중 가장 최신 표본만 사용한다. 위치 보간·시각 변경·GT 입력·접촉 허용치 완화는 추가하지 않는다. 기존 미래 패킷 대기·미래/오래된 위치 거절·ID와 시각 보존을 포함한6개 통합 검사와 위 발행 검사1개, SDK Python3.10 구문 확인을 통과했다.

054의 충전 대기 시점에 추정 위치가 작성된 정렬 범위 안에 있었고 SOC는 낮아졌지만, 당시 모든 큐 표본을 보존한 자료는 없으므로 이 결함이054 대기의 유일한 원인이라고 확정하지 않는다.054는 기존 소스로714초 부근에 충전을 시작했다. 새4대 캐리어 소스로 준비만 한055는 이 수정 전 버전이므로 미기동 폐기 표시를 남기고 후속 실행에 사용하지 않는다. 현재054의 운송은 계속 유지한다.


## FOT-115 — 내보내지 않는 장면 뒤60프레임을 반복 수집

운영 촬영기는 각 창에서240장을 수집하지만 대표 미디어는 첫180장만 선택했다. 동일한 원속도3초를 만들면서 여섯 창의 뒤60장씩 총360장을 더 렌더·저장하는 조건이었다. 후속 normal3 프로필을180프레임으로 줄이고 선택기·내보내기가 동결 프로필의 창 길이를 사용하도록 변경했다. 최대 계획 수집량은1,440→1,080장으로25% 줄지만 GPU 시간·바이트 절감의 실측값으로 표현하지 않는다.

CPU 검사4개에서 기존240프레임 입력의180장 선택, 새180프레임 입력의 전체 선택, 여섯 단계1,080장과181번째 초과 프레임 거절을 확인했다. SDK Python3.10의2파일 구문 확인도 통과했다. 원본60Hz·해상도·재생 속도·GIF 지연과 실제 운반/병행 이동 조건은 바꾸지 않았다. 현재054의 소스·설정·촬영 원본에는 변경을 소급 적용하지 않는다.


054는 이후에도 충전 대기 중 SOC가 World714초의83.93%에서839초 부근83.74%로 감소했다. 현재 큐의 정확한 원인을 확정하거나 허용치를 바꾸지 않고, 유용한4대 정지·재개 및 출하1/3건 기록을 보존한 뒤 실행 ID가 일치하는 SDK 감독 중단 요청으로 종료했다. 원본·설정·10,800초 상한은 수정하지 않았다. SDK Job·Nav2 cgroup·양 중계·API/웹을 모두 닫았고 두 중계 endpoint는 stopped/closed, 정리 오류0건이다. 미완료 실행의 대표 영상 내보내기는 실행하지 않았다. [054 종료 근거](evidence/local45-transport-054.json)

후속056은 충전 시각 결속 수정·4대 물리 캐리어 옵션·17mm 조감·창당180프레임·종료 정리 예산을 동결 소스에 포함해 시작했다. 준비만 한055와 과거 자료를 성공 결과로 바꾸지 않으며,056의 실제 장착·주행·충전 및 전체 완료 판정은 결과가 나올 때 갱신한다.


## FOT-116 — 모듈의 행렬 변환만 작성해 PhysX/Hydra의 동적 변환 속성이 없음

056은 네 실제 Carter의 장착 허용과5.4kg 모듈·고정 조인트 속성 readback,17mm 조감 적용을 통과했다. 이후 첫 물리/렌더 단계부터 carrier의 `xformOp:translate/orient`를 찾지 못했다는 경고가 반복됐다. 원래 단일 차량 시험은 `SingleRigidPrim` 생성자가 변환을 정규화했지만,4대 어댑터는 이 객체를 만들지 않고 `AddTransformOp`만 작성했다. 따라서 단일 차량 시험에서 보완됐던 정규화가 누락됐다.

설치 SDK4.5의 XFormPrim `_reset_xform_properties`와 같은 double precision의 translate→orient→scale을 작성하고 초기 world 행렬과 op 순서를 readback으로 대조하도록 수정했다. 차체 위치·모듈 형상·질량·관성·조인트·충돌 정책은 유지한다. SDK3.10 구문 확인은 통과했으며 새 소스의 물리/렌더 단계에서 반복 경고가 사라지는지 확인한다.056은 초기 단계에 감독 종료했고 모든 소유 트리를 정리했다. [실제 장착·카메라 및 종료 근거](evidence/local45-carrier-startup-056.json)

057에서 네 모듈의 translate→orient→scale readback과 World68.07초까지의 물리 기동을 확인했다. 닫힌 SDK stdout/stderr 전체에서 이전 변환 경고는0건이다. 네 차량 모두 오도메트리1,307개·각 scan 채널645개와 추정 위치를 받았다. 다만 아래 Nav2 기동 오류로 운송은 시작되지 않았다. 정상 기록 종료 검사는 통과하지 않았고, 감독자의 SDK Job·Nav2·양 중계 종료는 확인했다. [057 기동 근거](evidence/local45-startup-057.json)

## FOT-117 — lifecycle 요청 writer가 기존 응답 전용 XML의 범위 밖에서 실패

057의 최초 실패는 `carter2/controller_server/get_state service client: async_send_request failed`다. Controller는 activate까지 진행했지만 AMR2의 bt_navigator는 inactive에 남았다. 센서 미수신이나 렌더 경고를 원인으로 재사용하지 않는다. 기존 `--startup-service-match-wait`는 이름과 달리 service readiness를 기다리지 않고 Linux XML의 응답 writer64개에만1초 차단 예산을 작성했다. 요청 writer profile은0개였다.

설치된 rmw_fastrtps8.4.4의 [client 생성 소스](https://raw.githubusercontent.com/ros2/rmw_fastrtps/54d2240637f18e3ec710bb73c51c79abaf91ece4/rmw_fastrtps_cpp/src/rmw_client.cpp)와 [요청 전송 소스](https://raw.githubusercontent.com/ros2/rmw_fastrtps/54d2240637f18e3ec710bb73c51c79abaf91ece4/rmw_fastrtps_shared_cpp/src/rmw_request.cpp)를 대조했다. 요청은 `rq/...Request`, 응답은 `rr/...Reply` writer를 사용하고 서로 다른 profile을 선택한다. 따라서 기존 응답 설정은 실패한 요청 writer에 적용되지 않는다. 다만 실제 writer QoS·match·history와 write 실패 사유가 없어 기본 차단 예산의 만료를 확정 원인으로 적지 않는다.

후속 소스에서는 기존64개 응답 profile을 보존하고 같은4대·8개 component·2개 service의 정확한 요청 profile64개를 추가한다. 범용 client profile, QoS 환경변수, 통신 방식, stagger, 임무 기동 제한은 바꾸지 않는다. 기존1초 writer 예산을 양방향에 작성하며 CLI 설명에 readiness wait이 아님을 명시했다. 관련2검사에서128개 정확한 이름·RELIABLE·1초 값과 기본 profile/환경의 보존을 확인했다. 실제 기동 성공은 후속 실행 결과로 판정한다. 재발하면 상한 증대나 같은 조건 재실행보다 요청 writer의 실제 QoS·match·write 실패 진단을 우선한다.

058은 네 차량의 planner/controller/bt_navigator가 모두 active에 도달했고 실제 운송을 시작했다. AMR1의 CH1 도착은 World45.93초, 기하 정렬에 따른 가상 충전 진입은47.00초, 충전 예약 해제는97.55초다. 작성된 복귀 목표는SOC86%이며, 보존한 관측은62.00초85.187%와 해제 후128.50초85.936%다. 순간 최대 SOC 표본은 별도 보존하지 않았으므로86%를 실측 peak로 표기하지 않는다. 예약 해제는 차체의 통로 점유 해제가 아니며 이후 실제 출발까지 점유를 유지한다. FOT113/114의 수정 소스로 충전 증가·해제와 같은 실행의4대 웹 관측을 확인했지만, 이 시점에는 운송 완주·정상 기록 종료가 남아 있다. [058 실행 중 관측](evidence/local45-charging-058.json)


## FOT-118 — 충전 관측 큐가 콜백 드레인 중64개 상한을 넘음

058의 닫힌 nav.log 최초 원인은 `virtual charging odometry queue overflow`다. 센서/위치 오류나 이전 서비스 기동 오류가 아니다. 드레인 종료 뒤에만 소비하던 충전 오도메트리 큐에 콜백 표본이 누적됐다. 콜백에서 현재 ROS 시각 이하 표본을 순서대로 소비하도록 수정했다. 상한64, 원래 시각과 미래 표본 보존, 접촉·정지·TTL은 유지한다. 처리할 수 없는 미래 표본만64개 쌓이면 계속 명시 오류로 종료한다. 관련9검사에서96개 연속 표본의 순서/단일 소비와 미래64개 보존·다음 표본 오류를 확인했다.058은 출하1/3·반납1/3·다섯 촬영 창에서 종료됐고 모든 소유 트리·중계·웹을 정리했다. 새 수정의 실제 주행 확인은 남아 있다.

## FOT-119 — 세 주문과 매번 주차하는 시나리오를 반복 공장 운영으로 사용

기존 normal3는0/10/20초의 세 주문만 선언하고 매 DROP 뒤 초기 주차장으로 이동했다. 공정 입구2자리·공정1개·긴 구간 예약도 병행 동작을 제한했다. GIF의 재생이나 차량 수 변경만으로 지속 운영이 되지 않는다. 새 반도체 설정은 유한 근무 시연의12개 lot를 순차 선언하고 하차 후 다음 일감을 먼저 확인한다. 배차되지 않으면2초 뒤 작업장을 비운다. 같은 위치 재인수는 별도 stationary receipt가 없으므로 제외하며 성공 Nav2 결과를 만들지 않는다. CPU 이동/정차 모델에서 네 대 반복 참여·동시4임무·12출하/반납·재고 보존을 확인했다. 이 검사는 실제 Nav2·예약 혼합 운송의 성공을 의미하지 않는다. 새 제조 구역/기능/제약은 [반도체 구역 문서](semiconductor-cell.md)에 기록했다.

## FOT-120 — SDK4.5 RGB annotator 해제에 HydraTexture 목록을 전달

정지 렌더186은 조감 PNG를 만든 뒤 `annotator.detach([product])`에서 `HydraTexture has no attribute split`으로 실패했다. 설치 Replicator1.11.35의 `annotators.py`는 단일 HydraTexture 인수만 경로로 바꾸며 목록 원소를 바꾸지 않는다. 이미 연결한 제품을 모두 해제하는 `detach()`로 수정하고 기존 물리 시험처럼 `app.close(wait_for_replicator=False)`를 사용한다. 뒤의 access violation은 종료 중 후속 오류이며 최초 Python 예외와 구분한다. 감독 Job의 종료 코드0만으로 촬영 성공을 판정하지 않고 결과 JSON의 passed·두 RGB 해시·정리 오류 없음·소유 트리 종료를 함께 확인한다.

187에서는 두 RGB 촬영과 소유 트리 종료를 확인했지만 SDK shutdown 뒤 Python 저장 코드가 실행되지 않아 결과 상태가 starting에 남았다. RGB와 해시는 유효하며 전체 촬영 프로토콜 성공으로 바꾸지 않는다. 기존 물리 시험처럼 `app.close` 전에 최종 결과를 저장하고, 종료 여부는 외부 Job 영수증으로 확인하도록 수정했다. 같은 결과 기록 순서로 후속 촬영을 실행한다.

188에서는 최종 결과 passed, 같은 물리 프레임의1280×720 RGB 두 장, 원본 해시와 cleanup 오류0을 확인했다. 외부 Job의 입력 바이트 검사·종료·남은 소유 PID0도 확인했다. 운송이나 생산 성능이 아닌 정지 장면 촬영의 성공 범위다. [공개 촬영 근거](images/semiconductor-layout-local45-188/provenance.json)

## FOT-121 — Linux 중계의 초기 상태 기록 전 준비 시간 초과

061 운송 시도는 기존20초 중계 준비 한도에서 종료됐다. Windows 중계는 peer를 기다렸고 Linux 상태 JSON과 stdout은 생성되지 않았다. 시작 때 Linux 자식 하나가 D 상태였으나 해당 지점이나 차단 원인을 특정할 stack 자료는 없다. GPU 장면과 Nav2는 시작되지 않았고 양쪽 중계·소유 트리 종료 및 정리 오류0을 확인했다. 이후 같은 Ubuntu에서 ROS 환경 구성과 동결 소스 import·공유 파일 쓰기는0.4초 미만에 끝났으며, 이것만으로 실제 원인이 해결됐다고 판단하지 않는다.

다음 실행의 Linux 중계 로그에는 셸 진입과 ROS 환경 준비 완료를 각각 기록한다. 기존20초 한도·DDS·전달·명령 시각 정책은 유지한다. 재발하면 두 단계 로그와 초기 상태 파일의 유무로 Python 진입 전후를 구분하고, 시간 한도를 무작정 늘리거나 GPU 본 실행을 반복하지 않는다.

062는 두 셸 단계와 중계 연결을 통과해 실제 SDK·센서·Nav2 기동 뒤 orders 단계에 도달했다.061의 차단 원인이 특정된 것은 아니므로 중계 지연 원인 미확정 기록은 유지한다.062의 새 실패는 아래 캐리어 ID 계약 오류다.

## FOT-122 — 반복 lot의 캐리어 ID와 시각 슬롯 계약 누락

062의 최초 임무 오류는 `empty visual requires distinct declared tote IDs and sources`다. 생성기가 만든 `SC-CARRIER-*`가 기존 `F4-TOTE-*` 식별자 규칙에 맞지 않았다. 물류 이동 전에 발생했고 SDK·Nav2·중계와 소유 트리는 모두 종료됐다. 검증기를 느슨하게 하지 않고 생성 ID를 `F4-TOTE-SC-*`로 맞췄다.

같은12 lot CPU 흐름에 제품·빈 캐리어의 시각 상태와 실제 지원면 배치를 연결하자 추가로 출하대의7개 슬롯 부족과 미투입 lot/반납 캐리어의 source 슬롯 중복이 재현됐다. 이 둘은062의 실제 종료 원인이 아닌 CPU에서 확인한 후속 문제다. 출하 적치대를 트레이 로드포트로 바꾸고, 전체 선언 주문의 source 슬롯 계획을 제품과 반납 캐리어가 함께 사용하도록 했다. 반납 캐리어는 자기 lot가 떠난 자리를 사용하며 다른 대기 lot의 슬롯을 빼앗지 않는다. 기본 비-stream 제품 투영은 유지한다.

확장한 CPU 검사에서12 lot 출하·12개 반납·재고 보존·4대 반복 참여와 모든 상태의 슬롯 중복 없음·지원면 용량을 함께 확인했다. 관련14검사가 통과했고 기존 POSIX 읽기 권한 검사1개는 Windows에서 제외됐다. 새 소스의 실제 운송 완주는 아직 남아 있다.

## FOT-123 — 가상 충전을 끈 운송 기준선에 충전 보조 파일 옵션을 지정

063은 운송 확인을 위해 강제 가상 충전 옵션 없이 시작했다. 이 경우 `charging-state.json`을 생산하지 않는데, 과거 충전 실행용 API 실행기가 `--charging-source`를 계속 지정해 존재하는 본 상태도503으로 응답했다. 시뮬레이션이나 주행 실패가 아닌 API 실행 옵션의 불일치다. API·웹 소유자를 함께 닫은 뒤 이 기준선에서는 충전 보조 파일 옵션을 제외해 다시 연결했다. 같은 실행 ID·4대 상태·Linux의 실제 본 상태 경로와 반도체 설정, Next 프록시 연결을 확인했다. 가상 충전이 켜진 실행에서는 원래의 보조 파일 결합·동일 실행 검사·실패 처리를 유지한다.
