# Factory Ops Twin

두 AMR이 공장에 자재를 공급하고 완료품을 출하하는 과정을 검증하는 프로젝트입니다. 가상 산업용 센서 모듈 공장을 대상으로 주문·버퍼·공정·운반 상태를 연결하고, 배차와 공유 통로 복구를 비교합니다.

현재 **Isaac Sim 6.2·ROS 2 Jazzy·Nav2로 AMR 한 대의 주문 운송을 검증**했습니다. AWS L40S에서 Nova Carter가 S1→CI로 자재를 운반하고, 설정된 공정 처리 20초 뒤 CO→OUT으로 완료품을 운반했습니다. 네 navigation goal, 운영 원장, 실제 ROS bag, 전체 휠 궤적과 촬영 파일을 대조했습니다. 두 AMR의 교통 제어와 정책별 생산성 비교는 후속 단계입니다.

![Isaac Sim 6.2에서 렌더한 G1 공장 배치](docs/images/g1-factory-overview.png)

위 이미지는 G1 실행의 실제 공장 렌더입니다. 40×24 m 가상 센서 모듈 공장을 설비 박스와 인계 위치로 표현한 배치 검증 단계입니다. 현장 CAD와 생산 데이터를 동기화한 실공장 복제는 아닙니다. [초기 구상도](docs/images/concept-v3.png)는 계획을 설명하기 위해 생성한 이미지이며, 실행 증거와 구분합니다.

![Nova Carter의 S1·CI·CO·OUT 주문 운송 타임랩스](docs/images/g1-amr-mission.gif)

위 영상은 같은 G1 실행의 실제 주행 RGB를 묶은 타임랩스입니다. GIF는 약 2초 간격의 프레임을 5 fps로 재생합니다. 전체 원본 195프레임을 10 fps로 묶은 [1280×720 MP4](docs/images/g1-amr-mission.mp4), [출하 종료 원본 PNG](docs/images/g1-amr-out.png), [미디어 출처·해시](docs/images/g1-media-provenance.json)도 제공합니다. 원본의 sim time은 별도 명세에 보존합니다. 조향에는 휠 엔코더·RTX LiDAR·AMCL을 사용하고, 시뮬레이터 GT는 평가와 이미지 대응에만 사용합니다.

## 공장에서 AMR이 하는 일

```text
S1 / S2 / S3     CI            CELL1           CO          OUT
키트 준비   →   투입 버퍼  →  조립·검사·포장  →  완료 버퍼  →  출하
       AMR 운반                  논리 공정             AMR 운반
```

S1~S3은 동일한 키트 토트를 준비하는 위치입니다. 하나의 주문은 하나의 토트를 운반하며, 토트 안의 제품 수는 모델에 넣지 않았습니다. CI에 토트를 내리면 AMR은 다음 작업으로 이동할 수 있습니다. 공정 처리가 끝나면 CO에서 OUT으로 가는 새 운반 작업을 만들고, 빈 AMR 중 하나에 배정합니다.

현재 적재·하차는 논리 상태 전이입니다. 지게차의 포크나 로봇 팔의 파지 동작은 포함하지 않습니다. 공정은 작업자 1명, CI·CO 각 2슬롯, 설정 처리 시간 20초로 표현합니다. 이 값은 현장 측정값이 아닌 실험 설정입니다.

## 구현된 기능

| 기능 | 현재 동작과 검증 범위 |
|---|---|
| 공장 배치 | 40×24 m, 6개 인계 위치, 2개 대피 포켓, 19개 설비 장애물 |
| 지도 생성 | 50 mm 격자, 물리 장애물 지도와 정책 keepout 지도 분리, ROS PGM·YAML 출력 |
| USD 생성 | 비충돌 시각용 바닥과 물리 Plane 분리, 바닥·설비·마커 USDA 생성과 Isaac Sim 파싱 통과 |
| GPU 장면 검사 | Z-up·m 단위, 활성 충돌 API 20개·설비 19개·비충돌 마커 8개 검사, 1280×720 RGB와 SHA 저장 |
| GPU 종료 판정 | 별도 worker 종료 뒤 supervisor가 실행 ID·장면·PNG·종료 코드를 대조. L40S 실행에서 최종 PASS 확인 |
| 단일 AMR 운송 | S1 pickup→CI drop→공정 20초→CO pickup→OUT drop, 실제 goal UUID·성공 결과·신선한 map pose를 확인한 뒤 인계 확정 |
| 주행 평가 | 실제 충돌 형상의 회전 외곽 약 0.637 m + 여유 0.2 m로 전체 60 Hz 궤적 검사. 계획에는 위치 추정 여유 0.3 m를 추가하고 CI·CO·OUT 접근 정책 폭을 2.6 m로 설정 |
| 주행 명령 프로파일 | 정상 가감속·곡선 감속·회전 전 제동·회전 후 출발·후진 전환과 휠 속도/가속도 제한을 60 Hz 물리 루프에 적용. L40S에서 명령 한계와 실제 차체·엔코더 응답을 별도로 검사 |
| 주행·센서 수집 | G1의 60 Hz 휠·GT 궤적, 주행·조감 RGB 쌍, ROS bag 보존. G0b에서 RGB/Depth/semantic/bbox·카메라 파라미터 3세트의 정렬도 별도 검사 |
| ROS 2 연결 | RTX `/scan`, 엔코더 `/odom`, `/clock`, `map→odom→base_link→base_scan` TF, AMCL·Nav2·action 결과를 연결 |
| 운영 원장 | 주문·운반·버퍼·공정·완료 상태, 버전 검사, 중복 이벤트 억제, 로그 재생 |
| 배차 | 공통 버퍼·가용성 조건을 통과한 작업에 FIFO 또는 이동 ETA 적용 |
| 통로 예약 | 지정된 3개 자원과 출구의 원자 예약, stale 점유 유지, release ACK 검사 |
| 복구 결정 | 5초 지속 wait cycle의 양보 결정과 ACK 이후 재개. 60초 stall과 구분 |
| 로컬 데모 | 동일한 6개 주문·2개 논리 AMR의 두 정책 실행, CSV·JSON·HTML·SHA256 명세 생성 |

통로 로직은 호출자가 준 추정 위치·점유·출구 상태를 입력으로 받습니다. 로컬 교통 trace는 작성된 관측을 넣어 결정 규칙을 검사합니다. CPU 운반 데모는 격자 거리와 설정 속도로 시간을 계산하며 로봇 간 간섭과 예약 대기를 반영하지 않습니다. 두 결과를 합쳐 물리 주행 성능으로 해석할 수 없습니다.

## G1 실행 결과

검증 실행은 새 주행 프로파일을 적용한 `g1-010`, 코드 SHA는 `ec06754c62854c9b6358f67007c23a3fef7ede81`입니다. 두 worker의 정상 종료 후 ROS 이미지에서 CDR 메시지와 원본 파일을 검사해 `FACTORY_OPS_G1_VALIDATION=PASS`를 확인했습니다.

| 항목 | 실제 기록과 범위 |
|---|---|
| 운송 | 주문 1건, navigation goal 4개 성공, OUT 인계 시각 194.517 sim s |
| 궤적 | 60 Hz 원본 11,597개, 이름이 결속된 휠 각도 재적분·프레임 연속성·자세·지도 여유·주행 명령 한계 검사 통과 |
| 촬영 | 1280×720 주행·조감 RGB 195쌍, 원본 PNG 390장과 시각·pose·SHA 보존 |
| 종착 위치 | 정지 후 GT 기준 OUT 중심까지 0.0414 m. 반복 도킹 정확도는 미평가 |
| 위치 추정 | AMCL 대 GT 최대 XY 오차 0.2097 m, RMS 0.1067 m, 시각이 대응된 1,685개 표본 |
| ROS bag | SQLite integrity OK, `/scan` 1,902개·`/cmd_vel` 3,035개·`/clock` 11,412개, 네 goal UUID의 성공 상태와 원장 이벤트 대조 |

평가 여유는 앞선 실패 때와 같은 0.2 m입니다. 초기 G1에서는 바닥 충돌 표현과 저속 제자리 회전 문제를 대조 시험으로 분리했고, 2 m 접근 정책 통로가 위치 추정 오차를 수용하지 못한 실행도 실패로 보존했습니다. 현재 통로 설계와 제어 설정의 근거는 [GPU 연동 안내](docs/gpu-handoff.md)에 적었습니다. 위 결과는 한 주문의 검증이며 반복 성공률·물리 접촉 수·생산성 개선율을 뜻하지 않습니다.

## 주행 동작과 계측

Nav2의 속도 요청은 60 Hz 주행 프로파일을 거쳐 좌우 휠의 속도 목표로 전달됩니다. 정상 정지는 감속하고, 전진 중 제자리 회전을 요청하면 먼저 전진을 줄입니다. 회전 후 출발과 전후진 전환도 정지 단계를 거칩니다. 곡선에서는 `|v × ω|` 명령 한계에 맞춰 속도를 낮춥니다. 통신 명령 만료나 실행 오류에는 별도의 고장 정지를 적용합니다. 물리 엔진이 에셋의 구동·접촉 조건에 따라 차체의 움직임을 계산합니다.

![Isaac Sim 실제 기록의 주행 명령·휠 엔코더·차체 응답 비교](docs/images/amr-drive-response.png)

`motion-001`은 기본 Nova Carter와 Plane 바닥을 사용한 **별도 주행 시험**입니다. 76초 동안 4,560개 물리 표본과 10 Hz 원본 PNG 761장을 수집했습니다. 고정 시간표가 명령을 만들고 GT는 계측과 촬영에만 사용합니다. 공장 운송 결과와 구분합니다.

| 계측 항목 | 실제 차체 응답 |
|---|---|
| 직선 정상 속도 | 0.6014 m/s, 요청·적용 명령 0.6 m/s |
| 가속 10→90% 상승시간 | 1.925초, 마지막 1초의 실측 정상 속도 기준 |
| 정상 제동 | 1.733초·0.5207 m, 약 0.6 m/s 직진 뒤 정지 요청 |
| 제자리 회전 | 요청·적용 0.8 rad/s, 실측 0.7645 rad/s |
| 좌회전 곡선 | 적용 0.3162 m/s·0.6325 rad/s, 실측 0.3010 m/s·0.7475 rad/s |
| 후진 전환 | 적용 명령의 0 속도 표본 확인, 차체의 양→음 전환은 요청 후 1.213초 |

제동의 정지 기준은 전방 속도 절댓값 0.02 m/s 이하를 0.5초 유지하는 것입니다. 과도응답 계측에는 0.1초 이동 평균을 사용합니다. 곡선에서 보이듯 명령과 실제 응답은 다릅니다. 가감속·횡가속·휠 제한은 작성한 명령 한계이며, 실물 모터·적재량·바닥 조건에 맞춘 보정값은 아닙니다.

![전진 제동·제자리 회전·재출발의 실제 렌더 구간](docs/images/amr-turn-transition.gif)

GIF는 제동·회전·재출발 구간 16초를 확대해 10 fps로 재생합니다. [전체 76초 MP4](docs/images/amr-drive-profile.mp4)는 원본 촬영 간격과 같은 10 fps이며 타임랩스가 아닙니다. [설정·계측 정의](configs/factory-v3/motion-qualification.json), [미디어 출처·해시](docs/images/motion-media-provenance.json)에 실행 ID와 원본 프레임 연결을 기록했습니다.

## 로컬 실행

Python 3.12와 [uv](https://docs.astral.sh/uv/)를 사용합니다. Python 런타임 의존 패키지는 없습니다.

PowerShell:

```powershell
cd "$HOME/factory-ops-twin"
uv sync --locked
uvx --from pyright==1.1.414 pyright
uv run python -m unittest discover -s tests -v
uv run python -m factory_ops prepare --output runs/local
uv run python -m factory_ops verify runs/local
uv run python -m factory_ops replay runs/local/fifo/events.jsonl --expected runs/local/fifo/snapshot.json
```

`runs/local/report.html`을 브라우저에서 열면 배치도, 정책별 완료 기록과 이벤트 원장을 볼 수 있습니다. 준비 명령은 클라우드 인스턴스를 만들거나 실물 로봇에 명령하지 않습니다.

```text
runs/local/
  report.html              배치와 CPU 결과
  geometry.json            정상·폐쇄 경로와 배치 검사
  manifest.json            입력·출력 SHA256와 검증 범위
  maps/                    map.pgm, keepout.pgm, YAML, 명세
  scene/                   factory-v3.usda, 명세
  traffic-checks.json      stale 점유·wait cycle의 작성된 관측 trace
  fifo/, eta/              summary.json, snapshot.json, events.jsonl,
                           orders.csv, assignments.csv
```

CPU 운영 데모의 `physical_collisions`, `nav2_success_rate`, `rtf`, `peak_vram_gb`는 `null`로 저장하고 `primary_evaluation=false`를 붙입니다. G1의 한 주문 실행과 CPU 논리 데모, G0b의 촬영 부하 관측은 별도 기록입니다. G0b의 기존 Cube 바닥은 후속 대조 시험에서 주행 불안정을 보였으므로 당시 이동 거리와 RTF를 안정된 Nav2 성능의 근거로 사용하지 않습니다. 물리 접촉 수는 측정하지 않았습니다.

`verify`는 저장된 파일의 SHA256 일치를 확인하고, `replay`는 입력 이벤트 본문·순서·버전을 재생 결과와 대조합니다. 명세 자체의 서명이나 발행자 인증을 제공하는 기능은 아닙니다. 출력 폴더에 무관한 파일이 있으면 준비 명령이 덮어쓰기를 거절합니다.

CSV에서는 수식으로 해석될 수 있는 식별자 앞에 작은따옴표를 붙입니다. 원래 식별자는 JSON 원장과 snapshot에 보존합니다.

## 코드 구조

```text
configs/factory-v3/    배치·교통 자원·로컬 주문과 설정
factory_ops/
  layout.py           배치 검증, 격자 경로, 지도 출력
  footprint.py        실제 로봇 충돌 외곽과 지도 여유 검사
  motion.py           휠 각도 적분과 명령 watchdog
  drive_profile.py    정상 가감속·회전 전환·곡선·휠 제한, 별도 고장 정지
  motion_validation.py 기록한 명령 경계와 물리 응답의 독립 검사
  navigation.py       goal 결과와 CI·CO 인계·공정 연결
  usda.py             USD 텍스트 출력
  operations/         상태 모델, 원장, 배차
  traffic.py          예약·복구 결정
  traffic_fixtures.py 작성된 관측에 따른 결정 검사
  smoke.py            CPU 이벤트 실행
  report.py, cli.py   결과 보고와 명령
tests/                표준 unittest 회귀시험
scripts/isaac/        정적 장면·단일 AMR·RTX LiDAR·실제 RGB 수집
scripts/ros/          Jazzy Nav2 실행, mission action client, CDR·원본 증거 검사
scripts/brev/         코드 SHA를 고정한 두 컨테이너 실행·종료 supervisor
docs/                 설계 경계와 GPU 연동 순서
```

## 다음 실행 순서

1. G2: 두 Carter의 namespace·센서·TF를 분리하고, 예약·양보·stale 점유 유지와 복구 절차를 주행에 연결합니다.
2. G3: 같은 주문·장애 입력으로 FIFO/ETA를 정상·폐쇄·급증 조건에서 비교합니다. 부하 보정 후 10개 seed로 총 60회를 평가할 계획입니다.
3. G4: 수집 범위를 다양한 장면으로 늘리고 ROS bag·센서 정렬·라벨 품질을 검사합니다. 현재의 한 장면·인접 프레임을 독립 장면으로 세지 않으며, G0b·G1 자료는 학습용으로 승인하지 않았습니다.

환경 선정과 데이터 보존 순서는 [GPU 연동 안내](docs/gpu-handoff.md), 구현 경계는 [설계 설명](docs/architecture.md)에 적었습니다.

## 데이터와 에셋

공장 설비와 마커는 자체 작성한 박스 형상이며, 로봇은 NVIDIA 제공 Nova Carter를 참조합니다. 원본 로봇 에셋은 Git에 넣지 않습니다. 제공 에셋과 향후 도입할 에셋은 각각의 이용 조건을 따릅니다. 생성한 구상도는 시뮬레이션 실측 증거로 사용하지 않습니다.

코드·설정·문서와 README용 미디어만 Git에서 관리합니다. `runs/`, 대형 USD·원본 영상·데이터, 인증 파일은 Git에서 제외합니다. GPU 환경을 중지하기 전에는 결과를 별도 저장소로 복사하고 SHA256을 대조해야 합니다.

AWS L40S 환경에서는 Stop→Start 1회 후 결과 파일 29개·코드·캐시 이미지가 보존되고 컨테이너가 GPU를 다시 인식하는 것을 확인했습니다. 이 검사는 재시작 후 주행이나 렌더 성능 검증을 뜻하지 않습니다.

## 참고

- [Isaac Sim 호환성](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/compatibility.html)
- [Isaac Sim Python 설치](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
- [ROS 2 설치와 지원 조합](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Brev GPU 인스턴스](https://docs.nvidia.com/brev/concepts/gpu-instances)
