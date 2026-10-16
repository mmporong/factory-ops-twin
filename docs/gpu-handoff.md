# GPU 연동과 작업 보존

로컬 준비 명령은 GPU나 Brev 계정을 사용하지 않는다. 별도 장면 검사 스크립트는 Isaac Sim 6.2 환경에서 실행한다. AWS L40S에서 G0a 정적 장면, G0b 단일 AMR 기초 검증과 Stop·재시작 보존 시험을 통과했다. 자동 Sleep 기능은 제공하지 않는다.

## 검증한 실행 환경

| 항목 | 확인 결과 |
|---|---|
| Brev VM | AWS Seoul `g6e.2xlarge`, L40S 48 GB, 8 vCPU, RAM 64 GiB, 디스크 256 GiB |
| 컨테이너 OS | Ubuntu 24.04.3 |
| 드라이버 | NVIDIA 595.91.07 |
| Isaac Sim | 6.2.0, 공식 컨테이너 compatibility checker PASSED |
| 정적 장면 | 20개 활성 충돌 API·19개 설비·8개 비충돌 마커, Z-up, 1 m 단위 |
| 실행 결과 | supervisor·worker exit 0, `passed`, 1280×720 RGB 및 SHA 대조 |
| 실행 코드 | `55e67b5c034ba91376c7f62fa0fb23aaadac73c1` |
| 실행 ID | `507d19acee99990fc495903ec0e82124` |
| 이미지 digest | `sha256:a025b65a7442d56806a1cc3f5eed081caf736c24b72d62dccf68ef66945d8690` |
| Stop·재시작 | 같은 인스턴스에서 1회 수행, 29개 파일·보존 마커·코드·캐시 이미지 digest 일치, 컨테이너 GPU 재인식 |
| 콘솔 표시 요금 | 실행 중 $3.36/시간, Stopped 상태 $0.05/시간. 2026-10-16 관측값이며 최종 청구서 검증은 아님 |

원본 이미지·JSON·로그·GPU 사용 관측과 환경 기록 29개는 같은 manifest에 묶었다. Windows와 E 드라이브의 압축본 SHA256을 원격 원본과 대조했고, 압축 내부 29개 파일도 검증했다. 실제 주행·센서·물리 충돌·RTF는 이 결과의 검증 범위에 포함하지 않는다.

Stop·재시작 이후 검증 기록은 기존 압축본과 구분해 `persistence-after-restart.json`으로 보존했다. 원격 보존 마커 SHA256, 29개 파일의 크기·SHA, 실행 코드와 캐시 이미지 digest를 다시 확인하고 같은 이미지의 컨테이너에서 `nvidia-smi`를 실행했다. 로컬 및 `E:\FactoryOpsArtifacts\g0` 복사본 SHA256은 `17c47c99fd48e6942c55cdd594745cd17ebab64c875bd770da6a00e704556ff3`로 일치한다. 재시작 후 Isaac Sim 렌더를 다시 실행한 결과는 아니다.

## GPU가 필요한 시점

로컬 운영·배치·로그 재생 검증 뒤 다음 작업부터 GPU 환경이 필요하다.

1. `factory-v3.usda`를 OpenUSD/Isaac Sim에서 열어 파싱과 충돌체를 확인한다.
2. Nova Carter 1대와 센서·ROS 2/Nav2를 실행하고 실제 footprint와 통행 폭을 검증한다.
3. 2대로 확장하기 전 VRAM·RTF·센서 비용을 측정한다.

Isaac Sim 6.2 공식 문서는 Python 3.12를 사용하고 Ubuntu 24.04에서는 ROS 2 Jazzy를 권장한다. 실제 GPU 환경에서는 공식 호환성 표와 해당 이미지에 맞는 ROS workspace 버전을 확인한 뒤 이미지 digest·드라이버·ROS 패키지 버전을 기록한다. 이 저장소의 로컬 Python 3.12 환경은 Isaac 설치 검증을 뜻하지 않는다.

## Brev를 켜기 전

1. 현재 콘솔에서 GPU·VRAM·지역·총 시간당 요금·디스크 비용·Stop 지원 여부를 확인한다. 과거에 보인 가격이나 지원 상태를 현재 조건으로 가정하지 않는다.
2. Stop을 지원하는 인스턴스를 선택한다. 해당 제공자가 Stop을 지원하지 않으면 작업 보존 절차를 확인하고 일회성 compute로 사용한다.
3. 코드의 원격 SHA와 외부 결과 저장 위치를 확보한다. 대형 에셋·캐시·영상은 Git 저장소에 넣지 않는다.
4. 첫 실행은 장면·단일 로봇 확인에 한정하고, 측정 결과로 센서 수·해상도와 다음 세션 범위를 정한다.

## 중지·삭제 전 보존 순서

```text
실행 종료 → 코드 commit/push → 결과·환경 명세 외부 복사
        → 파일 SHA256 대조 → 인스턴스 Stop → 콘솔 상태·과금 상태 재확인
```

코드는 GitHub에, 실행 결과·영상·제공 에셋은 별도 지속 저장소에 둔다. 로컬 Windows의 결과 보존 위치는 `E:\FactoryOpsArtifacts`다. 이 폴더를 유일한 백업으로 사용하지 않는다. GPU 결과와 압축본을 로컬 및 E 드라이브로 전송하고 SHA256 대조를 완료했다.

외부 복사에는 실행 ID, 코드 SHA, 입력·출력 SHA256, 이미지 digest, 로봇·센서 설정, 시뮬레이션 시간과 wall time, GPU·드라이버, VRAM·RTF 기록을 함께 남긴다. 인증 정보는 추적하지 않는 설정이나 SSH 인증 도구로 전달하며 저장소·로그에 기록하지 않는다.

에이전트가 종료 후 Stop까지 처리하려면 해당 세션에 Brev 제어 경로와 유효한 인증, 종료 대상 인스턴스 식별자가 있어야 한다. 이 환경에서 에이전트의 CLI Stop, 콘솔 Stopped 전환, Start 이후 보존 검증을 완료했다. Stop 이후에도 콘솔에는 시간당 요금이 남는다. 중지 성공이나 남은 비용을 확인하기 전에는 비용 차단 완료로 보고하지 않는다.

현재 인스턴스 이름은 `factory-ops-twin-g0`다. Windows PowerShell에서 기존 WSL Ubuntu의 인증된 Brev CLI를 사용한다. `Stop`은 VM과 디스크를 보존하고, `Delete`는 사용하지 않는다.

```powershell
wsl -d Ubuntu -- sh -lc 'brev start factory-ops-twin-g0'
wsl -d Ubuntu -- sh -lc 'brev refresh'
wsl -d Ubuntu -- sh -lc 'brev ls'
wsl -d Ubuntu -- sh -lc 'brev shell factory-ops-twin-g0'
```

원격 셸의 프로젝트 위치는 `$HOME/workspace/factory-ops-g0/project`, 결과·로그·SDK 캐시는 그 상위 폴더에 있다. 작업을 마치고 백업을 대조한 뒤 원격 셸에서 `exit`하고 PowerShell에서 아래 명령을 실행한다. `brev ls`의 최종 상태와 콘솔 요금을 확인한다.

```powershell
wsl -d Ubuntu -- sh -lc 'brev stop factory-ops-twin-g0'
wsl -d Ubuntu -- sh -lc 'brev ls'
```

## Isaac Sim 6.2 장면 검사

`scripts/isaac/stage_smoke.py`는 신규 또는 빈 결과 폴더만 허용한다. 장면 SHA256, 기본 prim·좌표축·단위, 20개 충돌 API prim·19개 설비·8개 마커를 검사하고 1280×720 RGB PNG와 `stage-smoke.json`을 저장한다. 충돌 API 개수는 USD 구성 검사이며, 물리 충돌이나 AMR 주행 성공을 증명하지 않는다.

기본 CLI는 표준 라이브러리 supervisor로 실행하고 Isaac SDK는 별도 worker 프로세스에서 사용한다. worker는 종료 요청 전에 검증 checkpoint를 쓰고, SDK의 fast shutdown으로 종료한다. supervisor가 worker 종료 코드, 실행 ID, 장면 SHA, 고정 경로의 PNG 구조·SHA를 대조한 뒤에만 최종 `passed`로 승격한다. worker가 남긴 `validated`와 종료 코드 0 중 하나만으로 성공 처리하지 않는다.

아래 경로는 프로젝트를 `/workspace/project`에 마운트한 Isaac Sim 컨테이너 안의 경로다. 로컬 `prepare` 결과의 `runs/local/scene/factory-v3.usda`도 함께 전달해야 한다.

```bash
cd /isaac-sim
timeout --signal=TERM --kill-after=30s 900s \
  ./python.sh /workspace/project/scripts/isaac/stage_smoke.py \
  --scene /workspace/project/runs/local/scene/factory-v3.usda \
  --output /workspace/results/g0-stage-001
```

스크립트는 Isaac Sim 설치나 Brev 생성·중지를 수행하지 않는다. 위 명령은 전체 Python 프로세스에 900초 제한을 두고, TERM 뒤 30초 안에 종료되지 않으면 강제 종료한다. timeout 종료 코드 124·137, JSON 부재, PASS sentinel 부재는 실패로 처리한다. Docker 실행도 명명한 컨테이너만 대상으로 외부 제한과 종료 확인을 적용한다. JSON의 `status=passed`, 종료 코드 0, `FACTORY_OPS_ISAAC_SMOKE_STATUS=PASS`, 실제 PNG를 함께 확인한다. 로컬 compile과 정적 검사는 GPU 실행 검증을 대신하지 않는다.

공식 컨테이너 `nvcr.io/nvidia/isaac-sim:6.2.0`을 사용하고 pull 이후 digest를 기록한다. `ACCEPT_EULA=Y`가 필요하다. 기본 익명 원격측정은 컨테이너 환경변수 `OMNI_TELEMETRY_DISABLE_ANONYMOUS_DATA=1`로 차단하며, 선택 항목인 `PRIVACY_CONSENT=Y`는 설정하지 않는다. 첫 검사에는 WebRTC·Jupyter 포트를 공개할 필요가 없다.

## G0b 단일 AMR 검사

`scripts/isaac/robot_qualification.py`는 같은 supervisor를 사용하며, 기존 공장 장면 증거와 추가 로봇 증거를 모두 통과해야 한다. Nova Carter의 활성 충돌 형상에서 geometry extent를 다시 계산하고, root 기준 변환으로 합친다. vendor USD의 저장된 `extent`나 기존 29×29 계획 probe를 실제 로봇 크기로 간주하지 않는다. 접촉 offset·동적 충돌·안전 인증은 이 계산의 검증 범위에 포함하지 않는다.

로봇은 S1에서 짧은 개방 루프 휠 명령으로 움직인다. 120회 정지 warm-up 뒤 600회 측정 루프에서 조감 프레임과 RGB·optical-axis depth·semantic ID·bbox·카메라 파라미터를 수집하고, 같은 물리 프레임·시각·pose를 대조한다. GIF는 원본 PNG의 파생물이며, 원본과 frame manifest를 함께 보존한다. ROS 2 Jazzy의 `/clock` 수신은 센서 토픽·TF·Nav2 연동 완료를 뜻하지 않는다.

```bash
cd /isaac-sim
timeout --signal=TERM --kill-after=30s 900s \
  ./python.sh /workspace/project/scripts/isaac/robot_qualification.py \
  --scene /workspace/project/runs/local/scene/factory-v3.usda \
  --layout /workspace/project/configs/factory-v3/layout.json \
  --output /workspace/results/g0b-001
```

G0b `g0b-008` 실행에서 `FACTORY_OPS_G0B_STATUS=PASS`, supervisor·worker exit 0, `status=passed`, `cleanup=worker_process_exited`를 확인했다. 실행 코드와 원본 증거는 다음과 같다.

| 항목 | 실제 기록 |
|---|---|
| 코드 SHA | `8a620fb5948ba43bbd970d1194027a98bb89a5a0` |
| 실행 ID | `f72a0221fb2bff2271ca9fe4629bd14f` |
| 로봇 충돌 형상 | Boundable geometry 13개, root 기준 외곽 약 0.727×0.493×0.480 m |
| 지도 여유 | 회전 범위를 포함한 원의 반지름 0.637 m + 설정 여유 0.2 m. 위치 8곳과 601개 궤적 샘플 검사 통과 |
| 주행 | 120회 warmup 후 600회 측정, 10 sim s, 순변위 0.626686 m |
| 촬영 | 1280×720 원본 PNG 100장, 640×360 GIF 100프레임·100 ms 간격 |
| 센서 | chassis에 결속한 평가용 가상 카메라 640×360, RGB·float32 depth·uint32 semantic·bbox·카메라 파라미터 3세트 |
| ROS | Jazzy `/clock` 358개 수신, 비감소·시간 진행·주행 구간 포함 검사 |
| RTF | 10 sim s / 44.272667 wall s = 0.225873. 렌더·PNG·센서 저장 포함, warmup·초기 prime·GIF 인코딩 제외 |
| GPU 메모리 | 컨테이너 전체 수명 동안 1초 간격 114회 관측, 최대 4757 MiB. Nav2 또는 2대 부하 측정은 아님 |
| 원본 manifest | 실행 폴더 129개 파일의 크기·SHA256, 원본 PNG와 GIF·센서·궤적 연결 검사 |
| 시각 검수 | 단계별 checkpoint 4장, 출발 전·첫·중간·마지막 PNG에서 차체와 바퀴가 유지됨을 확인 |

`CaptureSession`이 로드 직후 medium RGB render product를 만들고 주행 종료까지 유지한다. overview는 첫 overview 촬영 때 생성해 유지하며, onboard annotator는 warmup 뒤 하나씩 연결하고 정지된 물리 시각·pose를 대조하며 prime한다. 측정 루프에는 이 초기화 시간을 넣지 않는다. `render_graph_primed_before_warmup`은 medium·overview에 대한 기록이며, onboard 초기화는 warmup 이후다.

초기 실행에서는 자동 증거 검사는 통과했지만 차체가 보이지 않는 영상이 남았다. 실패 영상은 보존하고 시각 검수를 추가했다. 지속형 캡처 수명주기로 바꾼 실행에서 회복을 확인했으며, SDK 내부 원인을 확정한 결과로 해석하지 않는다. vendor geometry·joint·material, instanceable 설정과 Fabric 설정은 바꾸지 않았다.

Depth의 `+inf`는 같은 프레임의 semantic ID 0 `BACKGROUND` 픽셀에서만 허용한다. NaN·`-inf`·0·음수, foreground의 무한대는 거부한다. semantic과 bbox annotator는 ID 공간이 달라 두 `idToLabels`를 분리해 보존·검사한다. 첫 실행에서 관측한 semantic ID 0과 bbox ID 0의 의미가 달랐으므로 두 map을 합치지 않는다.

센서 원본·궤적은 시뮬레이터 GT를 평가와 라벨 대응 확인에만 사용한다. G0b의 검증 범위에는 센서 ROS 토픽·TF·LiDAR·Nav2 goal, 물리 접촉 수, 생산성 비교와 학습용 데이터 승인이 포함되지 않는다. 생성 USD는 root layer를 export해 로봇 reference를 유지한다. 실패한 `g0b-003`의 flattened 진단 USD는 비공개 실행 자료로 보존한다.

G1 대조 시험에서 기존의 얇은 scaled Cube 바닥 충돌체가 초기 무명령 이동과 caster 불안정을 유발하는 것을 확인했다. 따라서 위 G0b의 이동 거리와 RTF는 당시 촬영 파이프라인의 실행 기록으로만 해석한다. 안정된 구름 주행이나 Nav2 운반 성능의 근거로 사용하지 않는다. 새 생성기는 시각용 `FloorVisual`과 물리 접촉용 Z=0 `Floor` Plane을 분리한다.

성공·실패 실행과 진단·환경·GPU 관측을 묶은 최종 manifest는 899개 파일이다. 원격 압축본 SHA256은 `c07e2ed0a9237127dd56527a28d0b73126ada383e2b50101a3cbf2885315dbfd`, manifest SHA256은 `afa15612dae3dae0d2aac9e939722c3136eb34184cedbe29aa5829c30d68f8ea`다. 외부 복사 대조 결과는 별도 `g0b-backup-verification.json`에 기록한다.

## G1 물리 기준과 실행 방법

같은 L40S·Isaac Sim 6.2 이미지에서 30초 단위 대조 시험을 수행했다. 앞 8초는 무명령, 이후 15초는 휠 속도 명령, 나머지는 정지 구간이며 선속도 가속도 한계는 0.25 m/s²다. 아래 수치는 이 통제 시험의 결과로, Nav2 성능 평가와 구분한다.

| 바닥·배치·촬영 조건 | 휠 명령 | 최대 roll/pitch 절댓값 | 마지막 XY 위치 | 실행 폴더 |
|---|---:|---:|---|---|
| 빈 장면·공식 GroundPlane·원점 배치 | 2 rad/s | 0.001582 rad | (4.2311, −0.0655) m | `g1-physics-vanilla-001` |
| 공장 Cube 바닥·API (4,4) 배치·촬영 없음 | 2 rad/s | 0.512334 rad | (4.0939, 1.8898) m | `g1-physics-factory-api-001` |
| 같은 공장·API 배치에서 바닥 충돌만 Plane으로 교체 | 2 rad/s | 0.001582 rad | (8.2311, 3.9337) m | `g1-physics-factory-api-plane-001` |
| 새 USD의 Plane·로봇 전체 배치·섀시 추적 카메라 | 2.857143 rad/s | 0.001582 rad | (10.0312, 3.8883) m | `g1-physics-factory-authored-manual-001` |

Cube 조건은 구동 명령을 주기 전부터 약 0.49 m 이동했다. Plane 조건은 빈 장면의 안정된 결과를 재현했다. 이 대조는 바닥 충돌 표현의 영향을 분리한 것이며 PhysX 내부의 접촉 계산 원인까지 확정한 결과는 아니다. 측정한 휠 충돌 반경 약 0.14 m는 안정된 직선 시험의 이동량과도 일치했다.

`g1-physics-turn-baseline-001`에서는 같은 장면에서 정지·직진·제자리 회전을 60초 동안 수행했다. 600개 관측에서 target 쓰기 전·후와 물리 step 후의 articulation은 모두 awake였다. 0.08 rad/s 회전 요청은 휠 target readback과 일치했지만 14.9초 동안 yaw가 0.00362 rad만 변했다. 이어진 0.2 rad/s 구간에서는 7.9초 동안 1.221 rad 회전했고 캐스터가 재정렬됐다. 이 시험은 낮은 명령의 접촉·캐스터 재정렬 문제를 재현했으며, 마찰과 기구의 기여를 각각 분리한 시험은 아니다.

Nav2 RPP는 현재 odom 각속도에서 `max_angular_accel / controller_frequency` 범위로 회전 요청을 제한한다. 기존 0.8 rad/s²·10 Hz 설정은 정지 시 0.08 rad/s를 반복하는 조건이었다. 후속 설정은 RPP 제한만 2.0 rad/s²로 바꿔 첫 요청이 0.2 rad/s에 도달하게 한다. 실제 휠 명령의 가속도 제한은 bridge의 0.5 rad/s²를 유지하고, 회전 상한 0.5 rad/s와 goal 허용치도 유지한다. 에셋의 sleep·마찰·solver 설정은 바꾸지 않는다. [Nav2 Jazzy RPP 구현](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)

로봇은 물리 시작 전에 전체 USD root를 이동한다. 초기화 전에 `WheeledRobot(... positions=...)`로 차체만 옮기면 sibling 링크의 USD 좌표가 아직 원점에 남아 사전 형상 검사를 왜곡하므로 이 경로를 사용하지 않는다. 런타임에는 관절 속도 명령을 사용하며 root 이동이나 GT pose 보정은 하지 않는다.

실행은 Isaac 이미지와 ROS Jazzy/Nav2 이미지의 두 컨테이너를 사용한다. 기존 이미지와 결과·캐시는 보존하고, 실행마다 새 `g1-NNN` 폴더와 run ID를 만든다. 아래 supervisor는 고정한 코드 SHA와 깨끗한 작업 트리를 요구한다.

시작 상태 조회는 monotonic wall time 기준 client별 최대 2 Hz로 제한하며, 미완료 요청을 중복 발행하지 않는다. 앞선 `g1-007`에서는 controller 프로세스가 살아 있었지만 lifecycle 상태 조회에 실패해 준비 시간 초과로 종료했다. Nav2의 해당 오류 문자열은 요청을 보낸 뒤 응답 future가 성공으로 완료되지 않았을 때 발생하며, 기본 상태 조회 대기시간은 2초다. 당시 로그도 약 2초 간격과 일치한다. 요청 과다·DDS 전달 지연·server 처리 지연 중 어느 것이 원인인지는 그 로그만으로 확정하지 않는다. [Nav2 service client 소스](https://github.com/ros-navigation/navigation2/blob/645abd95f2be02a13ca539b29c2ddc065db34f89/nav2_util/include/nav2_util/service_client.hpp#L79-L86), [lifecycle 대기시간 정의](https://github.com/ros-navigation/navigation2/blob/645abd95f2be02a13ca539b29c2ddc065db34f89/nav2_util/include/nav2_util/lifecycle_service_client.hpp#L56-L60)

```bash
cd "$HOME/workspace/factory-ops-g0/project"
source_commit=$(git rev-parse HEAD)
bash scripts/brev/run-g1.sh "$HOME/workspace/factory-ops-g0" g1-101 "$source_commit"
```

실행 파일은 `scripts/isaac/nav_scene.py`, `scripts/ros/g1_mission.py`, `scripts/ros/g1.launch.py`이고, 설정은 `configs/factory-v3/g1.json`과 `nav2-g1.yaml`이다. Nav2 이미지 생성 정의는 `scripts/ros/Dockerfile`에 둔다. CPU Python 환경에는 ROS·Isaac 런타임을 설치하지 않는다.

pair 종료 코드 0만으로 완료 처리하지 않는다. 두 worker가 끝난 뒤 ROS 이미지 안에서 `/opt/ros/jazzy/setup.bash`를 source하고 다음 검사를 수행한다. `run_id`에는 해당 `pair-exit.json`의 값을 넣는다.

```bash
python3 /workspace/project/scripts/ros/g1_evidence.py \
  --output /workspace/results/g1-101 \
  --layout /workspace/project/configs/factory-v3/layout.json \
  --run-id "$run_id"
```

`g1-validation.json`의 `passed=true`, 실제 ROS bag의 action·원장 이벤트, 60 Hz 원본 궤적, RGB 초·중·끝의 차체 가시성을 함께 확인한다. 성공·실패 trial과 물리 진단 원본은 삭제하지 않는다. 압축본과 내부 manifest를 로컬 및 외장 저장소에서 대조한 뒤 GPU를 Stop한다.

`g1-008`은 네 Nav2 goal과 주문 출하를 완료했지만 최종 여유 검사에서 실패했다. 전체 20,041개 샘플 중 748개가 CI·CO 접근 정책 영역을 벗어났다. 물리 장애물 셀 침범은 아니었으며, 가장 큰 여유 부족은 약 0.134 m였다. 이 실행의 AMCL 대 GT 위치 오차는 최대 약 0.240 m였다. 원래 2 m 정책 통로에서는 평가 반경에 0.3 m를 더하면 경로가 끊기므로, 후속 배치는 CI·CO·OUT 접근 정책 폭을 2.6 m로 재작성하고 계획 여유 0.3 m를 추가한다. 설비와 인계 위치, 평가 여유 0.2 m는 유지한다. `g1-008`은 실패 기준선으로 보존하며 통로 설계를 바꾼 실행의 성공 근거로 사용하지 않는다.

## G1 검증 실행

`g1-009`는 두 worker의 종료 코드 0, 실행 ID·장면 결과 일치와 `FACTORY_OPS_G1_VALIDATION=PASS`를 확인했다. 실제 ROS Jazzy 컨테이너에서 SQLite bag의 CDR을 해석했고, 네 goal UUID의 성공 상태와 원장 이벤트를 대조했다.

| 항목 | 기록 |
|---|---|
| 소스 | `1ded05b00f68a2161c2d1c35b1d10cec45a0f9a0` |
| 실행 ID | `30554994014633d9626b2dc7ee2cdc09` |
| goal 결과 시각 | S1 5.550, CI 92.833, CO 156.867, OUT 190.717 sim s. 모두 status 4, Nav2 error code 0 |
| 공정 | CI drop 후 설정된 논리 처리 20초, CO goal 발행 112.833 sim s |
| 평가·계획 반경 | 충돌 회전 외곽 약 0.637109 m + 평가 여유 0.2 m = 0.837109 m. Nav2는 위치 추정 여유 0.3 m를 더해 1.137109 m, inflation radius 1.337109 m |
| 전체 궤적 | 11,353개, 인접 physics frame 차이 1·시각 차이 1/60초, raw 휠 이름 결속과 독립 재적분 통과 |
| 지도 여유·자세 | 전체 표본에서 평가 반경의 raw-map 침범 없음. 최대 roll/pitch 절댓값 0.0017014 rad, 한계 0.35 rad |
| 종착 위치 | 정지 후 GT 기준 OUT 중심까지 0.033370 m |
| 위치 추정 오차 | AMCL 대 GT 최대 XY 0.268595 m·RMS 0.135960 m, 시각이 대응된 1,671개 표본 |
| 휠 odometry 오차 | GT 대비 최대 XY 0.843988 m·RMS 0.375875 m, 11,353개 표본. 엔코더 적분값을 GT로 보정하지 않음 |
| 실제 bag | SQLite integrity `ok`, `/scan` 1,880·`/odom` 3,760·`/cmd_vel` 2,947·`/clock` 11,284, 양수·유한 scan point 3,916,361개 |
| 원본 촬영 | 1280×720 주행·조감 RGB 191쌍, 382 PNG. 최초·중간·마지막 frame/time/GT와 원본 SHA 대조 |
| 처리 시간 | 기록한 mission sim 구간 189.217초, worker 시작부터 캡처 세션 정리까지 wall time 405.333초. SDK·에셋·센서 준비와 warmup을 포함하며 정상 주행만의 RTF가 아님 |
| GPU 관측 | 전체 컨테이너 수명에 걸쳐 1초 간격 408개 표본, 관측 최대 3,769 MiB. 한 로봇·LiDAR 1개·RGB 촬영 조건의 기록 |
| 진행 실패 | Nav2 `Failed to make progress` 0건, 실행 로그의 spin/backup/wait 동작 시작 기록 없음 |

bag DB의 SHA256은 `d5690e07a8b0124abf3272dee7b24762b202ecece010f06bd868aea99ba9d0e8`, strict 보고서 SHA256은 `c654658838709fa0b081dda7e4be18915058d3ba79d705cddbc594d3ecc913b8`이다. README의 PNG는 실제 원본 복사본이고, GIF·MP4는 촬영 프레임에서 만든 파생물이다. 선택한 frame index·배속·인코더·원본과 출력의 해시는 `docs/images/g1-media-provenance.json`에 기록한다.

이 결과는 주문 1건의 검증이며 반복 성공률·도킹 정확도·물리 접촉 수·두 AMR의 교통 성능을 측정한 결과는 아니다. G0b·G1의 한 장면과 인접 프레임은 독립 장면 데이터셋으로 세지 않으며 학습용으로 승인하지 않았다.

## 공식 근거

- [Isaac Sim 호환성](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/compatibility.html)
- [Python 설치 요구사항](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
- [ROS 2 지원 조합](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Brev 인스턴스 개념](https://docs.nvidia.com/brev/concepts/gpu-instances)
- [Brev Stop·Start와 workspace 보존](https://docs.nvidia.com/brev/cli/instance-management)
- [Isaac Sim 6.2 컨테이너 설치](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_container.html)
- [Isaac Sim 6.2 stage API](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.core.experimental.utils/docs/index.html)
- [Replicator 조감 이미지 저장](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/replicator_tutorials/tutorial_replicator_getting_started.html)
- [Isaac Sim 6.2 데이터 수집과 opt-out](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/data-collection.html)
- [Isaac Sim 6.2 depth 센서](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/sensors/isaacsim_sensors_camera_depth.html)
- [Isaac Sim 6.2 ROS clock](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/ros2_tutorials/tutorial_series/tutorial_ros2_clock.html)
