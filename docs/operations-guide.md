# 운영 화면과 재현 실행 안내

대표 이미지·영상과 현재 구간은 [README](../README.md), 전체 완료 조건은 [완성 계획](completion-plan.md)에 있다. 아래 CPU 모델은 실제 Isaac Sim·Nav2 주행과 별도 결과다. 이미 API·웹 서버가 실행 중이면 같은 포트에 중복 실행하지 않는다.

## CPU 웹 운영 모델

Python 3.12·uv를 사용하며 Python 런타임 의존 패키지는 없다. PowerShell에서 API를 실행한다.

```powershell
cd "$HOME/factory-ops-twin"
uv sync --locked
uv run python -m factory_ops.dashboard
```

가상 에너지·충전 운영을 보려면 위 API 명령 대신 아래 명령을 사용한다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops.dashboard --web-charging-profile configs/web-charging-model.json
```

다른 PowerShell 터미널에서 웹을 실행한다.

```powershell
cd "$HOME/factory-ops-twin/dashboard"
npm ci
npm run dev -- --port 3007
```

주소는 `http://127.0.0.1:3007`, 기본 API는 `http://127.0.0.1:8765`다. CPU 모델은 공용 차량에 자재 공급·출하·빈 토트 회수를 배정하며 유한 재고와 용기 재사용을 연결한다. 웹 이동은 작성된 경로의 보간이다. 선택형 충전 모델은 작성된 에너지 계수·두 충전대 예약·정렬·기하 접촉 대기·가상 충전·이탈을 사용하며 물리 접촉 센서나 전기 충전 결과가 아니다.

## 운영 기록 저장·재생

API에 `--scenario-dir`를 지정하면 웹에 기록 저장·재생 기능이 연결된다. CPU 모델의 예는 아래와 같다. 기존 API가 실행 중이면 새 프로세스를 중복으로 띄우지 않고 해당 실행의 설정에 옵션을 추가한다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops.dashboard --config configs/factory-semiconductor --scenario-dir runs/operator-scenarios
```

1. 웹에서 운영 상태를 확인한 뒤 기록 이름을 입력하고 `최근 관측 저장`을 누른다. API가 수신한 최근120개 이내의 관측만 저장한다. 버튼을 누르기 전에는 RAM에만 두며 같은 revision·시뮬레이션 시각의 중복 표본은 제외한다.
2. 저장된 기록을 선택하고 `기록 불러오기`를 누른다. 차량 선택·상세·작업·재고·충전 표시는 선택한 관측 시각으로 바뀐다.
3. 이전·다음 버튼과 슬라이더로 표본을 고르거나 자동 재생한다. 화면에 원래 시뮬레이션 시각과 표본 간격을 표시한다. 자동 재생은900ms마다 다음 관측을 보여주는 방식이며 원속도 영상이나 물리 재실행으로 사용하지 않는다.
4. `실시간으로 돌아가기`를 누르면 계속 수신하던 현재 운영 상태가 보인다. 재생 중에는 차량 명령과 새 기록 저장이 비활성화된다.

기록에는 원래 실행 ID·관측 시각·표본 해시·공장 배치 해시를 보존한다. 다른 배치나 손상된 파일의 재생은 거부한다. 한 파일의 상한은16MiB이며 기존 파일을 덮어쓰지 않는다. 이 기록은 관제 상태의 선택 보존이며 RGB·라벨 데이터셋·rosbag·물리 재현 증거와 별도로 관리한다. 실제 ROS 관측 API에도 같은 옵션을 사용할 수 있으며 저장소는 실행별 경로로 분리한다. 물리 실행의 동결 소스·원장·명령 경로는 바꾸지 않는다.

[064의 실제4대 관측 기록](evidence/scenario-replay-198.json)은 Next.js 프록시로 저장·목록·처음/마지막 표본 재생을 확인했다. 원래 시각·실행·공장 배치·파일 해시와 빈 재생 명령 권한을 대조했고, 저장 뒤에도 live 원천이 유지됐다. 브라우저에 연결된 표면이 없어 실제 클릭·반응형 화면 캡처는 미확인이다. 이 기록은 전체 물류 완주의 증거로 사용하지 않는다.

## 실제 ROS 실행의 읽기 전용 관측

ROS 임무의 `--dashboard` 발행기가 같은 실행의 `dashboard-state.json`을 기록한다. API의 `--source`로 이 파일을 읽고, 가상 충전과 운영 사건은 같은 실행의 `charging-state.json`, `actions.jsonl`로 연결한다. `--config`는 시뮬레이션과 같은 배치를 선택한다. 아래는4대 정상 운송 배치의 예이며 `<실행명>`은 관측할 실행 폴더 이름으로 바꾼다.

```powershell
cd "$HOME/factory-ops-twin"
$taskRunRoot = Join-Path $HOME 'FactoryOpsRuntime/<실행명>/results/run'
uv run python -m factory_ops.dashboard --config configs/factory-fleet4-normal3 --source "$taskRunRoot/dashboard-state.json" --charging-source "$taskRunRoot/charging-state.json" --events-source "$taskRunRoot/actions.jsonl"
```

`map→base_link` 추정 위치·엔코더 속도·작업 원장을 표시한다. 실제 ROS 원천은 관측 전용이며 웹에서 차량 주행·일시정지·주문을 명령하지 않는다. 관측이 오래되면 stale로 표시하고 원천에 없는 SOC·충전 대기열은 추정해서 채우지 않는다. 웹 임무 경과 시각과 사건의 ROS 원본 시각을 구분한다. 웹 관측을 종료 결과나 학습 데이터 승인으로 사용하지 않는다.

실행 중 bag SQLite에는 조회나 복사용 연결을 열지 않는다. 관측은 JSON/JSONL을 사용하며 bag 분석은 기록기·소유 실행 종료를 확인한 뒤 수행한다.

후속 ROS 소스의 충전 보조 파일은 같은 충전 단계에서도1시뮬레이션 초 간격으로 갱신한다. 충전 단계 변경 사건만 저널에 추가하며 heartbeat를 사건으로 중복 기록하지 않는다. 충전 판정은 큐의 오도메트리보다 미래가 아닌 실제 추정 위치를 기존 신선도 한도 안에서 선택한다. 새 소스의 SDK 확인은 남아 있고 진행 중인054에는 소급 적용하지 않는다. [관측과 수정 범위](evidence/local45-charging-diagnostic-178.json)

[실행051 연결 기록](evidence/local45-live-web-158.json)은 실행 ID·4대 차량·배치 revision이 원천과 일치하고 Next.js 프록시까지 전달됨을 확인한 자료다. 이 실행의 API는 관측 전용이다. 차량 선택 클릭과 실제 정지·재개의 브라우저 통합 완료는 별도 확인이 남아 있다.

## 공장 운영 미디어 촬영

새 로컬4.5 개발 실행의 `scripts/isaac/run_local45_fleet_pair.py`에 `--capture-views overview`를 지정하면 전체 공장 RGB만 수집한다. 기본값 `all`은 기존 overview·carter1·carter2의3시점이다. 선택값은 호스트 영수증과 SDK 미디어 명세에 기록한다.

운송 로직 개발에서는 `--operation-story off`로 여섯 단계의 매 물리 프레임 연속 촬영만 끌 수 있다. 기본값은 `on`이다. `off`에서도 물리·제어·원장·관측, 1200-step 간헐 RGB와 자재 revision 기록은 유지된다. 요청값과 실제 활성 여부는 호스트·장면 영수증에 남으며, 꺼진 촬영은 `not_collected`로 표시한다. 대표 운영 영상에 필요한 여섯 구간의 촬영을 완료한 것으로 세지 않는다. 운송 흐름을 확인한 뒤 `on`으로 별도 촬영한다.

overview 옵션은 추적 카메라 prim의 보정 정보를 남기되 해당 render product와 RGB annotator를 생성하지 않는다. 프레임별 활성/비활성 전환은 하지 않는다. 모든 차량의 같은 프레임 GT는 평가·미디어용으로 기록하며, 물리·센서·주행 허가·촬영 시각의 동결 검사는 유지한다. 기본3시점을 요구하는 기존 정식 검증에1시점 자료를 대신 제출하지 않는다. Replicator 합성 데이터의 modality·시점 수집 프로필도 이 옵션과 별도다.

관련 CPU 검사9개와 SDK Python3.10.15의5파일 구문 확인을 통과했다.054는1시점 옵션으로 실행 중이며 속도 개선을 확정한 비교는 없다. [공식 Replicator1.11.35 API](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.11.35/source/extensions/omni.replicator.core/docs/API.html) · [Isaac Sim4.5 렌더 제품 수명 관리 예제](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/tutorial_replicator_ur10_palletizing.html)

운송 구간의 시작은 화물·빈 토트의 운반 차량 ID와 같은 차량의 새 주행 허가·명령·엔코더 움직임을 대조한다. 다른 차량의 움직임으로 시작하지 않는다. 내보내기에서는 선택한 원본 구간의 해당 운반 차량 GT로 이동·회전을 확인한다. 이 GT는 미디어 평가용이며 주행에 넣지 않는다. 움직임이 없는 운송 구간을 대표 영상으로 내보내지 않고 원본과 사유를 보존한다. 출하 집계·공정·순환 완료의 정지 장면은 이 운송 조건과 구분한다.

새 실행의 출하 이후 창은 두 차량이 실제로 움직일 때 시작한다. 내보내기도 선택한 원본에서 두 차량의 이동·회전을 요구하고 화면에는 `출하 이후 · 병행 물류`로 표시한다. 기존6개 창 중 하나의 선택 시점을 옮기는 것으로 추가 촬영 창이나 수집량을 늘리지 않는다. 과거 실행의 첫 출하 집계 장면을 병행 물류로 재분류하지 않는다.

후속 normal3 프로필은 창당180물리 프레임·총1,080장으로 수집한다. 기존 내보내기가 각 창에서 첫180장만 선택하므로, 사용하지 않던 뒤60장을 수집 단계에서 제외한다. 기존6×240의 상한 안에서 줄이며 원본60Hz·GIF40fps·MP460fps와 운반 차량/병행 이동 판정은 유지한다. 실행 중054와 과거240프레임 자료는 동결 프로필대로 보존하고 내보내기가 두 프로필을 구분한다. 새 수집량의 실제 시간·용량 절감은 미측정이다.

## 4대 물리 캐리어 개발 옵션

새로 준비한 로컬4.5·PhysX planar 개발 실행에서 호스트와 SDK 실행기에 `--carrier-modules`를 지정하면 차체에 고정하는 물리 운반 모듈을 작성한다. 기본값은 꺼짐이며 진행 중인 실행에는 추가하지 않는다. 입력은 동결 소스에 포함한 `configs/vehicles/tote-carrier-carter-v2.json`으로 고정한다. 호스트가 소스·프로필의 archive/pin 해시를 확인한다.

SDK의 `after_robots_loaded` 단계에서 네 차량 각각의 실제 차체 변환·rigid body·collision geometry·센서 후보 위치를 측정한다. 캐시된 authored extent 대신 geometry-computed extent를 사용하고 기존 반경·장착 여유를 검사한다. 네 대 모두 통과하고 새 prim 경로가 비어 있어야 별도 dynamic carrier body·질량/관성·compound collider·차체 FixedJoint를 작성한다. 작성 직전 측정을 다시 대조하고 작성 후 속성 readback도 확인한다. 전역 collision mask·주행 한도·지도 반경은 바꾸지 않는다.

제품·빈 토트 표시는 차체 좌표계의 모듈 상판 높이에 맞춘다. 이 표시는 충돌·질량이 없는 시각 모델이며 접촉에 의한 운반이나 자동 하역을 뜻하지 않는다. 옵션의4대 실제 SDK 운송 실행·센서 FOV·물리 토트 인계는 아직 확인하지 않았다. CPU 계획5검사와 scene 연결/시각 위치4검사, SDK Python3.10.15의4파일 구문 확인을 통과했다. 새 SimulationApp은 기존 실행 종료 후 사용한다.

기준 구현은 설치된 SDK4.5의 Robot Assembler 소스와 기존 [단일 차량 적재 주행 도구](../scripts/isaac/probe_loaded_carrier.py)다. 외부 고정 조인트의 `excludeFromArticulation`과 연결된 두 body의 collision 설정을 재사용하며, 산업용 하중 정격·기계 강도는 검증 범위에 넣지 않는다. [설치 소스·프로필·검사 범위](evidence/carrier-fleet-adapter-177.json)

## Kit 운영 패널

기존 SDK의 Kit 확장 검색 경로에 저장소의 `extensions/`를 추가하고 `factory.ops.console`을 활성화한다. 기본 API 주소는 `http://127.0.0.1:8765`이며, 실행 전 `FACTORY_OPS_API_URL`로 다른 loopback 주소를 지정할 수 있다. 외부 HTTP 주소·리다이렉트·프록시는 허용하지 않는다.

관측된 AMR 목록·작업·목적지·속도·SOC 원천·적재물을 표시하며, 지원 capability가 있는 CPU 모델에서 주문 생성과 전체 일시정지·재개를 제공한다. 실행 ID·revision·명령 ID를 결속하고 접수와 적용 결과를 구분한다. 연결 지연이나 실행 변경을 감지하면 조작을 비활성화한다.

[실제 SDK4.5 시험 화면](evidence/kit-console-004/kit-console.png)과 [명령·촬영·종료 근거](evidence/kit-console-004/summary.json)는 CPU 원장 연결 시험이다. 같은 Kit 클라이언트의 네이티브 API 정지·재개는054에서 확인했고, Kit 패널 화면의 실제 ROS 연결은 남아 있다.

### 다음 예약 지점에서 정지·재개

로컬4.5 개발 실행에는 선택형 정지·재개 경로가 있다. 새 실행을 시작할 때 실행기와 API 양쪽에 `--live-anchor-controls`를 지정하고, API의 `--commands-dir`을 그 실행의 `results/run/commands`에 연결한다. 원천이4대·개발 프로필·동일 실행 ID·지원 capability를 발행해야 조작이 활성화된다. 이미 진행 중인 관측 전용 실행은 이 옵션으로 전환하지 않는다.

정지 요청은 `pending`으로 접수된다. 네 차량이 각자의 예약 지점에 도착하고 새 HOLD 응답과 연속 정지 관측을 충족하면 원천이 `applied`를 발행한다. 물리·ROS·가상 에너지 시계는 계속 흐른다. 재개는 이 상태에서 새 명령 ID와 현재 revision으로 요청한다.

WSL ROS를 조작할 때 API도 같은 WSL에서 실행한다. API가 큐에 넣는 `requested_at`과 ROS 수신기의 wall 시계가 같은 OS에 속하도록 구성한다. 관측 전용 Windows API는 계속 지원한다. 다음 예제의 `<시도>`는 사용 중인 동결 실행 폴더로 바꾼다.

```powershell
cd "$HOME/factory-ops-twin"
$stage = Join-Path $HOME "FactoryOpsRuntime/local45-transport-<시도>"
$linuxStage = (wsl.exe --distribution Ubuntu --exec /usr/bin/wslpath -a $stage).Trim()
wsl.exe --distribution Ubuntu --cd "$linuxStage/source" --exec /usr/bin/python3 -B -m factory_ops.dashboard `
  --config "$linuxStage/source/configs/factory-fleet4-normal3" --port 8766 `
  --source "$linuxStage/results/run/dashboard-state.json" `
  --charging-source "$linuxStage/results/run/charging-state.json" `
  --events-source "$linuxStage/results/run/actions.jsonl" `
  --commands-dir "$linuxStage/results/run/commands" --live-anchor-controls
```

웹은 별도 PowerShell에서 같은 API를 선택한다.

```powershell
cd "$HOME/factory-ops-twin/dashboard"
$env:OPS_API_URL = "http://127.0.0.1:8766"
node node_modules/next/dist/bin/next dev --hostname 127.0.0.1 --port 3007
```

아래 도구는 배정된 작업에서 차량의 엔코더 이동을 관측한 뒤 Kit 패널과 같은 `KitClient`로 정지·재개를 한 번씩 요청하고 원천의 적용 응답을 기록한다. `<실행 ID>`는 해당 실행의 ID로 바꾸고 새 출력 파일을 사용한다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python scripts/demo/native_anchor_flow.py --api http://127.0.0.1:8766 --expected-run-id "<실행 ID>" --output artifacts/native-anchor-flow.json
```

정지 적용 후 관측 신선도가 잠시 떨어지면 새 정지 증거와 resume capability를 기다린다. 도구가 종료된 뒤에는 원천에서 같은 정지 요청의 `applied`를 확인하고, 그 ID로 재개만 복구할 수 있다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python scripts/demo/native_anchor_flow.py --api http://127.0.0.1:8766 --expected-run-id "<실행 ID>" --resume-from-pause "<적용된 정지 명령 ID>" --output artifacts/native-anchor-resume.json
```

타임아웃·연결 오류 때 명령을 재전송하거나 실행을 종료하지 않는다. 마지막 적용 상태와 원천의 현재 정지 관측을 확인한다. [054의 실제4대 정지·재개 기록](evidence/local45-native-control-054.json)은 같은 실행의 `actions.jsonl`에 있는 `live_control_applied.anchor_evidence`와 요청·적용 event를 결속한다. 이는 네이티브 API의 제어 근거이며 브라우저 클릭·Kit 패널 화면·전체 물류 완주의 확인은 남아 있다.

## 모의 MES 입력

CPU API의 `POST /api/mes/events`는 모의 주문·재고 입고·설비 관측·heartbeat를 받는다. 같은 이벤트의 재전송을 중복 적용하지 않으며 다른 실행·오래된 시각·역행한 순번의 입력을 거부한다. 설비 관측은 정보로 표시한다. 실제 공장의 MES·설비 제어 연결은 아니다.

아래 예제는 실행 중인 CPU 모델에 입고 6개와 수량 2인 주문 한 건을 추가한다. 결과 파일이 이미 있으면 새 출력 경로를 사용한다. live/replay 원천에는 모의 MES 입력을 적용하지 않는다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python scripts/demo/mock_mes_flow.py --output artifacts/mes-flow-example.json
```

[HTTP 입력 기록](evidence/mock-mes-024/http-flow.json) · [웹 명령 기록](evidence/mock-mes-024/web-control.json) · [화면](evidence/mock-mes-024/web-control.jpg)

## CPU 보고서와 원장 재생

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops prepare --output runs/local
uv run python -m factory_ops verify runs/local
uv run python -m factory_ops replay runs/local/fifo/events.jsonl --expected runs/local/fifo/snapshot.json
uv run python -m factory_ops prepare-g3 --output runs/g3a
uv run python -m factory_ops verify-g3 runs/g3a
```

새 출력 폴더를 사용한다. `runs/local/report.html`은 배치·정책별 CPU 기록, `runs/g3a/report.html`은 경로 후보·정상/폐쇄·예약 타일을 보여준다. 준비 명령은 클라우드 인스턴스를 생성하거나 실물 로봇에 명령하지 않는다.

CPU 모델의 충돌 수·Nav2 성공률·RTF·VRAM은 `null`, `primary_evaluation=false`다. `verify`는 파일 해시, `replay`는 이벤트 본문·순서·버전과 재생 결과를 검사하며 발행자 인증을 제공하지 않는다. GPU 물리 실행과 비교 평가의 절차·제약은 [GPU 연동 안내](gpu-handoff.md), [설계](architecture.md), [완성 계획](completion-plan.md)을 따른다.
