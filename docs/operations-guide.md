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

## 실제 ROS 실행의 읽기 전용 관측

ROS 임무의 `--dashboard` 발행기가 같은 실행의 `dashboard-state.json`을 기록한다. API의 `--source`로 이 파일을 읽고, 가상 충전과 운영 사건은 같은 실행의 `charging-state.json`, `actions.jsonl`로 연결한다. `--config`는 시뮬레이션과 같은 배치를 선택한다. 아래는4대 정상 운송 배치의 예이며 `<실행명>`은 관측할 실행 폴더 이름으로 바꾼다.

```powershell
cd "$HOME/factory-ops-twin"
$taskRunRoot = Join-Path $HOME 'FactoryOpsRuntime/<실행명>/results/run'
uv run python -m factory_ops.dashboard --config configs/factory-fleet4-normal3 --source "$taskRunRoot/dashboard-state.json" --charging-source "$taskRunRoot/charging-state.json" --events-source "$taskRunRoot/actions.jsonl"
```

`map→base_link` 추정 위치·엔코더 속도·작업 원장을 표시한다. 실제 ROS 원천은 관측 전용이며 웹에서 차량 주행·일시정지·주문을 명령하지 않는다. 관측이 오래되면 stale로 표시하고 원천에 없는 SOC·충전 대기열은 추정해서 채우지 않는다. 웹 임무 경과 시각과 사건의 ROS 원본 시각을 구분한다. 웹 관측을 종료 결과나 학습 데이터 승인으로 사용하지 않는다.

실행 중 bag SQLite에는 조회나 복사용 연결을 열지 않는다. 관측은 JSON/JSONL을 사용하며 bag 분석은 기록기·소유 실행 종료를 확인한 뒤 수행한다.

[실행051 연결 기록](evidence/local45-live-web-158.json)은 실행 ID·4대 차량·배치 revision이 원천과 일치하고 Next.js 프록시까지 전달됨을 확인한 자료다. 이 실행의 API는 관측 전용이다. 차량 선택 클릭과 실제 정지·재개의 브라우저 통합 완료는 별도 확인이 남아 있다.

## 공장 운영 미디어 촬영

새 로컬4.5 개발 실행의 `scripts/isaac/run_local45_fleet_pair.py`에 `--capture-views overview`를 지정하면 전체 공장 RGB만 수집한다. 기본값 `all`은 기존 overview·carter1·carter2의3시점이다. 선택값은 호스트 영수증과 SDK 미디어 명세에 기록한다.

overview 옵션은 추적 카메라 prim의 보정 정보를 남기되 해당 render product와 RGB annotator를 생성하지 않는다. 프레임별 활성/비활성 전환은 하지 않는다. 모든 차량의 같은 프레임 GT는 평가·미디어용으로 기록하며, 물리·센서·주행 허가·촬영 시각의 동결 검사는 유지한다. 기본3시점을 요구하는 기존 정식 검증에1시점 자료를 대신 제출하지 않는다. Replicator 합성 데이터의 modality·시점 수집 프로필도 이 옵션과 별도다.

관련 CPU 검사9개와 SDK Python3.10.15의5파일 구문 확인을 통과했다. 실제 GPU에서의1시점 실행과 속도 개선은 아직 확인하지 않았다. 실행 중인051은 기존3시점을 유지한다. [공식 Replicator1.11.35 API](https://docs.omniverse.nvidia.com/kit/docs/omni_replicator/1.11.35/source/extensions/omni.replicator.core/docs/API.html) · [Isaac Sim4.5 렌더 제품 수명 관리 예제](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/replicator_tutorials/tutorial_replicator_ur10_palletizing.html)

운송 구간의 시작은 화물·빈 토트의 운반 차량 ID와 같은 차량의 새 주행 허가·명령·엔코더 움직임을 대조한다. 다른 차량의 움직임으로 시작하지 않는다. 내보내기에서는 선택한 원본 구간의 해당 운반 차량 GT로 이동·회전을 확인한다. 이 GT는 미디어 평가용이며 주행에 넣지 않는다. 움직임이 없는 운송 구간을 대표 영상으로 내보내지 않고 원본과 사유를 보존한다. 출하 집계·공정·순환 완료의 정지 장면은 이 운송 조건과 구분한다.

## Kit 운영 패널

기존 SDK의 Kit 확장 검색 경로에 저장소의 `extensions/`를 추가하고 `factory.ops.console`을 활성화한다. 기본 API 주소는 `http://127.0.0.1:8765`이며, 실행 전 `FACTORY_OPS_API_URL`로 다른 loopback 주소를 지정할 수 있다. 외부 HTTP 주소·리다이렉트·프록시는 허용하지 않는다.

관측된 AMR 목록·작업·목적지·속도·SOC 원천·적재물을 표시하며, 지원 capability가 있는 CPU 모델에서 주문 생성과 전체 일시정지·재개를 제공한다. 실행 ID·revision·명령 ID를 결속하고 접수와 적용 결과를 구분한다. 연결 지연이나 실행 변경을 감지하면 조작을 비활성화한다.

[실제 SDK4.5 시험 화면](evidence/kit-console-004/kit-console.png)과 [명령·촬영·종료 근거](evidence/kit-console-004/summary.json)는 CPU 원장 연결 시험이다. 실제 ROS 운송을 조작하는 통합은 남아 있다.

### 다음 예약 지점에서 정지·재개

로컬4.5 개발 실행에는 선택형 정지·재개 경로가 있다. 새 실행을 시작할 때 실행기와 API 양쪽에 `--live-anchor-controls`를 지정하고, API의 `--commands-dir`을 그 실행의 `results/run/commands`에 연결한다. 원천이4대·개발 프로필·동일 실행 ID·지원 capability를 발행해야 조작이 활성화된다. 이미 진행 중인 관측 전용 실행은 이 옵션으로 전환하지 않는다.

정지 요청은 `pending`으로 접수된다. 네 차량이 각자의 예약 지점에 도착하고 새 HOLD 응답과 연속 정지 관측을 충족하면 원천이 `applied`를 발행한다. 물리·ROS·가상 에너지 시계는 계속 흐른다. 재개는 이 상태에서 새 명령 ID와 현재 revision으로 요청한다.

아래 도구는 작업 중인 차량의 엔코더 속도를 관측한 뒤 Kit 패널과 같은 `KitClient`로 정지·재개를 한 번씩 요청하고 원천의 적용 응답을 기록한다. 초기 주차 상태를 운송 중 정지 시연으로 사용하지 않는다. `<실행 ID>`는 해당 새 실행의 ID로 바꾸고 새 출력 파일을 사용한다. 현재 관측 전용 실행051에는 명령을 보내지 않는다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python scripts/demo/native_anchor_flow.py --expected-run-id "<실행 ID>" --output artifacts/native-anchor-flow.json
```

타임아웃·연결 오류 때 명령을 재전송하거나 실행을 종료하지 않는다. 마지막 적용 상태를 관제에서 확인하고 기존 실행 감독자가 자원을 정리한다. 결과는 API 응답의 근거이며 브라우저 클릭이나 전체 물류 완주의 근거는 아니다. 네 차량의 세부 정지 자료는 같은 실행의 `actions.jsonl`에 있는 `live_control_applied.anchor_evidence`와 함께 확인한다. CPU 프로토콜 검사는 통과했으며 실제4대 정지·재개 실행은 남아 있다.

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
