# 운영 화면과 재현 실행 안내

대표 이미지·영상과 현재 구간은 [README](../README.md), 전체 완료 조건은 [완성 계획](completion-plan.md)에 있다. 아래 CPU 모델은 실제 Isaac Sim·Nav2 주행과 별도 결과다. 이미 API·웹 서버가 실행 중이면 같은 포트에 중복 실행하지 않는다.

## CPU 운영 모드

Python 3.12·uv를 사용하며 Python 런타임 의존 패키지는 없다. PowerShell에서 API를 실행한다.

```powershell
cd "$HOME/factory-ops-twin"
uv sync --locked
uv run python -m factory_ops.dashboard --config configs/factory-semiconductor-dual-lane --execution-mode operations --dispatch-policy eta --scenario-dir runs/operator-scenarios
```

이 모드는4대의 가감속·회전·구간 예약·점유·정지 앵커 우회와 가상 에너지·충전을 계산한다. 초기 주문4건을 배정하며 추가 주문은 관제에서 만든다. 운영 패널의 설비 고장·복구는 CELL1의 남은 처리시간을 보존한다. 계획 통로 변경은 승인된 운송이 통로를 비운 뒤 적용되므로 요청 접수와 적용 완료를 구분한다. 웹과 Kit은 같은 API의 실행 ID·정책·시각·명령 receipt를 읽는다.

기존 경로 보간 미리보기와 별도 충전 모델은 다음 명령으로 실행한다. 운영 모드의 교통 결과와 합치지 않는다.

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

주소는 `http://127.0.0.1:3007`, 기본 API는 `http://127.0.0.1:8765`다. 웹은 수신 표본 사이의 표시 좌표를 보간한다. 운영 모델의 계산과 화면 보간은 별개다. 가상 충전은 작성된 에너지 계수·충전대 예약·정렬·대기·SOC 증가·이탈을 사용하며 물리 접촉 센서나 전기 충전 결과가 아니다.

## 16대 CPU 배치와 진단 종료

전체 운영의 유한 수요·출하·반납·최종 정지를 확인하는 배치는 다음처럼 별도 출력 경로에 실행한다. 설정과 생산자 소스를 시작할 때 동결하며 GPU나 RGB를 사용하지 않는다. 아래 지평·벽시계 값은 실행 상한이다.

```powershell
cd "$HOME/factory-ops-twin"
$pilotLabel = Get-Date -Format "yyyyMMdd-HHmmss"
uv run python scripts/demo/run_process_line.py --config configs/factory-semiconductor-campus16 --output-dir "artifacts/operations/campus-$pilotLabel" --lots 48 --horizon 120000 --max-wall-seconds 5400 --cycle-stall-timeout 600
```

배차 후보를 선택할 때는 위 명령의 설정을 `configs/factory-semiconductor-campus16-coverage-eta`로 바꾸고 `--policy eta`를 명시한다. 이 설정은 실제 완료 운송2회 미만의 가용 차량을 먼저 고른 뒤 명목 ETA를 우선한다. 전 차량이2회를 채우면 ETA가 배정 수보다 앞선다. `fifo`와 함께 쓰면 거절하며, 기존 설정은 원래 배정 수 우선 정책을 유지한다. 다른7개 설정 파일은 기준선과 바이트가 같고 process-network는 배차 모드·반복 완료 기준 외 값이 같다. 가감속·회전·예약·점유·에너지·버퍼를 완화하지 않는다. 표적 검사와 main의 실제 snapshot 계약 확인은 통과했지만 전체 비교 실행은 아직 없다. [설정 해시·확인 범위](evidence/campus16-coverage-eta.json)

`--stall-timeout`은 이동·공정·충전 진전이 모두 없는 시뮬레이션 시간이며 기본600초다. 별도 `--cycle-stall-timeout`은 기본0으로 꺼져 있다. 켜면 같은 순환 대기 구성원의 pose·task·payload·완료 운송·누적 충전에 변화가 없는 시간을 센다. 다른 차량의 이동은 이 시계를 리셋하지 않는다. 기준에 도달하면 `incomplete / stationary_wait_cycle`과 최종 상태를 기록하며, 교통 문제를 해결했다고 판정하지 않는다.

실행 중에는 출력 경로의 `progress.json`만 읽는다.30시뮬레이션 초마다 교체하는 단일 파일이며 `operations.robots`에는 현재 pose·속도·작업·대기 이유·배터리·충전 상태, `operations.equipment_cells`에는12개 설비 WIP, `operations.charging`에는 도크 예약·접근/이탈 보류 이유가 들어간다. 경로 전체·이벤트 이력·촬영 자료는 복제하지 않는다. 새 상태 요약은 새 실행부터 발행되므로 구버전 실행에서 필드 부재를 빈 작업 상태로 해석하지 않는다. 원인 확인을 위해 진행 중인 gzip·SQLite를 열지 않는다.

Ctrl+C는 `incomplete / diagnostic_interrupt`로 마감하고 가능한 최종 snapshot·결과·닫힌 촬영 스트림·매니페스트를 남긴다. 실제 owned 터미널에서 이 경로와5프레임 보존·해시 일치·소유 Python0을 확인했다. 중단된 기록은 최종 미디어 입력으로 사용하지 않는다. 촬영용1Hz 기록은 `--record-start <시뮬레이션 초> --record-duration <최대300초>`로 선택하며, 원본 종료 뒤 내용과 해시를 확인한다. [표적 검사·중단 실행 범위](evidence/process-line-runner-lifecycle.json)

대표 미디어의 시점을 미리 정하기 어려우면 배치 명령에 `--record-best-window`를 추가한다.1Hz 관측의 최근300개와 최고 후보300개만 메모리에 보관하고 마감 때 선택한 창 하나를 저장한다. 출하·반납의 동시 증가, 두 증가량의 최솟값, 완료 운송, 같은 물류 task의 pose 이동 거리·참여 차량 순으로 고른다. 주차·충전과 `movement_purpose=traffic_yield`, task ID 전환 구간의 위치 차이는 이동 점수에서 제외한다. payload 없는 실제 집하 접근은 포함한다. 고정 `--record-duration`과 함께 사용할 수 없고 이 모드에서는 `--record-start`를 사용하지 않는다.300표본보다 일찍 끝나면 가용 표본만 저장하며 채워 넣지 않는다. 상태 파일·결과·기록 매니페스트의 `recording_selection`/`selection`에 시각·누계·선택 점수가 남는다. 선택 점수는 영상 편집용이며 전체 운영의 유효성이나 물리 이동 성공을 판정하지 않는다. 최종 렌더 입력은 별도로 완주·소스 안정·해시를 확인한다. [표적 검사·실제 소량 연결 범위](evidence/operations-clip-selection.json)

마감된 기록의 최종 발췌는 `scripts/media/select_operations_excerpt.py --recording <종료된 기록 폴더> --output <새 JSON>`으로 고른다. 기본 계약은16대·48lot이며 전체 출하/반납·최종 정지·차량별2회 이상 운송·16개 물류 임무 동시성·소스 안정이 모두 충족돼야 한다. 종료 매니페스트가 없는 기록은 gzip을 열기 전에 거절한다. 통과한 기록도 원본·동결 소스·압축 전후 해시를 확인한 뒤, 기본30초·30fps·8배속의900프레임에 맞는 후보5개를 고른다. 배치와 같은 물류 점수를 사용하고 출하/반납이 함께 증가하는 후보인지 출력한다. 이 도구는 GPU 렌더를 시작하지 않는다. 현재16대 완주 입력은 확인 중이다.

시각 소품 배치만 고쳤을 때는 운영 기록을 수정하지 않고 renderer의 `--factory`에 새 시각 설정을 지정한다. 기록의 layout SHA는 같아야 하며 나머지 운영·카메라 설정7개도 원본과 바이트를 대조한다. visual-assets는 원본/렌더 hash를 따로 남긴다.338에서 이 경로와 실제 모델 외곽의 통로·장애물·소품 간 여유를 확인했다. 최종 실행기는 완주48/48·정지·차량별2회·동시물류16개 미달을 GPU 시작 전에 거절하며, peak12인333의 최종 실행 거절도 확인했다. [시각 수정·입력 핀·실제 이미지](evidence/campus-complete-preview-338.json)

선택 차로 확장은 `--config configs/factory-semiconductor-campus16-wide-bridges --policy fifo`로 실행한다. AB/CD의 실제3m 차로와 방향별 예약을 사용하며 AC/BD·버퍼·재고·에너지·차체 반경은 유지한다. 지정 대기점 양보도 같은 모델에서 적용한다. 구성 요소 검사는 통과했지만330에서 도로와 독점 예약 영역의 잘못된 확장을 확인해 경계를 수정했다. 실제 반납 경로 회귀는 통과했다.331의 충전 도크와 적재 차량 순환 대기는 예약 도크의 경로 보호로 수정했으며 후속332는 예약 창 전환 오류로 종료돼 완주는 남아 있다. [기하와 확인 범위](evidence/campus16-wide-bridges.json) 기존 coverage ETA를 같은 차로에 결합하려면 `--config configs/factory-semiconductor-campus16-wide-bridges-coverage-eta --policy eta`를 사용한다. 운송2회 미만 차량의 참여를 우선하고 이후 명목 ETA 순으로 고른다.333은 이 설정으로48출하·48반납·최종 정지·전16대최소10운송을 완료했다. 동시 물류 최고12개여서 최종16대 미디어 입력은 아니다.332는 오류 종료됐으므로333과의 최종 수량 차이를 정책 우월성으로 해석하지 않으며 병렬 wall time도 처리속도 비교로 쓰지 않는다. [입력 동일성·진행 범위](evidence/campus16-333.json)

선택 정책 `service_window`는 자재 배정과 물리 인계점 예약을 분리한다. 다음 명령은 초기 WIP0·48lot·입력 버퍼2와 기존 안전/에너지 설정을 유지한다. 기본 정책의 배차 즉시 origin/destination 선점과 구분하며, 다른 설정에는 적용되지 않는다.

```powershell
cd "$HOME/factory-ops-twin"
$handoffLabel = Get-Date -Format "yyyyMMdd-HHmmss"
uv run python scripts/demo/run_process_line.py --config configs/factory-semiconductor-campus16-wide-bridges-handoff-admission --policy fifo --lots 48 --horizon 240000 --max-wall-seconds 10800 --stall-timeout 600 --cycle-stall-timeout 600 --record-best-window --output-dir "artifacts/operations/handoff-$handoffLabel"
```

lot·캐리어·목적지 버퍼 credit은 배차 때 확정한다. 안전 정지 위치의 차량은 첫 구간의 일시 충돌을 기다릴 수 있지만, 실제 bridge/tile 승인 전에는 이동하지 않는다. 서비스 goal을 포함한 마지막 정지 가능 window에서 다른 인계점 소유자를 확인하고, 통행 승인 후 이동 전에 gate를 확보한다. 적재/하역 확인과 차체 이탈 후 해제 조건은 유지한다. 새 행동8개·기존 영향8개 검사와 snapshot API 형태 확인을 통과했다.334는16개 동시 물류 배정을 기록했지만 이전 gate 잔류로 순환 대기 마감했다. 현재 목표 station을 구분하는 해제 수정과 실제 상태 표적1검사를 추가했으며, 수정본 전체 완주는335에서 판정한다. [확인 범위](evidence/campus16-service-window-admission.json)

## 16대 캠퍼스의 설비 고장·복구

`process-line` API의 기본값은 읽기 전용이다. 다음처럼 명령을 켠 로컬 실행에서는 관제의 운영 패널에서12개 설비 중 대상을 선택해 고장·복구를 요청한다. 기존 API와 포트가 겹치지 않는지 확인한다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops.dashboard --config configs/factory-semiconductor-campus16 --execution-mode process-line --lots 48 --port 18766 --process-line-controls --scenario-dir runs/campus-scenarios
```

웹은 `OPS_API_URL=http://127.0.0.1:18766`을 지정한 별도 포트에 연결한다. 명령은 현재 API가 계산하는 CPU 실행에만 적용되며 다른 운영 배치를 제어하지 않는다. API는 실행 ID·revision·`cell_id`를 확인하고 적용된 설비와 상태를 receipt의 `effect`로 반환한다. 기본 읽기 전용·외부 source·재생·Mock MES의 경계는 유지한다. 차량 조종·주문 추가·통로 변경은 지원하지 않는다. 고장·복구와12개 설비 상태는 저장한 관측에서 재현하며, 새 브라우저 클릭 확인은 도구 초기화 오류로 남아 있다.

### 설비 고장 비교 재현

다음 실행은48lot·9,000초의 정상 조건과 `L2.TEST` 300초 고장을 비교한다. 처리 중인 lot을 관측한 뒤 임시 loopback HTTP 서버로 고장·복구를 요청한다. 명령 receipt·경계 상태·대상 사건·최종 지표만 남기며 RGB나 전체 snapshot을 수집하지 않는다. 서버와 소유 스레드는 종료 시 정리한다.

```powershell
cd "$HOME/factory-ops-twin"
$comparisonLabel = Get-Date -Format "yyyyMMdd-HHmmss"
uv run python -m scripts.demo.compare_campus_equipment --output "artifacts/campus-equipment-comparison-$comparisonLabel"
```

벽시계 상한은1,200초이며 출력 폴더는 새 경로여야 한다. 다른 모델 실행과 병행할 때는 커밋한 소스의 별도 linked worktree에서 실행하고 소스·설정을 유지한다. 이미 마감한 정상 조건은 `--baseline-from <baseline.json 전체 경로>`로 재사용할 수 있지만 모델·설정 해시가 일치해야 한다.

[보존한 비교](evidence/campus-equipment-comparison.json)는 동결314 설정·모델과1340f8b 격리 실행을 사용했다. 첫 비교의 사건 ID 처리 오류와 두 번째 공유 실행의 소스 변경은 실패로 남기고, 소스가 일치한 마감 baseline과 격리 fault만 채택했다. 대상 `LOT-0004`의 TEST 완료가299.9초 지연됐고 고장 중 남은 시간이 보존됐다. 구간 종료 출하는 양쪽2건, 완료 운송은 정상40건·고장39건이다. 현재 main에서 새로 실행하면 후속 충전 수정이 포함되므로 이 수치의 동일 재현으로 취급하지 않는다.48lot 완주·전체 출하 감소·물리 운송 성능을 입증하는 비교는 아니다. [선별 경계 CSV](data/campus-equipment-comparison/boundaries.csv)

## 운영 기록 저장·재생

API에 `--scenario-dir`를 지정하면 웹에 기록 저장·재생 기능이 연결된다. CPU 모델의 예는 아래와 같다. 기존 API가 실행 중이면 새 프로세스를 중복으로 띄우지 않고 해당 실행의 설정에 옵션을 추가한다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops.dashboard --config configs/factory-semiconductor-dual-lane --execution-mode operations --scenario-dir runs/operator-scenarios
```

1. 웹에서 운영 상태를 확인한 뒤 기록 이름을 입력하고 `최근 관측 저장`을 누른다. API가 수신한 최근120개 이내의 관측만 저장한다. 버튼을 누르기 전에는 RAM에만 두며 같은 revision·시뮬레이션 시각의 중복 표본은 제외한다.
2. 저장된 기록을 선택하고 `기록 불러오기`를 누른다. 차량 선택·상세·작업·재고·충전 표시는 선택한 관측 시각으로 바뀐다.
3. 이전·다음 버튼과 슬라이더로 표본을 고르거나 자동 재생한다. 화면에 원래 시뮬레이션 시각과 표본 간격을 표시한다. 자동 재생은900ms마다 다음 관측을 보여주는 방식이며 원속도 영상이나 물리 재실행으로 사용하지 않는다.
4. `실시간으로 돌아가기`를 누르면 계속 수신하던 현재 운영 상태가 보인다. 재생 중에는 차량 명령과 새 기록 저장이 비활성화된다.

기록에는 원래 실행 ID·관측 시각·표본 해시·공장 배치 해시를 보존한다. 다른 배치나 손상된 파일의 재생은 거부한다. 한 파일의 상한은16MiB이며 기존 파일을 덮어쓰지 않는다. 이 기록은 관제 상태의 선택 보존이며 RGB·라벨 데이터셋·rosbag·물리 재현 증거와 별도로 관리한다. 실제 ROS 관측 API에도 같은 옵션을 사용할 수 있으며 저장소는 실행별 경로로 분리한다. 물리 실행의 동결 소스·원장·명령 경로는 바꾸지 않는다.

완료된 정상 운영의1980–2220초 구간에서 고른120개 원본 표본을 [공개 재생 패키지](data/operations-normal-223/README.md)로 제공한다. 압축 해제한 scenario를 위 저장소에 두면 같은 공장 배치의 웹에서 불러올 수 있다. 기록은 읽기 전용이다. 전체 실행은12회 출하·12회 반납·4대 주차를 완료했고, 선택 구간에서는 출하가1회·반납이2회 늘었다. 발췌 구간만으로 전체 완료를 입증하지 않는다.

[064의 실제4대 관측 기록](evidence/scenario-replay-198.json)은 Next.js 프록시로 저장·목록·처음/마지막 표본 재생을 확인했다. 원래 시각·실행·공장 배치·파일 해시와 빈 재생 명령 권한을 대조했고, 저장 뒤에도 live 원천이 유지됐다. 웹 타입 검사와 빌드는 코드 정합성 증거다. [실행070 화면](images/web-console-070/dashboard-live-carter1.jpg)은 기존 읽기 전용 관측이며, 최신 운영 명령·receipt·재생 조합의 새 브라우저 클릭 확인은 아직 수행하지 않았다. 이 자료를 전체 물류 완주의 증거로 사용하지 않는다.

### CPU 기록에서 GPU 재생 미디어까지

다음 순서는 운영 모델의 원본 snapshot 기록, 웹/Kit용 읽기 전용 scenario 가져오기, 공용 GPU lease 안의 Isaac Sim 재생, CPU 미디어 내보내기를 분리한다. 같은 PowerShell에서 순서대로 실행한다. 모든 출력 폴더는 실행 전에 없어야 하며 기존 원본을 덮어쓰지 않는다.

먼저 정상 ETA 운영을1Hz로 기록한다. 이 단계는 CPU에서 실행하며 이미지를 만들지 않는다.

```powershell
cd "$HOME/factory-ops-twin"
$runLabel = Get-Date -Format "yyyyMMdd-HHmmss"
$recording = "artifacts/operations/recording-$runLabel"
uv run python -m factory_ops.operations_recording `
  --config configs/factory-semiconductor-dual-lane --case normal --policy eta `
  --horizon 5400 --window-start 0 --window-duration 5400 --sample-hz 1 `
  --output-dir $recording
```

웹과 Kit에서 읽을 구간은 원본 snapshot 그대로 별도 scenario 저장소로 가져온다. 아래 예제는1980–2220초에서 최대120개를 고르며 보간하지 않는다.

```powershell
cd "$HOME/factory-ops-twin"
$scenarioImport = "artifacts/operations/replay-$runLabel-import"
uv run python scripts/demo/import_operations_recording.py `
  --recording $recording --config configs/factory-semiconductor-dual-lane `
  --start 1980 --end 2220 --output-dir $scenarioImport `
  --name "normal operations 1980-2220"
```

GPU 재생은 공용 `Use-GpuLease.ps1` 안에서 새 launcher를 실행한다. Isaac Sim과 준비된 에셋 캐시의 설치 위치는 현재 PC에 맞게 바꾼다. launcher는 입력 해시를 고정한 뒤 내부 Windows Job으로 SDK 자손을 소유하고, 종료 receipt에 남은 PID가 없어야 성공으로 판정한다.

```powershell
cd "$HOME/factory-ops-twin"
& "$HOME/CodexSignals/Get-GpuLeaseStatus.ps1"

$hostPython = (Resolve-Path ".venv/Scripts/python.exe").Path
$sdkPythonBat = "E:\IsaacSim\isaac-sim-4.5.0\python.bat"
$assetCache = "E:\FactoryOpsAssets\simready-context"
$recordingAbsolute = (Resolve-Path $recording).Path
$factoryAbsolute = (Resolve-Path "configs/factory-semiconductor-dual-lane").Path
$gpuOutput = "E:\factory-ops-runs\operations-render-$runLabel"

$launcherArguments = @(
  "scripts/isaac/run_operations_replay.py",
  "--sdk-python-bat", $sdkPythonBat,
  "--recording", $recordingAbsolute,
  "--factory", $factoryAbsolute,
  "--asset-cache", $assetCache,
  "--output", $gpuOutput,
  "--start", "1980", "--duration", "30", "--speed", "8",
  "--fps", "30", "--width", "1280", "--height", "720",
  "--subframes", "2", "--maximum-wall-time", "1500"
)
& "$HOME/CodexSignals/Use-GpuLease.ps1" `
  -Owner "factory-ops-replay-$runLabel" `
  -FilePath $hostPython -ArgumentList $launcherArguments -WaitTimeoutSeconds 1800
```

`run_operations_replay.py`는 현재 `--help`와 Python 구문 검사까지만 확인했다. 실행228은 같은 입력 고정·Windows Job 소유·receipt 검사를 사용한 동등한 수동 Job 경로로 만들었으며, 새 launcher의 실제 GPU 통과 근거로 소급하지 않는다. 실행228은 5,400개 1Hz snapshot 중1980–2220초의240초 구간을 pose 보간해900프레임·30fps·8배속의30초 영상으로 표시했다. 재생 동안 PhysX는 멈춰 있었고 접촉 물리 결과가 아니다. [228 내보내기 출처](images/operations-local45-228/provenance.json)

GPU 프로세스가 종료되고 receipt의 `remaining_owned_pids`가 빈 배열인 것을 확인한 뒤, MP4·GIF 내보내기는 CPU에서 실행한다. 이 단계에는 Pillow가 있는 Python과 PATH의 FFmpeg가 필요하다. 운영 모델 전용 uv 환경에는 Pillow가 포함되지 않는다. 아래 `python`은 현재 PC에서 Pillow가 설치된 인터프리터를 가리킨다.

```powershell
cd "$HOME/factory-ops-twin"
$mediaOutput = "artifacts/operations/export-$runLabel"
python scripts/media/export_operations_replay.py `
  --render-dir "$gpuOutput/render" `
  --host-receipt "$gpuOutput/host/receipt.json" `
  --output $mediaOutput --gif --gif-fps 25 --start-frame 0 --count 900
```

내보내기 도구는 renderer 결과·프레임 해시·run ID·Job 종료 receipt를 대조한다. HUD는 원본 metadata만 사용하고 새 측정값을 만들지 않는다. 완성 MP4를 입력으로960px·64색·25fps GIF를 순차 생성하며 전송용 GIF에만 노이즈 억제를 적용한다. GPU lease는 렌더 작업이 끝나면 반환하고 CPU 인코딩 동안 보유하지 않는다.

고해상도 촬영은 renderer의 크기를1920×1080으로 지정한다. 내보내기는 원본 장면 픽셀을 유지하고 차량 번호·HUD만 크기에 맞춰 합성하며 결과 MP4는1920×1290이다.1280·1920·2560·3840px 폭의16:9 원본을 지원하고 매니페스트와 실제 크기가 다르면 거부한다.720p 원본을 고해상도 촬영으로 표시하지 않는다. `--gif-width 1280`으로 더 큰 GIF를 선택할 수 있고 기본960px는 유지한다. 현재 대표228은 기존720p 촬영 결과이며 새16대 대표 영상은 완주 기록에서 교체한다. [원본 크기 보존·인코딩 확인 범위](evidence/replay-native-resolution.json)

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

[실행229의 실제 SDK4.5 화면](evidence/kit-operations-229/kit-console.png)과 [명령·촬영·종료 근거](evidence/kit-operations-229/summary.json)는 CPU 운영 API 연결 시험이다. pause, resume, 주문 생성, `CELL1` 고장, 복구의5개 명령이 같은 business run에 적용됐고 종료 뒤 소유 PID는0개였다. aisle close/open 버튼은229에서 실행하지 않았으며 CPU 검사 범위로 남는다. 화면 뒤 빈 viewport는 연결 시험 배경이므로 공장 전체 렌더로 해석하지 않는다. 같은 Kit 클라이언트의 네이티브 API 정지·재개는054에서 확인했고, Kit 패널 화면의 실제 ROS 연결은 별도 범위다.

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
