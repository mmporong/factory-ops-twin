# Factory Ops Twin

반도체 후공정의 검사·포장 구역에서 AMR의 자재 공급, 완제품 출하, 빈 토트 반납, 충전을 운영 원장과 관제 화면으로 연결하는 디지털 트윈 포트폴리오입니다. 4대 운영 비교를 기준선으로 보존하고, 4개 생산 베이를 연결한16대가48개 lot의 출하·빈 용기 회수·최종 정지를 완료했습니다. CPU 운영 비교와 Isaac Sim 물리 시험, 합성 데이터의 실사 전이 실험은 각각의 실행 근거로 구분합니다.

[![16대 AMR 공정 간 이송·출하·빈 용기 회수와 차량별 작업·SOC를 표시한 운영 재생](docs/images/campus16-final-339/operations-replay.gif)](docs/images/campus16-final-339/operations-replay.mp4)

같은 실행에서 **출하48·빈 용기 반납48·240회 운송·16대 최종 정지**를 완료했습니다. 각 차량은9~21회 운송했으며, 물류 임무 최고 동시 수는16개·실제 물류 이동 최고치는6대입니다.

영상은 완주335의 공정 간 이송·출하·빈 용기 회수 구간을 Isaac Sim4.5에서 재생한339입니다.30초·8배속이며, MP4는30fps·장면1920×1080+상태 패널, GIF는1280px·25fps·17.2MB입니다. 이미지를 누르면 MP4가 열립니다. 발췌 중 출하는5→6건, 반납은2→3건으로 늘어납니다. 차량 번호·작업·SOC와 물류 이동·대기·충전을 구분하고 최근12sim초의 같은 운송 작업을 색 선으로 표시합니다.

CPU의1Hz 원본 pose 사이를 표시용으로 보간했고 PhysX는 멈춰 있습니다. 영상의 궤적은 재생 위치이며 접촉 물리 운송이나 새 주행 측정값을 뜻하지 않습니다. [영상 출처·해시·구간 수치](docs/images/campus16-final-339/provenance.json) · [전체 운영 결과](docs/evidence/campus16-335.json) · [4대 기준선 영상](docs/images/operations-local45-228/operations-replay.mp4)

## 구현 범위

- **운영 모델:** 유한 재고와 WIP, 공정 처리, 출하, 빈 용기 회수, FIFO·ETA 배차, 구간 예약, 통로 폐쇄, 설비 고장·복구를 계산합니다.
- **16대 AMR:** 작성된 가감속·회전 한도와 정지 지점을 따라 움직이며, 충전 수요·대기열·패드 정렬·가상 충전·이탈을 운영 흐름에 포함합니다.
- **관제:** Next.js·React Three Fiber 화면에서 작업, 목적지, 적재물, 속도, SOC 원천, 예약 경로, 대기 이유, 재고와 사건을 함께 봅니다.
- **재현:** 같은 조건의 정책 비교, 읽기 전용 관측 재생, 명령 receipt, 원본 해시와 실행 ID를 보존합니다.
- **검증 경계:** CPU 운영, Isaac Sim 물리, ROS 관측, 웹 표시용 보간을 서로 다른 증거로 관리합니다.

[현재 범위와 완료 조건](docs/completion-plan.md) · [설계와 제어 경계](docs/architecture.md) · [공정 모델](docs/semiconductor-cell.md)

### 16대 연결 캠퍼스:48lot 운영 완료

| 4개 베이·16대의 운영 상태 | 검사 셀 근접 장면 |
|---|---|
| ![완주335 기록의 16대 작업·SOC·물류 궤적과 공정 상태를 표시한339 GPU 재생](docs/images/campus16-final-339/overview.png) | ![절차형 핸들러·테스터와 트레이 인계대의 실제 정적 GPU 렌더](docs/images/campus16-static-294/cell_process.png) |

왼쪽은 완주335의 **17,672초 시점**을 재생한339입니다. 그 시점의 출하5·반납2와16대의 위치·작업·SOC를 표시하며, 상단은 물류 임무·이동/회전·대기·충전을 구분합니다. 오른쪽은294의 정적 검사 셀 장면입니다.

82×76m에 시각 생산 설비76개, 제공처 SimReady 모델88개, 차량 홈16개와 가상 충전 도크8개를 구성했습니다. 서비스 대기점12개·교량 진입 대기점8개·전용 출구8개를 배치했고, 넓힌 차로에 걸린 소품 두 개를 옮겨 실제 모델 외곽의 통로·장애물·이웃 소품 검사를 통과했습니다. 생산 장비 본체는 공개 기능 자료를 참고한 절차형 모델입니다. [339 원본·화면 출처](docs/images/campus16-final-339/provenance.json) · [촬영·종료 근거](docs/evidence/campus16-final-339.json) · [배치 조건](docs/evidence/campus16-layout-292.json) · [교통 경로 배치도](docs/images/campus16-layout-292/campus-layout.png)

`factory-semiconductor-campus16`은 4개 TEST→AOI→PACK 라인을 베이 사이로 연결합니다. 유한 재고 240개와 캐리어 64개를 사용하고 출하 뒤 원래 저장소로 빈 용기를 돌려보냅니다. 입력·출력 버퍼와 설비 고장, 전역 배차, 가감속·회전, 구간 점유와 가상 충전을 같은 운영 모델에서 계산합니다. 공정 시간·배치는 작성 가정입니다. 출하 지점에서는 같은 운반구의 반납 작업을 이어받는 배차를 추가했습니다. 점유·용량·에너지 조건을 만족해야 하며,335는 이 로직을 포함해48lot을 완주했습니다. 개별 기능의 개선율은 분리해 측정하지 않았습니다. [연속 인계의 확인 범위](docs/evidence/campus16-station-handoff.json)

| 확인 범위 | 확보한 결과 | 범위·한계 |
|---|---|---|
| CPU 연속 물류335 | 출하48·반납48·총240운송·최종 정지,16대 모두9~21회 운송 | 작성 조건의 유한 배치 |
| 같은 실행의 동시성335 | 물류 임무 최고16개, 실제 물류 이동 최고6대 | 작업 배정·이동·대기·충전을 구분 |
| AB·CD 차로 확장 | 반대 방향 예약 창 비중첩·두 차량 진행·339 GPU 영상 | 차체·점유·정지 가드 유지 |
| 무적재 물리·센서307/312 | 16대 주행·정지,32개 ROS/RTX 라이다 토픽 | 7초 무적재 구성요소 시험 |
| 대표16대 미디어339 | 900 RGB·30fps MP4·25fps GIF·같은 작업의 최근 궤적 | 원본1Hz 표시용 보간·PhysX 정지 |

335는 빈 WIP와 유한 수요48lot에서 시작해 전량 출하·빈 용기 회수 후 정지했습니다. **240개 재고는192개 잔여+48개 소비로 보존됐고,64개 캐리어가 모두 가용 상태로 돌아왔습니다.**16대가 각각9~21회 실제 운송을 완료했습니다. 물류 임무가 최대16개 겹쳤으며 실제 물류 이동 최고치는6대입니다. 주차·충전·양보 이동은 물류 계수에 넣지 않았습니다. 작성한 공정 시간·배치의 CPU 유한 배치 결과로, 실공장 생산성이나16대의 동시 이동을 뜻하지 않습니다. [335 결과·마감 해시](docs/evidence/campus16-335.json) · [차량별 완료 운송](docs/data/campus16-335/completed-transports.csv)

AB·CD는 차로 중심 간격3m와 방향별 예약을 사용하며 AC·BD는 한 대씩 통과합니다. 차체 외곽·예약 충전 도크·정지 점유·안전 출구를 통행 승인에서 확인합니다. 정지 재계획은 보호 창을 원자적으로 전환하거나 불가능한 대안을 철회합니다. 전체 상태 이미지와 대표 영상339에는 소품 배치 수정을 반영했고, 운영 layout과 나머지7개 입력은335의 동결 원본과 같습니다. [차로·예약 범위](docs/evidence/campus16-wide-bridges.json)

선택 설정 `service_window`는 lot·캐리어·목적지 버퍼를 배차 때 예약하고, 물리 인계점은 접근 구간의 통행 허가 뒤 확보합니다. 임시 통로 충돌은 안전한 정지 위치에서 기다리며 허가 전 이동·적재를 차단합니다. 이전 인계점은 실제 차체가 이탈한 뒤 해제합니다.335는 이 설정으로 반복 운송·16개 물류 임무의 동시성·48개 출하/회수·최종 정지를 같은 실행에서 충족했습니다. [구현·검사](docs/evidence/campus16-service-window-admission.json) · [실행 방법](docs/operations-guide.md#16대-cpu-배치와-진단-종료)

<details>
<summary>이전16대 재생 연결 미리보기</summary>

![16대 차량의 상태와 공정 셀·재고를 표시한 재생 미리보기](docs/images/campus16-replay-285/preview.png)

GPU 재생에16대 모델과 적재 상태를 연결했고, 각 차량의 SOC·임무와12개 셀의 상태를 표시합니다. 위 이미지는 부분 운영279를3프레임으로 재생한284의 RGB에 같은 기록의 정보를 합성한 미리보기입니다. 물리 시간은 멈춰 있습니다. 현재 대표339는 상단의335 완주 기록을 사용합니다. [재생 근거](docs/evidence/campus-replay-284.json) · [화면 출처](docs/images/campus16-replay-285/provenance.json)

</details>

미디어 내보내기는720p부터4K까지16:9 장면 크기를 유지하고, 차량 번호·상태·최근 물류 궤적을 합성합니다.339는1920×1080 원본900프레임을 사용하며 GIF는1280px·25fps입니다. 최종 원본·운영300표본·재현 소스·종료 영수증·미디어를 선별해 별도 드라이브에 백업했고984개 파일의 크기와 해시를 대조했습니다. [촬영·미디어 근거](docs/evidence/campus16-final-339.json) · [보존 위치·백업 해시](docs/evidence/campus16-final-storage-339.json) · [영상 발췌·실행법](docs/operations-guide.md#16대-cpu-배치와-진단-종료)

<details>
<summary>16대 운영 정책·교통·충전과 재현 방법</summary>

**물리·센서 시험:** 로컬 RTX3060·Isaac Sim4.5에서 Carter16대가7초·420physics step 동안 약0.58m 이동한 뒤 정지했습니다. 차량별 바퀴 조인트와32개 ROS/RTX 라이다 토픽에서 증가하는 시각과 유효 메시지14~15개를 확인했으며, 최대 GPU 메모리는6,536/12,288MiB였습니다. 짧은 무적재 구성요소 시험으로 장시간 물류·Nav2·적재 접촉의 결과와 구분합니다. [307·312 실행 근거](docs/evidence/campus-capacity-307-312.json) · [시험 종료 시 일부 구역](docs/images/campus-capacity-312/overview.png)

**배차와 인계:** 기본 정책은 반납·출하·후단 WIP를 우선하고, 운송 배정이 적은 차량을 선택합니다.335의 `service_window` 설정은 lot·캐리어·목적지 버퍼의 예약과 물리 인계점의 접근 승인을 분리합니다. 출하 지점에서는 같은 운반구의 반납을 연속 배차할 수 있지만 용량·점유·에너지 조건을 모두 만족해야 합니다. 선택 정책 `downstream_wip_drain_eta_after_coverage`는 실제 완료 운송2회 미만의 차량을 우선한 뒤 명목 ETA를 사용합니다. 이 정책의333은48개 출하·회수와 최종 정지를 완료했으며, 동시 물류 최고치는12개입니다. 단일 실행의 결과를 정책의 통계적 우월성으로 해석하지 않습니다. [인계점 예약](docs/evidence/campus16-service-window-admission.json) · [연속 배차](docs/evidence/campus16-station-handoff.json) · [선택 배차 정책](docs/evidence/campus16-coverage-eta.json)

**통행과 충전:** 실제 차체 외곽과 정지 점유를 포함한 구간 예약, 교량 출구 확인, 가감속·회전 한도를 적용합니다. 넓힌 AB·CD 차로는 방향별로 예약하며 AC·BD는 한 대씩 통과합니다. 빈 집하·귀가 차량은 안전한 대피점으로 양보한 뒤 원래 임무를 재개합니다. 충전은 접근 경로·대기 에너지·패드 소유권·가상 충전·이탈을 계산하고, 홈 소유권을 교환하면 귀가 목적지도 갱신합니다. 주차·충전·양보 이동은 완료 물류 운송이나 물류 동시 수에 넣지 않습니다. [차로와 예약](docs/evidence/campus16-wide-bridges.json) · [대피 범위](docs/evidence/campus16-home-parking-yield.json) · [충전 에너지](docs/evidence/campus16-charge-margin-320.json)

**관측과 보존:** 상태 파일에서16대의 작업·속도·대기·SOC와12개 논리 공정 셀의 WIP를 확인합니다. `--record-best-window`는 전체 실행 중 출하·반납·물류 이동을 기준으로 최대300개 표본을 선별합니다. 고정 순환 대기를 감시하며, 종료 때 최종 상태·결과·기록 매니페스트·소스 해시를 마감합니다. 미완료 기록은 진단용으로만 보존합니다. [상태 출력·종료](docs/evidence/process-line-runner-lifecycle.json) · [구간 선별](docs/evidence/operations-clip-selection.json) · [오류와 수정 기록](docs/troubleshooting.md)

```powershell
cd "$HOME/factory-ops-twin"
uv run python scripts/demo/run_process_line.py --config configs/factory-semiconductor-campus16-wide-bridges-handoff-admission --policy fifo --lots 48 --horizon 240000 --max-wall-seconds 10800 --stall-timeout 600 --cycle-stall-timeout 600 --record-best-window --output-dir artifacts/operations/my-campus16-pilot
```

출력 폴더는 새 경로를 사용합니다. 위 값은 실행 상한이며 예상 소요 시간이 아닙니다. CPU 운영 기록으로 접촉 물리·센서를 검증하지 않습니다. [전체 실행 인자와 영상 발췌](docs/operations-guide.md#16대-cpu-배치와-진단-종료)

**Brev:** 기존 Nebius2×L40S 환경은 재사용 당시 `UNHEALTHY`여서 SDK 실행 전에 중단했고, 이후 실제 `STOPPED`를 확인했습니다. 기존 클라우드는 중지 상태로 보존하며 현재16대 CPU 운영과 위 물리·센서 시험은 로컬 결과입니다. `scripts/brev/reuse_scale.py`의 기본 동작은 사전 확인이고, 첫 클라우드 묶음 상한은6시간·$40입니다. 실제 정산 비용은 미확인입니다. [재사용 실패·중지 근거](docs/evidence/brev-scale-reuse-239.json)

</details>

## 반도체 후공정 장면

| 36×28m 전체 구역 | 검사 셀 근접 장면 |
|---|---|
| ![이중 차로와 생산 설비를 배치한 반도체 후공정 구역](docs/images/semiconductor-dual-local45-213/overview.png) | ![테스트 헤드와 상부 서비스 설비가 있는 검사 셀](docs/images/semiconductor-dual-local45-214/test-handler.png) |

남·북 주통로와 동쪽 연결 통로를 방향별로 나누고, 19개 시각 생산 설비와 35개 제공처 모델을 배치했습니다. 제공처 모델은 정비 캐비닛·작업대·카트 34개와 NVIDIA 포장 컨베이어 1개입니다. 생산 장비 본체와 건물 서비스 형상은 기능 자료를 참고한 절차형 모델이며 제조사 CAD나 인증 공장의 복제물이 아닙니다.

위 36×28m 기준선은 `CELL1` 한 곳에서 공정을 계산하고 나머지 18개 설비는 시각·충돌 맥락을 만듭니다. 새 캠퍼스의 12개 논리 공정 셀과 구분하며, 76개 시각 설비 모두의 생산 능력을 계산한다는 뜻은 아닙니다. [장면 합성 근거](docs/evidence/semiconductor-dual-lane-201.json) · [에셋과 라이선스](docs/asset-attribution.md)

## 운영 정책 비교

![정상·테스터 고장·통로 폐쇄에서 FIFO와 ETA를 비교한 12lot 운영 결과](docs/evidence/operations-comparison-223.svg)

정상, 테스터 고장, 통로 폐쇄에 FIFO와 ETA를 조합한 6개 조건을 5,400초 지평에서 실행했습니다. 모든 조건에서 출하 12건, 빈 토트 반납 12건, 4대 주차가 완료됐습니다. 같은 seed는 확률 표본이 아니라 실행 조건 식별자로 사용했으며, 반복 통계나 실공장 성능을 뜻하지 않습니다.

- 정상 조건에서 ETA의 누적 교통 대기는 FIFO보다 **42.1% 적었습니다.**
- 통로 폐쇄 조건에서 ETA의 누적 출하 지연은 FIFO보다 **10.6% 많았습니다.**
- 거리 중심 ETA가 정상 흐름에는 유리했지만 통로 제약까지 포함하면 항상 우세하지 않았습니다.
- 납기 여유와 처리시간은 작성 가정이며, 최소 여유 1.73m는 원형 외곽 기반 CPU 관측값입니다.

[조건·실행 ID·원본 수치](docs/evidence/operations-comparison-223.md) · [기계 판독 결과](docs/evidence/operations-comparison-223.json)

### 캠퍼스 설비 고장·복구

![TEST 고장 중 잔여 처리시간 보존과 정상 대비299.9초 완료 지연](docs/evidence/campus-equipment-comparison.svg)

같은48lot 수요와9,000초 관측 구간에서 `L2.TEST`의 정상·고장을 비교했습니다. HTTP로300초 고장·복구를 적용하자 처리 중인 `LOT-0004`의 남은 시간은29.6초로 보존됐고, 복구1초 뒤28.6초로 줄었습니다.

| 관측 항목 | 정상 | TEST 300초 고장 |
|---|---:|---:|
| 대상 lot의 TEST 완료 시각 | 224.7초 | 524.6초 |
| 구간 종료 시 출하 | 2건 | 2건 |
| 완료 운송 | 40건 | 39건 |

대상 공정은299.9초 늦어졌지만 이 구간에서 전체 출하 차이는 나타나지 않았습니다. 동결314 모델·설정의 CPU 비교이며 물리 운송·브라우저 조작·48lot 완주 결과와 구분합니다. [도표 출처](docs/evidence/campus-equipment-comparison-figure.json) · [비교 근거와 원본 해시](docs/evidence/campus-equipment-comparison.json) · [선별한 상태10행](docs/data/campus-equipment-comparison/boundaries.csv) · [재현 방법](docs/operations-guide.md#설비-고장-비교-재현)

## 물리 주행 기준선

![무적재 Nova Carter의 참조·엔코더·GT 궤적과 속도·횡오차](docs/evidence/local45-path-baseline-226.svg)

Isaac Sim 4.5에서 무적재 Nova Carter 한 대가 2m 직선과 반지름 2m의 1/4 원호를 추종했습니다. 제어와 도착 판정에는 휠 엔코더만 사용했고, 시뮬레이터 GT는 독립 평가에만 사용했습니다.

| 지표 | 결과 |
|---|---:|
| 경로 횡오차 RMS | 4.76cm |
| 종점 위치 오차 | 7.12cm |
| 실제 최고 선속도 | 0.5915m/s |
| 정지 후 이동 | 0.01mm |

이 결과는 무적재 단일 차량의 구성요소 시험입니다. Nav2, 플릿 교통, 적재 접촉, 도킹, 공장 전체 운송 성능을 포함하지 않습니다. [실행·원본·프로세스 종료 근거](docs/evidence/local45-path-baseline-226.json) · [제어 기준선 설명](docs/motion-baseline.md)

<details>
<summary>도킹 정렬: 저속 접근·방향 정렬·1초 정지 확인</summary>

| 엔코더 제어와 독립 GT 평가 | 실제 목표 패드 정렬 장면 |
|---|---|
| ![도킹 정렬 궤적과 오차](docs/evidence/local45-docking-alignment-231.svg) | [![목표 패드에서 정지한 Nova Carter](docs/images/docking-alignment-local45-231/docked.png)](docs/images/docking-alignment-local45-231/docking-alignment-4fps.mp4) |

별도 실행231에서 엔코더만으로 약1.5m를 접근하고 목표 방향에 맞춰 정지했습니다. 독립 GT의 위치 오차는 **4.19mm**, yaw 오차는 **2.00°**, 정지 수락 후 이동은 **0.58mm**였습니다. 실제 RGB66장을4fps·16.5초 MP4로 보존했으며 프레임을 보간하지 않았습니다. 작성한 패드에 대한 무적재 단일 차량 시험이고, 전기 접촉·충전 전력·실물 도킹 정확도는 검증하지 않았습니다. [조건·판정·소유 프로세스 종료](docs/evidence/local45-docking-alignment-231.json) · [미디어 출처](docs/images/docking-alignment-local45-231/provenance.json)

</details>

<details>
<summary>보조 물리 영상: 6kg 적재 주행과 교차 통로 양보</summary>

| 6kg 적재 곡선·제동·회전 | 두 AMR의 양보·통과·재개 |
|---|---|
| ![6kg 토트를 올린 Nova Carter의 곡선과 제동 시험](docs/images/loaded-mobility-local45-135/loaded-turn-40fps.gif) | ![교차 통로에서 선행 차량을 보낸 뒤 재출발하는 두 AMR](docs/images/junction-hero-local45-142/logistics-yield-40fps.gif) |

[적재 주행 60fps MP4](docs/images/loaded-mobility-local45-135/loaded-motion-60fps.mp4)는 한 대의 적재 안정성·가감속·회전을 확인한 시험이며 자동 적재나 도킹 시험이 아닙니다. [결과](docs/evidence/local45-loaded-mobility-135.json)

[교차 통로 60fps MP4](docs/images/junction-hero-local45-142/logistics-yield-60fps.mp4)는 4대가 있는 장면에서 두 차량의 정지·양보·재출발을 확인한 시연입니다. 동적 우회 경로 생성이나 전체 ROS/Nav2 운송 완주를 뜻하지 않으며 화물은 비충돌 시각 표시입니다. [결과](docs/evidence/local45-junction-hero-142.json)

</details>

## 웹 관제와 운영 기록

![실행070을 읽기 전용으로 관측한 웹 관제 화면](docs/images/web-console-070/dashboard-live-carter1.jpg)

위 이미지는 실행070의 기존 관측 화면입니다. 차량을 고르면 작업·목적지·적재물·엔코더 속도·SOC 원천·대기 이유와 남은 경로를 확인합니다. 현재 코드에는 CPU 운영 명령, 명령 receipt, 최근 120개 관측의 저장·읽기 전용 재생이 연결돼 있습니다. 이 최신 조합의 새 브라우저 클릭 확인은 아직 수행하지 않았습니다. [이미지 출처](docs/images/web-console-070/provenance.json) · [운영 화면 사용법](docs/operations-guide.md)

선별한 정상 운영 기록 120개는 작은 공개 패키지로 제공합니다. 원본 JSON을 보간하지 않고 gzip으로 보존했으며, 복원 SHA와 전체 실행·발췌 구간의 차이를 함께 기록했습니다. [데이터 내려받기와 재생 방법](docs/data/operations-normal-223/README.md) · [공개 출처](docs/data/operations-normal-223/provenance.json)

<details>
<summary>Isaac Sim 4.5 Kit 운영 패널: 실행229</summary>

![Isaac Sim 4.5에서 실행한 Factory Operations 패널](docs/evidence/kit-operations-229/kit-console.png)

네이티브 Kit 패널에서 일시정지, 재개, 주문 생성, `CELL1` 고장, `CELL1` 복구의 5개 명령을 같은 business run에 적용하고 receipt를 확인했습니다. 종료 후 Job에 남은 소유 PID는 0개였습니다. 배경은 공장 전체 장면이 아닌 빈 viewport 연결 시험이며 ROS, 접촉 물리, 하드웨어 제어의 증거가 아닙니다. [실행 요약·해시·종료 기록](docs/evidence/kit-operations-229/summary.json)

</details>

<details>
<summary>기존 40×24m 정적 장면의 합성데이터</summary>

![RGB·깊이·semantic·instance·bbox를 묶은 기존 정적 데이터](docs/images/g4-render-004/g4-dataset-modalities.png)

Isaac Sim Replicator로 12개 시각 구성, 30시점, 3카메라에서 RGB 1,080장과 깊이·semantic·instance·bbox·카메라 정보를 만들었습니다. 기존 40×24m 정적 장면의 데이터이며 현재 36×28m 장면의 학습 데이터나 학습 성능 결과가 아닙니다. [데이터 검사](docs/images/g4-render-004/dataset-validation-report.json) · [데이터 계약](docs/dataset-contract.md)

</details>

## 합성 데이터가 실제 사진에서도 통하는가

기존 Replicator RGB 1,080장의 생성·검사 기록을 바탕으로, 실제 물류 사진에서 검출 성능과 실제 라벨 예산 곡선을 비교하는 보조 실험을 준비하고 있습니다. **검출기 학습과 실사 mAP 결과는 아직 없습니다.**

공식 RGB ZIP을 받아 사진5,097장과 박스151,428개의 메타데이터를 대조했습니다. 모두 디코딩·해상도가 맞았고 분할 간 동일 파일/픽셀·pHash 거리6 이하 후보는0건이었습니다. 촬영 환경·시퀀스 감사는 남아 있습니다. LOCO에는 AMR 클래스가 없고, 소형 적재함이 없는 초기 튜닝 후보도 수정했습니다. 평가용 sub1은 원본 val 일부를 고정한 프로젝트 분할이며 공식 LOCO test가 아닙니다. [RGB 감사](docs/evidence/loco-rgb-249.json) · [분할·해시 index](docs/data/loco-eval-v1/README.md) · [범주·분할 출처](docs/evidence/loco-protocol-310.json)

반도체 장면의 트레이·매거진을 팔레트·적재함으로 잘못 학습하지 않도록 에셋 허용 목록과 거절 검사를 구현했습니다. 실제 학습 후보 사진과 비교해 얕은 회색 트레이 대신 높이24cm의 NVIDIA 청색 적재함을 추가했습니다. 팔레트·청색 적재함의 소량 촬영302는4개 seed·16장 RGB를 생성했고, **60개 객체의 자동 박스가 각 객체 semantic 마스크 외곽과 일치**했습니다. 원본 에셋의 기본 라벨은 소비용 사본에서 분리했고 class/object_id·에셋 해시를 연결해 보존했습니다. LOCO 클래스 등가성과 학습 적합성은 미승인입니다. [302 촬영·라벨·출처 근거](docs/evidence/perception-blue-probe-302.json) · [이전 회색 후보278](docs/evidence/perception-probe-278.json)

![팔레트와 깊은 청색 적재함의4개 seed·16장 실제 GPU 촬영](docs/images/perception-blue-probe-302/contact-sheet.png)

새 export는 `object_id`로 각 객체의 semantic mask와 tight bbox 외곽을 대조합니다. 프레임 경계에 닿은 객체는 포함하고, 완전히 가려져 visible mask가 없는 객체는 거부합니다. 기존302 원본도 이 검사에서16프레임·60개 객체가 일치했으며 원본 데이터는 바꾸지 않았습니다. [검사 범위와 원본 해시](docs/evidence/perception-object-mask-export.json)

위 자료는 단순 바닥 위 후보 에셋의 export 확인용입니다. [청색 적재함 원본 RGB](docs/images/perception-blue-probe-302/deep-blue-bins.png) · [팔레트 혼합 원본 RGB](docs/images/perception-blue-probe-302/pallet-and-bins.png). 학습 준비가 완료된 공장 데이터로 보지 않습니다. 후속 학습 파일럿은512장, Cosmos는10클립부터 시작하며 변환본의 객체·라벨 정합을 확인하기 전에는 원본 라벨을 복사하지 않습니다. [비교 조건·GPU·비용과 확대 기준](docs/synthetic-to-real-plan.md)

실제 LOCO 학습용 사진6장과 합성 후보를 비교해 적층·가림·원거리 소형 객체·현장 배경의 차이를 정리했습니다. 평가용 sub1은 열지 않았습니다. 기존 용기를 제한된 외형의 baseline으로 사용하되, 단일 청색 용기가 전체 소형 적재함 클래스를 대표한다고 해석하지 않습니다. [실사·합성 비교 이미지와 준비 범위](docs/synthetic-to-real-plan.md#실제-학습-사진과-합성-후보의-차이)

512장 파일럿은 회색·청색 용기의 적층·가림·배경·크기·시점 변형을 고정하고 객체별 박스·마스크와 깊이를 검사합니다. v5 입력의 촬영328은 **유효101장·객체545건의 박스/마스크·깊이 일치**를 확인했습니다.102번째 프레임은 박스가 발행된 객체의 semantic ID가 없어 거절했고 RGB·라벨·깊이를 진단용으로 보존했습니다. 한 장 진단336에서 동일 입력의 tight·loose 박스가 모두1×2픽셀로 재현됐지만 의미·인스턴스 마스크에는 해당 객체의 픽셀이 없었습니다. 화면 경계의 박스/래스터 불일치까지 확인했고 정확한 처리 원인은 미확정입니다. 대량 촬영과 학습은 보류합니다. **512장 완성 데이터와 검출기 학습 성과는 아직 없습니다.** [328 부분 결과·원본 해시·종료 근거](docs/evidence/perception-pilot-328.json) · [v5 입력과 확인 경계](docs/evidence/perception-pilot-v5-preparation.json) · [촬영 구현](docs/evidence/perception-pilot-512-preparation.json)

<details>
<summary>한 프레임으로 재현한 박스·마스크 경계 불일치</summary>

![같은 프레임의 RGB·인스턴스 마스크와 다섯 객체의 박스/화소 수 비교](docs/images/perception-frame102-336/diagnostic.png)

프레임102의 장면·카메라·객체를 고정해 한 번 촬영한 진단입니다. 실패 객체는 유효 학습 자료에 포함하지 않았습니다. [실행·해시·종료 근거](docs/evidence/perception-frame102-336.json) · [이미지 출처](docs/images/perception-frame102-336/provenance.json) · [에셋 라이선스](docs/asset-attribution.md)

</details>

<details>
<summary>합성 파일럿328의 유효101장 중 대표10장</summary>

![회색·청색 용기와 팔레트의 여러 배치·시점에서 고른 유효 부분10장](docs/images/perception-pilot-328/partial-contact-sheet.png)

유효101장 중 고른 실제 RGB10장입니다. 실패한102번째 프레임은 포함하지 않습니다. 배경·가림 소품은 절차형 단순 형상이며 실제 창고 사진이 아닙니다. NVIDIA 원본 에셋·CC-BY-4.0·소비용 사본 변경 내역은 [에셋 출처](docs/asset-attribution.md)에 기록했습니다.

</details>

## 현재 진행 상태

| 구간 | 확보한 증거 | 남은 확인·확장 |
|---|---|---|
| 1. 공정·원장 | 4개 라인·12개 논리 셀, 48lot 출하·반납과 유한 재고 보존 | 최종 정지·16대 동시 운영 |
| 2. CPU 운영 | 4대 6조건 완주, 캠퍼스16대 전 차량4회 이상·총240회 운송 | 최종 정지 판정·16개 동시 물류 임무·차량 가용성 |
| 3. 물리 주행 기준선 | 경로 추종226, 적재135, 교차 양보142, 도킹 정렬231 | 전기 접촉 충전·실물 검증은 후속 확장 |
| 4. 운영 비교·원인 추적 | 4대의 6개 조건, 캠퍼스 TEST 고장·복구의 처리 지연 비교 | 캠퍼스48lot 완주, 반복 통계·실공장 성능은 미확인 |
| 5. 관제·재현·미디어 | 실행070 관측, 공개 120표본 재생, Kit229 명령, 캠퍼스 설비 HTTP 명령, 운영 재생228 | 최신 웹 클릭·16대 최종 장면 조합 확인 |
| 6. 포트폴리오·데이터 | 영상·비교·물리 기준선·LOCO RGB/분할 감사 | 16대 새 미디어, 합성→실사 파일럿·선별 보존 |

현재 본체의 목표는 16대 반복 물류 운영과 장애 대응을 같은 기록으로 재현하는 것입니다. 기존 4대 운영·단일 차량 물리 시험의 성과는 유지하며, 16대 물리·센서 수용성은 별도 확인합니다. 합성→실사 실험은 본체와 병행하는 보조 트랙입니다.

실패 원인과 재발 방지는 [트러블슈팅 기록](docs/troubleshooting.md)에 남깁니다. 현재 구현과 장기 완료 기준은 [완성 계획](docs/completion-plan.md), 선별 원본·별도 드라이브 백업·만료된 후보 파일은 [보존 색인](docs/evidence/operations-storage-230.json)에서 관리합니다.

## 로컬 실행

CPU 운영과 웹은 Python 3.12·uv, Node.js·npm을 사용합니다. 물리·렌더 시험 환경은 Isaac Sim 4.5와 RTX 3060이며 헤드리스로 실행했습니다. 같은 포트의 API나 웹이 이미 실행 중이면 프로세스를 중복으로 띄우지 않습니다.

```powershell
cd "$HOME/factory-ops-twin"
uv sync --locked
uv run python -m factory_ops.dashboard --config configs/factory-semiconductor-dual-lane --execution-mode operations --dispatch-policy eta --scenario-dir runs/operator-scenarios --port 8765
```

```powershell
cd "$HOME/factory-ops-twin/dashboard"
npm ci
npm run dev -- --port 3007
```

웹 주소는 `http://127.0.0.1:3007`입니다. 기존 API가 다른 옵션으로 실행 중이라면 사용 중인 작업을 마친 뒤 해당 API만 종료하고 위 설정으로 다시 시작합니다. API 주소가 같으면 웹을 다시 시작할 필요가 없습니다.

16대 캠퍼스의 공정 모델은 별도 포트의 `process-line` 모드로 관측합니다.4개 라인·12개 셀·48lot을 계산하며 기본값은 읽기 전용입니다. `--process-line-controls`를 추가하면 운영 패널에서 대상 설비를 선택해 고장·복구를 적용할 수 있습니다. 명령은 해당 로컬 CPU 실행에만 적용되고, 실행 ID·revision·명시한 설비를 확인한 receipt를 반환합니다. 차량 조종·주문 추가·통로 변경은 이 옵션으로 열리지 않습니다. 기존4대 API·웹을 유지하면서 다음 API와 웹을 각각 실행할 수 있습니다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops.dashboard --config configs/factory-semiconductor-campus16 --execution-mode process-line --lots 48 --port 18766
```

설비 고장·복구와 관측 저장을 사용할 실행에서는 위 API 명령에 `--process-line-controls --scenario-dir runs/campus-scenarios`를 추가합니다. 저장한 관측에는12개 설비의 상태가 남으며 재생은 읽기 전용입니다. 이 옵션은 별도 실행 중인 배치의 상태를 바꾸지 않습니다.

```powershell
cd "$HOME/factory-ops-twin/dashboard"
$env:OPS_API_URL = "http://127.0.0.1:18766"
npm run dev -- --port 13008
```

캠퍼스 웹 주소는 `http://127.0.0.1:13008`입니다. API287의 읽기 전용 HTTP 관측을 보존하며, 후속 명령 연결은 HTTP 고장 적용, 설비 명령의 고장·복구·같은 실행의 receipt·재생 보존을 포함한9개 관련 검사를 통과했습니다. 화면의 설비 선택 변경은 TypeScript 검사를 통과했지만 브라우저 도구의 커널 초기화 오류로 새 클릭·반응형 확인은 남아 있습니다. 이 결과는48lot 완주를 뜻하지 않습니다. [읽기 전용 기준선287](docs/evidence/process-line-api-287.json) · [명령·화면 변경의 확인 범위](docs/evidence/campus-equipment-controls.json)

6개 운영 조건 비교는 다음 명령으로 재현합니다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops.operations_experiment --config configs/factory-semiconductor-dual-lane --output artifacts/operations/my-comparison/matrix.json --all --horizon 5400
```

## 저장소 구조와 한계

| 경로 | 역할 |
|---|---|
| `configs/` | 공장 배치, 경로, 주문, 주행, 에너지 설정 |
| `factory_ops/` | 공정 원장, 재고, 배차, 예약, 충전, 관제 API, 평가 |
| `scripts/isaac/`, `scripts/ros/` | Isaac Sim 물리 시험, 센서, ROS 2·Nav2 연결 |
| `dashboard/`, `extensions/` | 웹 3D 관제와 Kit 운영 패널 |
| `docs/images/`, `docs/evidence/`, `docs/data/` | 미디어, 실행 근거, 공개 재생 데이터 |

시뮬레이터 GT는 제어 입력이 아니라 사후 평가·라벨·미디어에 사용합니다. 현재 결과는 실물 Sim-to-Real 검증, 산업 안전 인증, 처리량 보증을 포함하지 않습니다. 로봇은 NVIDIA Nova Carter를 참조하며 원본 에셋은 저장소에 넣지 않습니다. 외부 모델의 출처와 변경 내용은 [에셋 표기](docs/asset-attribution.md), GPU 실행과 원본 보존 방식은 [GPU 안내](docs/gpu-handoff.md)에서 확인할 수 있습니다.
