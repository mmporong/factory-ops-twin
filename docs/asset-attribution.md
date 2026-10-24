# NVIDIA SimReady 공장 에셋

새78×60m 캠퍼스는 아래 이중 차로 베이의 제공처 모델을4개 베이로 옮긴 배치다. 교량·진입 통로·PACK 이탈 대기 구역의0.1m 여유를 침범하는36개 fixture를 제외하고104개를 사용한다. 축척·출처 리비전·시각 전용 조건을 유지하며, 생산 충돌 외곽76개는 별도로 보존한다. 실제 CPU USD 생성은 마쳤지만 캠퍼스의 GPU 촬영·물리 수용성 검증은 남아 있다. [캠퍼스 생성 근거](evidence/campus16-254.json)

반도체 최종검사·포장 베이의 외곽 정비 구역에는 NVIDIA SimReady 모델 3종을 사용한다. 생산 장비의 테스트 핸들러·ATE·AOI를 대신하는 모델은 아니다. 금속 표면, 손잡이·경첩·바퀴와 텍스처가 있는 정비 소품을 실제 크기로 배치한다.

새 이중 차로 구성에는 출하 포장 구역의 `FG_LOW`에 [conveyor_belt_a61](https://huggingface.co/datasets/nvidia/simready-assets/tree/653d91e182011f357cf1644013cc94ddf194d349/conveyor_belt_a61)을 추가한다. 롤러·벨트·프레임·배선·검사 게이트 형상이 있는 범용 컨베이어이며 반도체 테스트 핸들러로 표시하지 않는다. 모델 크기는 약1.542×4.000×2.311m다.90도 회전해 기존4.6×2.5m 평면 외곽 안에 넣고, 충돌 외곽의 높이는1.6m에서2.4m로 함께 갱신한다. 임의 축소와 기존 절차형 포장대의 중복 표시는 하지 않는다. 벨트 구동·검사 센서·자동 선별은 현재 구현 범위가 아니다.

이 구성의4종·35개 모델은 고정 revision `653d91e182011f357cf1644013cc94ddf194d349`를 사용한다. 전체 선택 런타임은1,341,514,263B이며, 이전 캐시와 해시가 같은147개 파일을 재사용해 추가 다운로드를494,707,812B로 줄였다. 이전 캐시를 수정하지 않는다. 장비에 배치하는 모델도 물리 API·관절 typed prim을 제거한 시각 전용이다. 기존 설비 충돌체가 물리·센서 외곽의 권위이며 모델의 실제 XYZ 형상이 그 외곽 안에 있어야 한다. 같은 revision의3종 지원 소품34개도 새 통로 밖에 재배치한다.

[실제 USD 합성](evidence/semiconductor-dual-lane-201.json)에서35개 월드 외곽, 정적 충돌체20개 보존, 추가 물리 API·관절0개를 확인했다. [202 조감·핸들러 RGB](images/semiconductor-dual-local45-202/provenance.json)와 [203 컨베이어 근접 RGB](images/semiconductor-dual-local45-203/provenance.json)를 SDK4.5에서 촬영했다.202의 단색 추가 프레임은 제외했으며 원본을 성공 이미지로 바꾸지 않았다.203은 수정된 카메라 하나의 재촬영으로, 두 실행을 하나의 운영 시각으로 합치지 않는다. 렌더 오류0건과 각 Job의 종료·남은 소유 PID0을 확인했다.

현재 구성의 캐시는 아래 명령으로 준비한다. 제공처 원본은 Git에 넣지 않는다.

```powershell
cd "$HOME/factory-ops-twin"
$factorySdk = 'E:/IsaacSim/isaac-sim-4.5.0'
$factoryAssetCache = 'E:/FactoryOpsAssets/simready-context'
& "$factorySdk/kit/python/python.exe" -B scripts/assets/prepare_factory_context.py --config configs/factory-semiconductor-dual-lane/visual-assets.json --cache-root $factoryAssetCache --sdk $factorySdk
$env:FACTORY_OPS_ASSET_CACHE = $factoryAssetCache
```

| 용도 | NVIDIA 원본 패키지 | 준비된 형상 크기 X×Y×Z | 배치 수 |
|---|---|---|---:|
| 공구 보관 캐비닛 | [furniture_workbench_modular_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/524747d9b5e1f21b5b18b56041788c01d745f1a1/furniture_workbench_modular_a01) | 0.714×1.445×2.050m | 15 |
| 이동식 정비 작업대 | [equipment_rolling_workstation_cabinet_c01](https://huggingface.co/datasets/nvidia/simready-assets/tree/524747d9b5e1f21b5b18b56041788c01d745f1a1/equipment_rolling_workstation_cabinet_c01) | 0.584×1.017×1.392m | 10 |
| 정비 부품 카트 | [equipment_utility_cart_e01](https://huggingface.co/datasets/nvidia/simready-assets/tree/524747d9b5e1f21b5b18b56041788c01d745f1a1/equipment_utility_cart_e01) | 0.854×0.412×0.907m | 9 |

`furniture_workbench_modular_a01`의 선택된 모델은 높은 양문 캐비닛 형상이다. 패키지 이름과 장면의 역할을 구분한다. 치수는 공개 CAD의 실측 사양이 아닌, 내려받은 USD의 가시 형상을 미터·Z축 위 방향으로 정리한 뒤 측정한 바운딩 박스다. 배치할 때 축을 돌릴 수 있지만 임의로 확대하지 않는다.

## 출처와 이용 조건

- 제작자: **NVIDIA**. 데이터셋: [nvidia/simready-assets](https://huggingface.co/datasets/nvidia/simready-assets).
- 이전196 촬영의 고정 리비전: `524747d9b5e1f21b5b18b56041788c01d745f1a1`. 현재 이중 차로의4종 구성은 위에 적은 `653d91e182011f357cf1644013cc94ddf194d349`를 사용한다.
- 모델 패키지 라이선스: **CC BY 4.0**. 원본의 라이선스·저작권 파일은 캐시에 함께 보존한다. 패키지별 원문 링크는 [설정의 catalog](../configs/factory-semiconductor/visual-assets.json)에 있다. MDL 등 포함 파일의 별도 고지도 보존한다.
- 변경: 소비용 USD에 미터·Z-up·바닥 기준 중심 정렬을 적용하고, 정적 배경용으로 Physics/PhysX API와 관절 typed prim을 제거했다. 원본 패키지는 수정하지 않는다. 출처, 파일별 SHA256, 준비 도구 해시는 캐시 색인에 남긴다.
- 이 페이지와 README의 모델 미리보기 PNG는 NVIDIA가 배포한 원본 썸네일을 변경 없이 복사한 자료다. **프로젝트에서 촬영한 RGB가 아니다.** 파일별 원본 경로·해시는 [미리보기 출처](images/factory-context-assets-193/provenance.json)에 있다.

이 모델의 사용이 NVIDIA의 추천이나 실제 반도체 공장의 ESD·클린룸 인증을 뜻하지 않는다. 정비 소품의 적재·주행·작업 동작을 검증한 범위도 아니다.

## 최신 카탈로그와 공장 형상 기준

핸들러·ATE·AOI와 나머지 생산 설비는 공개 기능 자료를 참고한 절차형 모델이다. NVIDIA SimReady는 정비·포장 소품4종에 사용하며, 모든 생산 장비가 제공처의 고품질 CAD인 것은 아니다. 사용자가 제시한 [Caleb의 게시물](https://x.com/calebfahlgren/status/2113308155270541536)을 계기로 아래 OpenUSD 카탈로그를 확인했다. 첨부 영상의 사용 모델·렌더러와 현재 프로젝트의 구성이 같다고 확인한 것은 아니다. SimReady 표기는 목적별 메타데이터·검증 근거를 확인하기 위한 체계이며 시각 품질이나 최신 버전의 보증으로 사용하지 않는다.

[SimReady Central 공식 문서](https://docs.omniverse.nvidia.com/simready/latest/simready-central.html)는 모델별 profile·검증 근거·라이선스를 확인하고, 고정 revision과 재질·텍스처 의존성을 함께 보존하도록 안내한다. 새 후보는 `nvidia/simready-assets`의 `653d91e182011f357cf1644013cc94ddf194d349`에서 확인했다. 아래 수치는 선택 다운로드에 포함되는 런타임 파일 합계다. 얇은 USD 진입 파일의 크기만으로 설치 용량을 계산하지 않는다.

| 확인한 모델 | 선택 런타임 용량 | 적용 판단 |
|---|---:|---|
| [conveyor_belt_a61](https://huggingface.co/datasets/nvidia/simready-assets/tree/653d91e182011f357cf1644013cc94ddf194d349/conveyor_belt_a61) | 505,775,934B | 새 출하 포장 구역에 적용. 반도체 핸들러 대용으로 표시하지 않음 |
| [conveyor_belt_a47](https://huggingface.co/datasets/nvidia/simready-assets/tree/653d91e182011f357cf1644013cc94ddf194d349/conveyor_belt_a47) | 282,454,899B | 분기 컨베이어. 기존 포장 설비 외곽에 실제 크기로 들어가는지 확인 필요 |
| [workstation_factory_a01](https://huggingface.co/datasets/nvidia/simready-assets/tree/653d91e182011f357cf1644013cc94ddf194d349/workstation_factory_a01) | 1,286,854,586B | 산업 워크스테이션. 용도·치수와 SDK4.5 재질 호환성 확인 전에는 미적용 |

공장 형상은 [Amkor의 테스트·포장 현장 사례](https://amkor.com/test-services/)와 [Advantest의 테스트 핸들러 설명](https://www.advantest.com/en/products/component-test-system/test-handler/)을 기준으로 삼는다. 트레이 저장·로트 투입, 핸들러와 테스터, 검사·마킹, 포장·출하의 관계를 보이고 정비 카트는 서비스 공간에 둔다. 실제 공장의 측량 배치나 특정 제조사의 CAD 복제를 주장하지 않는다.6m 주통로는 현재 보수적인 예약 외곽을 분리하기 위한 설계값이며 업계 표준 통로 폭으로 제시하지 않는다.

## 이전 정비 에셋 구성의 촬영과 재현

남쪽 정비 열, 입고·반납 로드포트 옆 지원 공간, 주차장 사이 기술자 공간과 외벽에 총34개를 배치한다. 합산 바닥 외곽은 약24.58㎡다. 모든 AMR 이동 구역·도킹·주차 영역과 기존 설비에서 최소0.1m를 비운다. 물류 경로에 물건을 채우기 위한 배치는 허용하지 않는다.

CPU에서 실제 USD 참조를 합성해34개 형상의 월드 바운딩 박스와 배치 기록의 일치를 확인했다. 기존20개 충돌체 경로는 유지됐고 추가 모델의 물리 API·관절 typed prim은0개다. 이 배경 모델은 렌더링 범위이며 LiDAR·충돌 지형에 추가하지 않는다. **Isaac Sim4.5에서34개를 포함한 조감·근접 RGB 두 장을 촬영했다.** 렌더·관절 오류0건과 촬영 프로세스 종료를 확인했다. 실패한 이전 운송063에는 새 에셋이 들어 있지 않다. [합성 결과와 소스 해시](evidence/factory-context-196.json) · [실제 RGB와 종료 기록](images/semiconductor-layout-local45-196/provenance.json)

런타임과 텍스처의 선택 다운로드는 총835,738,329바이트다. 제공처 파일은 Git에 넣지 않고 로컬 캐시에 보존한다. Windows의 기존 Isaac Sim4.5 SDK를 사용해 CPU에서 준비하며, Kit·GPU·ROS를 시작하거나 패키지를 추가 설치하지 않는다. SDK 경로가 다르면 아래 값을 설치 위치로 바꾼다.

```powershell
cd "$HOME/factory-ops-twin"
$factorySdk = 'E:/IsaacSim/isaac-sim-4.5.0'
$factoryAssetCache = 'E:/FactoryOpsAssets/simready-context'
& "$factorySdk/kit/python/python.exe" -B scripts/assets/prepare_factory_context.py --config configs/factory-semiconductor/visual-assets.json --cache-root $factoryAssetCache --sdk $factorySdk
$env:FACTORY_OPS_ASSET_CACHE = $factoryAssetCache
```

같은 PowerShell에서 새 반도체 구성의 실행기를 시작하면 이 캐시를 읽는다. 다른 터미널에서 실행할 경우에도 같은 환경변수를 설정해야 한다. 캐시가 없거나 준비가 끝나지 않았으면 오류로 중단하며, 낮은 품질의 대체 모델로 조용히 바꾸지 않는다.
