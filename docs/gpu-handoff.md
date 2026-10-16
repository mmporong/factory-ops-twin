# GPU 연동과 작업 보존

로컬 준비 명령은 GPU나 Brev 계정을 사용하지 않는다. 별도 장면 검사 스크립트는 Isaac Sim 6.2 환경에서 실행한다. 이 스크립트의 실제 GPU 실행과 원격 Stop은 아직 검증하지 않았으며, 자동 Sleep 기능을 제공하지 않는다.

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

코드는 GitHub에, 실행 결과·영상·제공 에셋은 별도 지속 저장소에 둔다. 로컬 Windows의 결과 보존 위치는 `E:\FactoryOpsArtifacts`다. 이 폴더를 유일한 백업으로 사용하지 않는다. 폴더와 코드 묶음의 로컬 복사본은 준비했으며, GPU 결과 전송은 아직 수행하지 않았다.

외부 복사에는 실행 ID, 코드 SHA, 입력·출력 SHA256, 이미지 digest, 로봇·센서 설정, 시뮬레이션 시간과 wall time, GPU·드라이버, VRAM·RTF 기록을 함께 남긴다. 인증 정보는 추적하지 않는 설정이나 SSH 인증 도구로 전달하며 저장소·로그에 기록하지 않는다.

에이전트가 종료 후 Stop까지 처리하려면 해당 세션에 Brev 제어 경로와 유효한 인증, 종료 대상 인스턴스 식별자가 있어야 한다. AWS L40S 환경 생성과 Brev 기본 SSH 프록시 접속을 확인했다. 원격 Stop 실행은 아직 검증하지 않았다. 중지 성공이나 남은 비용을 확인하기 전에는 비용 차단 완료로 보고하지 않는다.

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

## 공식 근거

- [Isaac Sim 호환성](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/compatibility.html)
- [Python 설치 요구사항](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html)
- [ROS 2 지원 조합](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- [Brev 인스턴스 개념](https://docs.nvidia.com/brev/concepts/gpu-instances)
- [Isaac Sim 6.2 컨테이너 설치](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_container.html)
- [Isaac Sim 6.2 stage API](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/py/source/extensions/isaacsim.core.experimental.utils/docs/index.html)
- [Replicator 조감 이미지 저장](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/replicator_tutorials/tutorial_replicator_getting_started.html)
- [Isaac Sim 6.2 데이터 수집과 opt-out](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/common/data-collection.html)
