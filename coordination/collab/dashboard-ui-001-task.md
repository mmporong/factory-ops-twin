# dashboard-ui-001 / attempt1 / contract revision1

사용자가 현재 isaac-sim 대화를 총괄로 유지하고 옆의 control 페인에 대시보드 구현을 맡기라고 명시했다. 당신의 대상 identity는 Herdr default/w1:p1C, terminal term_65e06d27e415810, Codex session 01a13ae8-3482-7192-8245-8303f7fc4847 이다. 현재 대화의 관계없는 작업을 덮지 말고 이 카드 수신을 ACK한다.

프로젝트 루트 C:/Users/LIMMM/factory-ops-twin. 기준 main c67c2dac4dcee1f339fe764b6cac62e85466cefb. AGENTS.md와 C:/Users/LIMMM/.codex/skills/front/SKILL.md를 읽는다. 현재 요구는 AMR4대이며 과거8대 목표는 대체됐다.

읽을 계약: coordination/collab/dashboard-contract.md (SHA256 d6c5ceb6323369fad8f1ded212ec18c9f578eb63fa2294b388760d36403f9eab). 참고 이미지 C:/Users/LIMMM/AppData/Local/Temp/codex-clipboard-6a9kuJ.png를 view_image로 확인한다. 이미지의 흰색/파랑3D 물류 운영 화면을 우리32×20m 실내 생산 구역과4대 AMR로 적용한다. 사용자가 제공한 작성자 설명은 React/React Three Fiber/three.js, Next.js 선호,3D client-only 로딩이다. screenshot은 완성품 실행 증거가 아닌 디자인 참조다.

소유권: dashboard/ 전체만 작성한다. Next.js·React·React Three Fiber·three.js와 필요한 공식 R3F 제어 도구의 패키지 설치/lock/config는 이 사용자 요청의 구현 범위다. 불필요한 UI 라이브러리·외부 유료 자산·정적 대형 이미지 의존성을 늘리지 않는다. 다른 경로는 결과 receipt coordination/collab/results/dashboard-ui-001.json만 쓴다. root는 Python API와 scene/운영 어댑터·계획/README를 맡는다. 공유 트리에서 혼자 작업하는 것이 아니므로 다른 변경을 되돌리지 않는다. Git commit/push/배포/GPU 기동/추가 세션·native agent 생성은 하지 않는다.

구현 목표:
1. 중앙 사선 조감 공장3D, 상단 실제 API KPI, 우측 선택 상세, 하단 작업 흐름·4대 목록. 한국어, 화이트/옅은파랑, 절제된 파랑 강조. WareTrack의 브랜드·인물·트럭/야드·허위 수치를 복제하지 않는다.
2. 공장 geometry는 GET /api/scene의 bounds/obstacles/stations/allowed_regions와 같은 좌표. geometry의 m 단위와 map x,y→Three x,0,-y 변환 일관성. 토트/AMR/설비가 식별 가능하고 박스만 넓게 흩어진 화면을 피한다.3D 부분은 client-only lazy boundary, WebGL 실패 시 목록과 상세 유지.
3. AMR 또는 목록 클릭→동일 선택 상세: 현재작업·목적지·적재물·속도·대기이유·배터리. battery_pct=null이면 미연결, 원천을 보존한다. 초기 root 모델은 live Isaac가 아니므로 모드표시(local_model/replay/live)와 연결/지연 상태를 솔직하게 표시한다.
4. snapshot 폴링과 명령 API를 Next 서버 경유로 연결. 초기 pause/resume/create_order만 capabilities에 따라 노출/활성. 조작 후 applied/pending/rejected와 이유를 보여주며 서버 상태로 확정한다. 프런트에서 pose/SOC/작업 완료를 임의 생성하지 않는다. 충전/teleop 등 미연결 기능은 가짜 작동 버튼을 만들지 않는다.
5. 키보드 선택·명확한 포커스·목록 대체 경로·reduced motion·데스크톱/태블릿/모바일. 타입 검사/빌드와 핵심 상호작용 검증. 대규모 형식검사보다 실제 클릭 결과·데이터 흐름을 우선.

API는 root가 http://127.0.0.1:8765 에 구현한다. 계약의 JSON key를 따르고 변경이 필요하면 결과/질문 파일로 올린다. 구현 시작은 API 준비를 기다릴 필요가 없으나 실제 통합 검증은 root API가 준비된 뒤 수행한다. 최초에는 coordination/collab/results/dashboard-ui-001.json에 ACK와 selected stack을 기록하고 root가 회수할 수 있도록 한다. 완료 receipt에는 task/attempt/revision, 변경 경로, 검증 명령·결과, 오류/제약, 실행 서버 PID/중지 명령을 남긴다. root가 작성자가 아닌 검수자로 브라우저/통합을 검사한다. 세션 모델·추론 강도·이름·포커스를 바꾸지 않는다.
