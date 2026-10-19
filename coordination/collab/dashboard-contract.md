# 4대 공장 운영 대시보드 계약

Revision: 1. 기준 소스: c67c2dac4dcee1f339fe764b6cac62e85466cefb.

사용자 최신 결정은 최종 AMR4대와 React/Next.js·React Three Fiber 기반3D 운영 화면이다. 기존8대 목표를 대체한다. 참고 이미지는 C:/Users/LIMMM/AppData/Local/Temp/codex-clipboard-6a9kuJ.png, 작성자 링크는 https://x.com/DilumSanjaya/status/2106426962738880879 이다. 이미지의 구성과 선택 상호작용을 참고하며 브랜드·트럭/야드 업무를 복제하지 않는다.

## 역할과 범위

- root(isaac-sim 대화): Python API·운영 원장/시뮬레이션 어댑터·테스트, 완료 계획/README·Git 통합, 최종 브라우저 검수. 기존 물리/센서/안전 계약 유지.
- control(default/w1:p1C): dashboard/ 안의 Next.js·React·R3F 앱과 그 앱의 패키지/lock/config/types/테스트. 다른 경로 수정·commit/push·GPU 기동·추가 에이전트·배포 금지. 원장은 root만 갱신.
- 같은 저장소에 다른 작성자가 있으므로 다른 변경을 되돌리지 않는다. 공유 계약 수정은 root와 합의한다.

## 데이터/API 경계

API 기본 주소는 http://127.0.0.1:8765 이며 Next 서버 경유로 호출한다. 브라우저에서 클라우드 인증 정보나 ROS/Brev 자격증명을 다루지 않는다.

- GET /api/snapshot: 아래 Snapshot JSON. 주기적 조회로 시작하며 향후 전송 방식 변경에도 동일 데이터 계약 유지.
- GET /api/scene: configs/factory-v4/layout.json에 근거한 경량 scene JSON. 시뮬레이션의 좌표를 재사용하며 독립적인 공장 배치를 만들지 않는다.
- POST /api/commands: {command_id: string, expected_revision: integer, action: string, robot_id?: string, parameters?: object}. action 초기 지원은 pause/resume/create_order. command_id 재사용은 동일 payload만 허용. 실제 지원 목록은 snapshot.capabilities 기준. charger/teleop/물리 적재는 handler 연결 전 비활성.
- 명령 응답: {command_id, status: applied|pending|rejected, reason: string|null, revision: integer}. live 모드에서 요청 접수만으로 applied로 표시하지 않는다. 웹에서 pose/stock/SOC를 임의로 덮어쓰지 않는다.
- live 수집/승인 경로는 root가 구현. frontend는 수신 snapshot만 표시한다. API 실패·stale는 연결 끊김으로 표시하고 마지막 위치를 유지한다.

Snapshot 필드(모든 key 존재, 미관측 값은 null):

```json
{
  "schema_version": 1,
  "run_id": "local-model",
  "revision": 0,
  "mode": "local_model",
  "sim_time_s": 0,
  "observed_at": "2026-10-19T00:00:00+00:00",
  "connection": "connected",
  "paused": false,
  "capabilities": ["pause", "resume", "create_order"],
  "robots": [{
    "id": "amr-1", "label": "AMR 01", "status": "idle",
    "pose": {"x_m": 3.5, "y_m": 4.0, "yaw_rad": 0},
    "speed_mps": 0, "battery_pct": null, "battery_source": "unavailable",
    "task_id": null, "task_type": null, "task_label": null,
    "payload_id": null, "destination": null, "waiting_reason": null,
    "route": [], "observed_sim_time_s": 0
  }],
  "tasks": [],
  "inventory": {"stock": [], "totes": [], "conservation": []},
  "kpis": {"active_robots": 0, "total_robots": 4, "completed_orders": 0, "pending_tasks": 0, "stock_units": 0},
  "events": [],
  "commands": []
}
```

최종 목표는4대다. live/replay에서2대만 관측했다면2대의 관측만 표시하며, 미관측 차량을 움직이는 로봇으로 보충하지 않는다.
task: {id, order_id, type, state, robot_id, source, destination, payload_id, progress: number|null}。
event: {id, sim_time_s, type, message, robot_id: string|null}。
route: [{x_m, y_m}]. scene: {layout_revision, bounds_xy, stations, pockets, physical_obstacles, amr_allowed_regions, visual_zones, reserved_work_areas}. layout.json과 같은 구조다. Z-up/m의 map 평면(x,y)을 Three.js의(x,0,-y)로 변환하고 yaw에도 같은 변환을 적용한다.

## 화면과 검수

화이트 배경·옅은 파랑·파랑 강조색. 중앙의 사선 조감 공장, 상단의 소수 KPI, 오른쪽의 선택 AMR/토트/설비 상세, 하단의 작업 타임라인과4대 목록으로 구성한다. 한국어 UI이며3D·목록 선택이 동기화된다. AMR 선택 시 현재 작업·적재물·목적지·대기 이유·속도·SOC를 보여준다. 미관측 SOC는 미연결로 표시하며 값을 만들지 않는다.

로컬 모델/기록 재생/실제 시뮬레이션 연결을 구분해 표시한다. 라이브 위치의 표시 보간은 화면에만 쓰며 제어·도착 판정에 사용하지 않는다. sim_time과 수신 시각을 구분한다. WebGL 불가 시에도 목록·상세·조작을 사용할 수 있어야 한다. 실제 버튼의 결과·pending/실패·재연결·키보드·reduced-motion·desktop/mobile을 검증한다.

결과 receipt는 coordination/collab/results/dashboard-ui-001.json이다. task_id=dashboard-ui-001, attempt=1, contract_revision=1, ACK, 변경 파일, 검증 명령/종료 결과, 알려진 제약, 남긴 프로세스 PID/종료 방법을 포함한다. root가 검수하므로 자기 결과를 accepted로 판정하지 않는다.
