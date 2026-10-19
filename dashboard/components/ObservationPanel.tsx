import { modeNames, sceneSize, type Scene, type Snapshot } from "../lib/types";

type Props = {
  snapshot: Snapshot | null;
  scene: Scene | null;
  stale: boolean;
  age: number | null;
  observationAge: number | null;
  latency: number | null;
};

export default function ObservationPanel({
  snapshot, scene, stale, age, observationAge, latency,
}: Props) {
  const number = new Intl.NumberFormat("ko-KR", { maximumFractionDigits: 1 });
  const readOnly = snapshot && !snapshot.capabilities.some((action) => ["pause", "resume", "create_order"].includes(action));
  const scope = snapshot?.mode === "local_model"
    ? "CPU 운영 원장과 경로 보간 · 물리 주행·센서 검증 미연결"
    : snapshot?.mode === "replay"
      ? "저장된 관측 재생 · 현재 실행 상태와 구분해 확인하세요."
      : snapshot
        ? "실행 원천의 관측 수신 · 센서·종료 검증은 별도 증거가 필요합니다."
        : "운영 원천에 연결하고 있습니다.";
  return (
    <section className="observation-panel" aria-labelledby="observation-title">
      <div className="observation-heading">
        <div>
          <span className="eyebrow">ISAAC SIM · ROS 2</span>
          <h2 id="observation-title">시뮬레이션 관측</h2>
        </div>
        <span className={`observation-state ${stale ? "is-stale" : ""}`}>
          {snapshot ? stale ? "관측 지연" : readOnly ? "읽기 전용 · 관측 수신" : "운영 데이터 수신" : "연결 대기"}
        </span>
      </div>
      <dl className="observation-metrics">
        <div>
          <dt>데이터 원천</dt>
          <dd>{snapshot ? modeNames[snapshot.mode] : "미연결"}</dd>
        </div>
        <div>
          <dt>시뮬레이션 시계</dt>
          <dd>{snapshot ? <>{number.format(snapshot.sim_time_s)} <span>s</span></> : "미관측"}</dd>
        </div>
        <div>
          <dt>관측 경과</dt>
          <dd>{observationAge === null ? "미관측" : <>{number.format(observationAge)} <span>초</span></>}</dd>
        </div>
        <div>
          <dt>공장 배치 · 주차</dt>
          <dd>{sceneSize(scene)}{scene && <small>주차 {Object.keys(scene.pockets).length}면 · API 배치 기준</small>}</dd>
        </div>
      </dl>
      <div className="observation-foot">
        <p>{scope}</p>
        <details className="observation-details">
          <summary>센서·실행 근거 확인</summary>
          <div className="observation-evidence">
            <div>
              <h3>수신 원천</h3>
              <dl>
                <div><dt>실행 ID</dt><dd>{snapshot?.run_id ?? "미연결"}</dd></div>
                <div><dt>원장 버전</dt><dd>{snapshot?.revision ?? "미연결"}</dd></div>
                <div><dt>관측 시각</dt><dd>{snapshot ? new Intl.DateTimeFormat("ko-KR", { hour: "2-digit", minute: "2-digit", second: "2-digit" }).format(new Date(snapshot.observed_at)) : "미관측"}</dd></div>
                <div><dt>최근 API 수신</dt><dd>{age === null ? "미연결" : `${age}초 전`}</dd></div>
                <div><dt>API 왕복</dt><dd>{latency === null ? "미측정" : `${latency} ms`}</dd></div>
                <div><dt>배치 버전</dt><dd>{scene?.layout_revision ?? "미연결"}</dd></div>
                <div><dt>좌표계 · 지도 해상도</dt><dd>{scene?.frame ?? "미연결"} · {scene?.map_resolution_m == null ? "미연결" : `${scene.map_resolution_m} m`}</dd></div>
              </dl>
            </div>
            <div>
              <h3>Isaac Sim 연결 항목</h3>
              <p className="evidence-note">구성 목표: AMR 4대 · LiDAR 8개. 아래 항목은 현재 API에서 관측값을 제공하지 않습니다.</p>
              <dl>
                <div><dt>LiDAR 수신·주기</dt><dd>미연결</dd></div>
                <div><dt>Nav2 · 위치 추정</dt><dd>미연결</dd></div>
                <div><dt>교통 예약·통행 승인</dt><dd>미연결</dd></div>
                <div><dt>물리 적재·접촉 검증</dt><dd>미연결</dd></div>
                <div><dt>실행 종료 후 검증</dt><dd>결과 미연결</dd></div>
                <div><dt>원본 보존 · 인스턴스 Stop</dt><dd>증거 미연결</dd></div>
              </dl>
              <p className="evidence-note">주문 완료 수와 명령 승인은 물리 검증·원본 보존·Stop 완료를 뜻하지 않습니다.</p>
            </div>
          </div>
          {snapshot?.model_scope && <p className="source-scope"><span>원천 범위</span> {snapshot.model_scope}</p>}
        </details>
      </div>
    </section>
  );
}
