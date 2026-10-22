"use client";

import { FormEvent, useCallback, useEffect, useMemo, useRef, useState } from "react";

import type { Snapshot } from "../lib/types";

type Scenario = {
  scenario_id: string;
  name: string;
  created_at: string;
  frame_count: number;
};

type ReplaySnapshot = Snapshot & {
  replay?: {
    scenario_id: string;
    frame_index: number;
    frame_count: number;
  };
};

type Props = {
  liveSnapshot: Snapshot | null;
  replaySnapshot: Snapshot | null;
  onReplay: (snapshot: Snapshot | null) => void;
};

const PLAYBACK_INTERVAL_MS = 900;
const scenarioDate = new Intl.DateTimeFormat("ko-KR", {
  month: "numeric",
  day: "numeric",
  hour: "2-digit",
  minute: "2-digit",
});

async function responseJson<T>(response: Response): Promise<T> {
  const body = (await response.json().catch(() => ({}))) as { error?: string } & T;
  if (!response.ok) {
    const error = new Error(body.error || "운영 기록을 불러오지 못했습니다.");
    Object.assign(error, { status: response.status });
    throw error;
  }
  return body;
}

function errorStatus(error: unknown) {
  return typeof error === "object" && error !== null && "status" in error
    ? Number((error as { status: unknown }).status)
    : null;
}

export default function ScenarioReplayPanel({ liveSnapshot, replaySnapshot, onReplay }: Props) {
  const [name, setName] = useState("교대조 운영 기록");
  const [scenarios, setScenarios] = useState<Scenario[]>([]);
  const [selectedId, setSelectedId] = useState("");
  const [currentIndex, setCurrentIndex] = useState(0);
  const [simTimes, setSimTimes] = useState<Record<number, number>>({});
  const [available, setAvailable] = useState<boolean | null>(null);
  const [listLoading, setListLoading] = useState(true);
  const [frameLoading, setFrameLoading] = useState(false);
  const [saving, setSaving] = useState(false);
  const [playing, setPlaying] = useState(false);
  const [message, setMessage] = useState("");
  const listAbort = useRef<AbortController | null>(null);
  const frameAbort = useRef<AbortController | null>(null);
  const saveAbort = useRef<AbortController | null>(null);
  const frameRequest = useRef(0);

  const selected = useMemo(
    () => scenarios.find((scenario) => scenario.scenario_id === selectedId) ?? null,
    [scenarios, selectedId],
  );
  const replay = replaySnapshot as ReplaySnapshot | null;
  const replayIndex = replay?.replay?.scenario_id === selectedId ? replay.replay.frame_index : currentIndex;
  const frameCount = selected?.frame_count ?? replay?.replay?.frame_count ?? 0;
  const connectedLive = liveSnapshot?.connection === "connected"
    && (liveSnapshot.mode === "live" || liveSnapshot.mode === "local_model");
  const canSave = available === true && connectedLive && replaySnapshot === null
    && name.trim().length > 0 && name.trim().length <= 80 && !saving;

  const markUnavailable = useCallback(() => {
    setAvailable(false);
    setScenarios([]);
    setSelectedId("");
    setPlaying(false);
    onReplay(null);
    setMessage("현재 연결된 운영 원천에서는 기록 기능을 사용할 수 없습니다.");
  }, [onReplay]);

  const loadList = useCallback(async (preferredId?: string) => {
    listAbort.current?.abort();
    const controller = new AbortController();
    listAbort.current = controller;
    setListLoading(true);
    setMessage("");
    try {
      const response = await fetch("/api/scenarios", { cache: "no-store", signal: controller.signal });
      const result = await responseJson<{ scenarios: Scenario[] }>(response);
      const ordered = [...result.scenarios].sort((a, b) => b.created_at.localeCompare(a.created_at));
      setScenarios(ordered);
      setAvailable(true);
      setSelectedId((current) => {
        if (preferredId && ordered.some((item) => item.scenario_id === preferredId)) return preferredId;
        if (ordered.some((item) => item.scenario_id === current)) return current;
        return ordered[0]?.scenario_id ?? "";
      });
      if (ordered.length === 0) setMessage("저장된 운영 기록이 없습니다.");
    } catch (error) {
      if (controller.signal.aborted) return;
      if (errorStatus(error) === 404) markUnavailable();
      else setMessage(error instanceof Error ? `${error.message} 잠시 후 다시 확인해 주세요.` : "운영 기록을 불러오지 못했습니다.");
    } finally {
      if (!controller.signal.aborted) setListLoading(false);
    }
  }, [markUnavailable]);

  useEffect(() => {
    void loadList();
    return () => {
      listAbort.current?.abort();
      frameAbort.current?.abort();
      saveAbort.current?.abort();
    };
  }, [loadList]);

  const fetchFrame = useCallback(async (scenarioId: string, index: number, signal: AbortSignal) => {
    const query = new URLSearchParams({ scenario_id: scenarioId, frame_index: String(index) });
    const response = await fetch(`/api/scenarios?${query}`, { cache: "no-store", signal });
    return responseJson<ReplaySnapshot>(response);
  }, []);

  const loadFrame = useCallback(async (index: number) => {
    if (!selected || index < 0 || index >= selected.frame_count) return;
    frameAbort.current?.abort();
    const controller = new AbortController();
    frameAbort.current = controller;
    const requestId = ++frameRequest.current;
    setFrameLoading(true);
    setMessage("");
    try {
      const previousNeeded = index > 0 && simTimes[index - 1] === undefined;
      const [frame, previous] = await Promise.all([
        fetchFrame(selected.scenario_id, index, controller.signal),
        previousNeeded ? fetchFrame(selected.scenario_id, index - 1, controller.signal) : Promise.resolve(null),
      ]);
      if (controller.signal.aborted || requestId !== frameRequest.current) return;
      setCurrentIndex(index);
      setSimTimes((current) => ({
        ...current,
        ...(previous ? { [index - 1]: previous.sim_time_s } : {}),
        [index]: frame.sim_time_s,
      }));
      onReplay(frame);
    } catch (error) {
      if (controller.signal.aborted) return;
      setPlaying(false);
      if (errorStatus(error) === 404) markUnavailable();
      else setMessage(error instanceof Error ? `${error.message} 다른 기록이나 프레임을 선택해 주세요.` : "프레임을 불러오지 못했습니다.");
    } finally {
      if (!controller.signal.aborted && requestId === frameRequest.current) setFrameLoading(false);
    }
  }, [fetchFrame, markUnavailable, onReplay, selected, simTimes]);

  useEffect(() => {
    if (!playing || frameLoading || !selected || replaySnapshot === null) return;
    if (replayIndex >= selected.frame_count - 1) {
      setPlaying(false);
      return;
    }
    const timer = window.setTimeout(() => void loadFrame(replayIndex + 1), PLAYBACK_INTERVAL_MS);
    return () => window.clearTimeout(timer);
  }, [frameLoading, loadFrame, playing, replayIndex, replaySnapshot, selected]);

  async function saveScenario(event: FormEvent<HTMLFormElement>) {
    event.preventDefault();
    if (!canSave) return;
    saveAbort.current?.abort();
    const controller = new AbortController();
    saveAbort.current = controller;
    setSaving(true);
    setMessage("");
    try {
      const response = await fetch("/api/scenarios", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name: name.trim() }),
        signal: controller.signal,
      });
      const saved = await responseJson<Scenario>(response);
      await loadList(saved.scenario_id);
      setMessage(`“${saved.name}” 기록을 ${saved.frame_count}개 관측 표본으로 저장했습니다.`);
    } catch (error) {
      if (controller.signal.aborted) return;
      if (errorStatus(error) === 404) markUnavailable();
      else setMessage(error instanceof Error ? `${error.message} 연결 상태와 기록 이름을 확인해 주세요.` : "운영 기록을 저장하지 못했습니다.");
    } finally {
      if (!controller.signal.aborted) setSaving(false);
    }
  }

  function chooseScenario(nextId: string) {
    frameAbort.current?.abort();
    frameRequest.current += 1;
    setSelectedId(nextId);
    setCurrentIndex(0);
    setSimTimes({});
    setPlaying(false);
    setFrameLoading(false);
    onReplay(null);
  }

  function returnLive() {
    frameAbort.current?.abort();
    frameRequest.current += 1;
    setPlaying(false);
    setFrameLoading(false);
    onReplay(null);
    setMessage("실시간 관측으로 돌아왔습니다.");
  }

  const currentSimTime = replaySnapshot?.mode === "replay" ? replaySnapshot.sim_time_s : simTimes[replayIndex];
  const previousSimTime = replayIndex > 0 ? simTimes[replayIndex - 1] : undefined;
  const interval = currentSimTime !== undefined && previousSimTime !== undefined
    ? currentSimTime - previousSimTime
    : null;

  return (
    <section className="scenario-panel" aria-labelledby="scenario-title">
      <div className="scenario-heading">
        <div>
          <span className="eyebrow">OPERATION RECORD</span>
          <h2 id="scenario-title">운영 기록 저장·재생</h2>
        </div>
        <span className={`scenario-state ${replaySnapshot ? "is-replay" : ""}`}>
          {replaySnapshot ? "기록 재생 · 조작 비활성" : connectedLive ? "실시간 관측" : "저장 대기"}
        </span>
      </div>

      {available === false ? (
        <div className="scenario-unavailable">
          <p>{message}</p>
          <button type="button" className="secondary" onClick={() => void loadList()} disabled={listLoading}>
            {listLoading ? "확인 중" : "연결 다시 확인"}
          </button>
        </div>
      ) : (
        <div className="scenario-grid">
          <form className="scenario-save" onSubmit={saveScenario}>
            <label htmlFor="scenario-name">기록 이름</label>
            <div className="scenario-input-row">
              <input
                id="scenario-name"
                value={name}
                maxLength={80}
                onChange={(event) => setName(event.target.value)}
                disabled={available !== true || saving}
                aria-describedby="scenario-save-help"
              />
              <button className="primary" type="submit" disabled={!canSave}>
                {saving ? "저장 중" : "최근 관측 저장"}
              </button>
            </div>
            <small id="scenario-save-help">
              최근 관측 최대 120개를 보존합니다. {replaySnapshot ? "재생을 끝내야 새 기록을 저장할 수 있습니다." : !connectedLive ? "연결된 실시간 관측이 필요합니다." : `${name.length}/80자`}
            </small>
          </form>

          <div className="scenario-picker">
            <label htmlFor="scenario-list">저장된 기록</label>
            <div className="scenario-input-row">
              <select
                id="scenario-list"
                value={selectedId}
                onChange={(event) => chooseScenario(event.target.value)}
                disabled={listLoading || scenarios.length === 0}
              >
                {scenarios.length === 0 && <option value="">저장된 기록 없음</option>}
                {scenarios.map((scenario) => (
                  <option value={scenario.scenario_id} key={scenario.scenario_id}>
                    {scenario.name} · {scenarioDate.format(new Date(scenario.created_at))} · {scenario.frame_count}개 표본
                  </option>
                ))}
              </select>
              <button
                type="button"
                className="secondary"
                onClick={() => void loadFrame(0)}
                disabled={!selected || frameLoading}
              >
                {frameLoading && replaySnapshot === null ? "불러오는 중" : "기록 불러오기"}
              </button>
            </div>
          </div>
        </div>
      )}

      {available !== false && selected && (
        <div className="scenario-timeline">
          <div className="scenario-frame-status">
            <strong>{replaySnapshot ? `${replayIndex + 1} / ${frameCount} 표본` : "기록을 불러오세요"}</strong>
            <span>
              {currentSimTime === undefined ? "시뮬레이션 시각 미선택" : `시뮬레이션 ${currentSimTime.toFixed(1)}초`}
              {interval !== null && ` · 이전 표본과 ${interval.toFixed(1)}초 간격`}
            </span>
          </div>
          <input
            className="scenario-range"
            type="range"
            min={0}
            max={Math.max(selected.frame_count - 1, 0)}
            step={1}
            value={replayIndex}
            onChange={(event) => void loadFrame(Number(event.target.value))}
            disabled={replaySnapshot === null || frameLoading || selected.frame_count < 2}
            aria-label="운영 기록 프레임"
            aria-valuetext={`${replayIndex + 1} / ${selected.frame_count} 표본`}
          />
          <div className="scenario-controls">
            <button type="button" onClick={() => void loadFrame(replayIndex - 1)} disabled={replaySnapshot === null || frameLoading || replayIndex <= 0}>이전 표본</button>
            <button
              type="button"
              className="primary"
              onClick={() => setPlaying((value) => !value)}
              disabled={replaySnapshot === null || frameLoading || replayIndex >= selected.frame_count - 1}
              aria-pressed={playing}
            >
              {playing ? "재생 정지" : "표본 재생"}
            </button>
            <button type="button" onClick={() => void loadFrame(replayIndex + 1)} disabled={replaySnapshot === null || frameLoading || replayIndex >= selected.frame_count - 1}>다음 표본</button>
            <button type="button" className="secondary" onClick={returnLive} disabled={replaySnapshot === null}>실시간으로 돌아가기</button>
          </div>
          <small>고정 간격으로 저장된 관측 표본을 넘겨봅니다. 물리 시뮬레이션 재실행이나 원속도 영상이 아닙니다.</small>
        </div>
      )}

      {available !== false && (
        <div className="scenario-message">
          <span role="status" aria-live="polite">{listLoading ? "운영 기록을 확인하고 있습니다." : frameLoading ? "선택한 표본을 불러오고 있습니다." : message}</span>
          {available === null && !listLoading && (
            <button type="button" className="text-button" onClick={() => void loadList()}>목록 다시 확인</button>
          )}
        </div>
      )}
    </section>
  );
}
