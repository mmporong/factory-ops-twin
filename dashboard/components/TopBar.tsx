"use client";
import { forwardRef } from "react";
import Icon from "./Icon";
import SearchBox from "./SearchBox";
import { formatClock, type SearchItem, type Tone } from "../lib/ops-view";
import { sceneSize, type Scene, type Selection, type Snapshot } from "../lib/types";

type Props = {
  scene: Scene | null;
  snapshot: Snapshot | null;
  status: { label: string; tone: Tone; detail: string };
  searchItems: SearchItem[];
  onPick: (selection: Selection) => void;
  onOverview: () => void;
  quickAction: { label: string; title: string; paused: boolean; disabled: boolean; run: () => void } | null;
  drawerOpen: boolean;
  onToggleDrawer: () => void;
};

export default forwardRef<HTMLButtonElement, Props>(function TopBar(
  { scene, snapshot, status, searchItems, onPick, onOverview, quickAction, drawerOpen, onToggleDrawer },
  drawerButton,
) {
  return (
    <header className="topbar">
      <a className="brand" href="/" aria-label="FactoryOps 관제 처음 화면">
        <span className="brand-mark" aria-hidden="true"><Icon name="box" size={20} /></span>
        <span className="brand-name">Factory<span>Ops</span></span>
      </a>
      <SearchBox items={searchItems} onPick={onPick} />
      <div className="topbar-spacer" />
      <button type="button" className="site-chip" onClick={onOverview} aria-label="검사·포장 구역 전체 보기">
        <span className="site-code" aria-hidden="true"><Icon name="factory" size={17} /></span>
        <span className="site-text">
          <strong>검사·포장 구역</strong>
          <small>{sceneSize(scene)} · AMR {snapshot?.robots.length ?? 4}대</small>
        </span>
        <Icon name="home" size={16} />
      </button>
      <div className={`live-pill tone-${status.tone}`} role="status" title={status.detail}>
        <i aria-hidden="true" />
        <strong>{status.label}</strong>
        <span className="live-clock">
          <span className="sr-only">시뮬레이션 시각 </span>
          {snapshot ? formatClock(snapshot.sim_time_s) : "--:--"}
        </span>
      </div>
      {quickAction && (
        <button type="button" className={`quick-action${quickAction.paused ? " is-paused" : ""}`} title={quickAction.title} disabled={quickAction.disabled} onClick={quickAction.run}>
          <Icon name={quickAction.paused ? "play" : "pause"} size={16} />
          <span>{quickAction.label}</span>
        </button>
      )}
      <button
        ref={drawerButton}
        type="button"
        className="drawer-toggle"
        aria-expanded={drawerOpen}
        aria-controls="ops-drawer"
        onClick={onToggleDrawer}
      >
        <Icon name="sliders" size={17} />
        <span>운영 패널</span>
      </button>
    </header>
  );
});
