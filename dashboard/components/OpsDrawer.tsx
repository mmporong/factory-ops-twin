"use client";
import { useEffect, useRef, type KeyboardEvent, type ReactNode } from "react";
import Icon, { type IconName } from "./Icon";

export type DrawerTab = "control" | "record" | "evidence";
const tabs: { id: DrawerTab; label: string; icon: IconName }[] = [
  { id: "control", label: "운영 제어", icon: "sliders" },
  { id: "record", label: "기록 재생", icon: "record" },
  { id: "evidence", label: "관측 근거", icon: "shield" },
];

type Props = {
  open: boolean;
  tab: DrawerTab;
  onTab: (tab: DrawerTab) => void;
  onClose: () => void;
  panels: Record<DrawerTab, ReactNode>;
};

/** Non-modal side panel; every tab stays mounted so replay state survives tab switches. */
export default function OpsDrawer({ open, tab, onTab, onClose, panels }: Props) {
  const heading = useRef<HTMLHeadingElement>(null);
  const tabRefs = useRef<Record<string, HTMLButtonElement | null>>({});
  useEffect(() => {
    if (open) heading.current?.focus();
  }, [open]);

  function onTabKey(e: KeyboardEvent<HTMLButtonElement>) {
    const i = tabs.findIndex((t) => t.id === tab);
    const next = e.key === "ArrowRight" ? i + 1 : e.key === "ArrowLeft" ? i - 1 : null;
    if (next === null) return;
    e.preventDefault();
    const target = tabs[(next + tabs.length) % tabs.length];
    onTab(target.id);
    tabRefs.current[target.id]?.focus();
  }

  return (
    <aside
      id="ops-drawer"
      className="drawer"
      hidden={!open}
      aria-labelledby="drawer-title"
      onKeyDown={(e) => {
        if (e.key === "Escape") {
          e.stopPropagation();
          onClose();
        }
      }}
    >
      <header className="drawer-head">
        <h2 id="drawer-title" ref={heading} tabIndex={-1}>운영 패널</h2>
        <button type="button" className="icon-button" onClick={onClose} aria-label="운영 패널 닫기" title="닫기">
          <Icon name="close" />
        </button>
      </header>
      <div className="drawer-tabs" role="tablist" aria-label="운영 패널 구분">
        {tabs.map((t) => (
          <button
            key={t.id}
            ref={(el) => {
              tabRefs.current[t.id] = el;
            }}
            type="button"
            role="tab"
            id={`drawer-tab-${t.id}`}
            aria-selected={tab === t.id}
            aria-controls={`drawer-panel-${t.id}`}
            tabIndex={tab === t.id ? 0 : -1}
            onClick={() => onTab(t.id)}
            onKeyDown={onTabKey}
          >
            <Icon name={t.icon} size={16} />
            {t.label}
          </button>
        ))}
      </div>
      {tabs.map((t) => (
        <div
          key={t.id}
          className="drawer-panel"
          role="tabpanel"
          id={`drawer-panel-${t.id}`}
          aria-labelledby={`drawer-tab-${t.id}`}
          hidden={tab !== t.id}
        >
          {panels[t.id]}
        </div>
      ))}
    </aside>
  );
}
