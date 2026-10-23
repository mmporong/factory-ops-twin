"use client";
import { useEffect, useRef, useState } from "react";
import Icon, { type IconName } from "./Icon";
import type { CameraApi, Layers } from "./FactoryScene";

type Props = {
  camera: () => CameraApi | null;
  canFocus: boolean;
  onFocus: () => void;
  layers: Layers;
  onLayers: (layers: Layers) => void;
};

const layerNames: [keyof Layers, string][] = [
  ["zones", "공정 구역 색"],
  ["parking", "대기면·충전대"],
  ["routes", "진행 경로"],
  ["inventory", "원장 재고 도식"],
];

export default function MapControls({ camera, canFocus, onFocus, layers, onLayers }: Props) {
  const [open, setOpen] = useState(false);
  const toggle = useRef<HTMLButtonElement>(null);
  const panel = useRef<HTMLFieldSetElement>(null);

  useEffect(() => {
    if (!open) return;
    function onDown(e: PointerEvent) {
      if (!panel.current?.contains(e.target as Node) && !toggle.current?.contains(e.target as Node)) setOpen(false);
    }
    window.addEventListener("pointerdown", onDown);
    return () => window.removeEventListener("pointerdown", onDown);
  }, [open]);

  const tools: { icon: IconName; label: string; run: () => void; disabled?: boolean }[] = [
    { icon: "plus", label: "확대", run: () => camera()?.zoom(0.72) },
    { icon: "minus", label: "축소", run: () => camera()?.zoom(1.38) },
    { icon: "rotateLeft", label: "왼쪽으로 회전", run: () => camera()?.rotate(-Math.PI / 4) },
    { icon: "rotateRight", label: "오른쪽으로 회전", run: () => camera()?.rotate(Math.PI / 4) },
    { icon: "home", label: "공장 전체 보기", run: () => camera()?.home() },
    { icon: "focus", label: "선택 대상으로 이동", run: onFocus, disabled: !canFocus },
  ];

  return (
    <div className="map-controls">
      <div className="tool-stack" role="toolbar" aria-label="3D 시점 조작" aria-orientation="vertical">
        {tools.map((t) => (
          <button key={t.label} type="button" className="icon-button" aria-label={t.label} title={t.label} onClick={t.run} disabled={t.disabled}>
            <Icon name={t.icon} />
          </button>
        ))}
        <button
          ref={toggle}
          type="button"
          className="icon-button"
          aria-label="표시 항목"
          title="표시 항목"
          aria-expanded={open}
          aria-controls="layer-panel"
          onClick={() => setOpen((v) => !v)}
        >
          <Icon name="layers" />
        </button>
      </div>
      {open && (
        <fieldset
          id="layer-panel"
          ref={panel}
          className="layer-panel card"
          onKeyDown={(e) => {
            if (e.key === "Escape") {
              setOpen(false);
              toggle.current?.focus();
            }
          }}
        >
          <legend>표시 항목</legend>
          {layerNames.map(([key, label]) => (
            <label key={key}>
              <input type="checkbox" checked={layers[key]} onChange={(e) => onLayers({ ...layers, [key]: e.target.checked })} />
              {label}
            </label>
          ))}
          <p>드래그로 회전, 휠로 확대합니다. 오른쪽 드래그는 이동입니다.</p>
          <p>랙 위 재고·토트는 원장 기반 도식이며 물리 위치는 관측하지 않습니다.</p>
        </fieldset>
      )}
    </div>
  );
}
