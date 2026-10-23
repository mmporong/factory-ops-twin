"use client";
import { Canvas, useFrame, useThree, type ThreeEvent } from "@react-three/fiber";
import {
  Billboard,
  Edges,
  Environment,
  Grid,
  Html,
  Lightformer,
  Line,
  OrbitControls,
  RoundedBox,
  useCursor,
} from "@react-three/drei";
import type { Line2, LineSegments2, OrbitControls as OrbitControlsImpl } from "three-stdlib";
import type { Scene, Snapshot, Selection, Rect, Robot } from "../lib/types";
import { readable, stationNames } from "../lib/types";
import { inventoryDisplay } from "../lib/inventory-display";
import { laneArrows } from "../lib/traffic-lanes";
import {
  equipmentLabel,
  pathAhead,
  robotState,
  routeChange,
  selectionPoint,
  toneOf,
  zoneColor,
  type RouteChange,
  type Tone,
} from "../lib/ops-view";
import { RoundedBoxGeometry } from "three/examples/jsm/geometries/RoundedBoxGeometry.js";
import {
  memo,
  useCallback,
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
  type MutableRefObject,
} from "react";
import {
  BoxGeometry,
  BufferGeometry,
  CanvasTexture,
  DoubleSide,
  Float32BufferAttribute,
  NeutralToneMapping,
  PCFSoftShadowMap,
  SRGBColorSpace,
  Spherical,
  Vector3,
  type Group,
  type PerspectiveCamera,
  type Texture,
} from "three";

export type CameraApi = {
  zoom: (factor: number) => void;
  rotate: (radians: number) => void;
  home: () => void;
  focusOn: (selection: Selection) => boolean;
};
export type Layers = { zones: boolean; parking: boolean; routes: boolean; inventory: boolean };
export type Insets = { top: number; right: number; bottom: number; left: number };

type Props = {
  scene: Scene;
  snapshot: Snapshot | null;
  selection: Selection | null;
  onSelect: (s: Selection | null) => void;
  layers: Layers;
  insets: Insets;
  cameraRef: MutableRefObject<CameraApi | null>;
};

const palette = {
  ground: "#e4e8f3",
  slab: "#f3f4f9",
  floor: "#eef0f6",
  aisle: "#dde2ef",
  wall: "#f9fafd",
  accent: "#2f5bea",
  steel: "#39445e",
  amr: "#f4b42c",
  select: "#2f5bea",
};
const toneColor: Record<Tone, string> = {
  blue: "#2f5bea",
  green: "#17a06b",
  amber: "#e8920f",
  gray: "#8a93a8",
  red: "#e0453a",
};
const HOME_THETA = 0.72;
const HOME_PHI = 0.9;
const wrapAngle = (a: number) => Math.atan2(Math.sin(a), Math.cos(a));
const reducedMotion = () =>
  typeof window !== "undefined" && window.matchMedia("(prefers-reduced-motion: reduce)").matches;

const cargoPartGeometry = new BoxGeometry(1, 1, 1);
const cartonGeometry = new RoundedBoxGeometry(0.41, 0.3, 0.31, 2, 0.012);
// Flat chevron lying on the floor, tip at the origin pointing along +x.
const chevronGeometry = new BufferGeometry();
chevronGeometry.setAttribute("position", new Float32BufferAttribute([0, 0, 0, -0.58, 0, -0.34, -0.38, 0, 0, -0.58, 0, 0.34], 3));
chevronGeometry.setIndex([0, 1, 2, 0, 2, 3]);

function CargoPart({ size, position, color }: { size: [number, number, number]; position: [number, number, number]; color: string }) {
  return <mesh geometry={cargoPartGeometry} scale={size} position={position} castShadow receiveShadow><meshStandardMaterial color={color} roughness={0.65} /></mesh>;
}

function SoftBox({
  size,
  position,
  color,
  radius = 0.12,
  roughness = 0.45,
  metalness = 0.05,
}: {
  size: [number, number, number];
  position: [number, number, number];
  color: string;
  radius?: number;
  roughness?: number;
  metalness?: number;
}) {
  return (
    <RoundedBox
      args={size}
      position={position}
      radius={Math.min(radius, ...size.map((n) => n * 0.45))}
      smoothness={3}
      bevelSegments={3}
      castShadow
      receiveShadow
    >
      <meshPhysicalMaterial color={color} roughness={roughness} metalness={metalness} clearcoat={0.18} clearcoatRoughness={0.45} />
    </RoundedBox>
  );
}

/** Screen-space status tag that follows a 3D anchor; list rows carry the accessible copy. */
function Tag({
  position,
  title,
  note,
  tone = "gray",
  selected = false,
  small = false,
}: {
  position: [number, number, number];
  title: string;
  note?: string;
  tone?: Tone;
  selected?: boolean;
  small?: boolean;
}) {
  return (
    <Html position={position} center zIndexRange={[8, 0]} style={{ pointerEvents: "none" }}>
      <div aria-hidden="true" className={`map-tag tone-${tone}${selected ? " is-selected" : ""}${small ? " is-small" : ""}`}>
        <i />
        <strong>{title}</strong>
        {note && <span>{note}</span>}
      </div>
    </Html>
  );
}

function SelectionBox({ size, y }: { size: [number, number, number]; y: number }) {
  return (
    <mesh position={[0, y, 0]} raycast={() => null}>
      <boxGeometry args={size} />
      <meshBasicMaterial color={palette.select} transparent opacity={0.08} depthWrite={false} />
      <Edges color={palette.select} lineWidth={1.6} />
    </mesh>
  );
}

function Pin({ x, y, scale = 1 }: { x: number; y: number; scale?: number }) {
  return (
    <group position={[x, 0, -y]} scale={scale}>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.035, 0]} raycast={() => null}>
        <ringGeometry args={[0.55, 0.7, 48]} />
        <meshBasicMaterial color={palette.accent} transparent opacity={0.6} depthWrite={false} />
      </mesh>
      <mesh position={[0, 1.95, 0]} rotation={[Math.PI, 0, 0]} castShadow raycast={() => null}>
        <coneGeometry args={[0.31, 0.78, 32]} />
        <meshStandardMaterial color={palette.accent} roughness={0.35} />
      </mesh>
      <mesh position={[0, 2.42, 0]} castShadow raycast={() => null}>
        <sphereGeometry args={[0.38, 32, 16]} />
        <meshStandardMaterial color={palette.accent} roughness={0.35} />
      </mesh>
      <Billboard position={[0, 2.44, 0]}>
        <mesh position={[0, 0, 0.385]} raycast={() => null}>
          <circleGeometry args={[0.15, 24]} />
          <meshBasicMaterial color="#ffffff" />
        </mesh>
      </Billboard>
    </group>
  );
}

function CameraRig({
  scene,
  snapshot,
  insets,
  apiRef,
}: {
  scene: Scene;
  snapshot: Snapshot | null;
  insets: Insets;
  apiRef: MutableRefObject<CameraApi | null>;
}) {
  const camera = useThree((s) => s.camera) as PerspectiveCamera;
  const controls = useThree((s) => s.controls) as OrbitControlsImpl | null;
  const size = useThree((s) => s.size);
  const invalidate = useThree((s) => s.invalidate);
  const goal = useRef<{ target: Vector3; radius: number; theta: number; phi: number } | null>(null);
  const moved = useRef(false);
  const snapshotRef = useRef(snapshot);
  snapshotRef.current = snapshot;
  const [a, b, c, d] = scene.bounds_xy;

  const home = useCallback(() => {
    const w = c - a;
    const h = d - b;
    const freeW = Math.max(260, size.width - insets.left - insets.right) / size.width;
    const freeH = Math.max(220, size.height - insets.top - insets.bottom) / size.height;
    const tanV = Math.tan((camera.fov * Math.PI) / 360);
    const tanH = tanV * (size.width / Math.max(1, size.height));
    const ex = w * Math.abs(Math.cos(HOME_THETA)) + h * Math.abs(Math.sin(HOME_THETA));
    const ey = (w * Math.abs(Math.sin(HOME_THETA)) + h * Math.abs(Math.cos(HOME_THETA))) * Math.cos(HOME_PHI) + 3 * Math.sin(HOME_PHI);
    // With floating side panels the far corners may run under them; a stacked layout shows the whole floor.
    const radius = Math.max(ex / (2 * tanH * freeW), ey / (2 * tanV * freeH)) * (insets.right > 0 ? 0.84 : 1.02);
    return { target: new Vector3((a + c) / 2, 0, -(b + d) / 2), radius, theta: HOME_THETA, phi: HOME_PHI };
  }, [a, b, c, d, camera.fov, size.width, size.height, insets.left, insets.right, insets.top, insets.bottom]);

  const current = useCallback(() => {
    if (goal.current) return { ...goal.current, target: goal.current.target.clone() };
    const target = controls ? controls.target.clone() : new Vector3();
    const s = new Spherical().setFromVector3(camera.position.clone().sub(target));
    return { target, radius: s.radius, theta: s.theta, phi: s.phi };
  }, [camera, controls]);

  const place = useCallback(
    (g: { target: Vector3; radius: number; theta: number; phi: number }) => {
      if (!controls) return;
      controls.target.copy(g.target);
      camera.position.copy(g.target).add(new Vector3().setFromSpherical(new Spherical(g.radius, g.phi, g.theta)));
      camera.lookAt(g.target);
      controls.update();
      invalidate();
    },
    [camera, controls, invalidate],
  );

  const animate = useCallback(
    (g: { target: Vector3; radius: number; theta: number; phi: number }) => {
      if (reducedMotion()) {
        goal.current = null;
        place(g);
      } else {
        goal.current = g;
        invalidate();
      }
    },
    [invalidate, place],
  );

  useLayoutEffect(() => {
    camera.setViewOffset(
      size.width,
      size.height,
      (insets.right - insets.left) / 2,
      (insets.bottom - insets.top) / 2,
      size.width,
      size.height,
    );
    camera.updateProjectionMatrix();
    invalidate();
  }, [camera, size.width, size.height, insets.top, insets.right, insets.bottom, insets.left, invalidate]);

  useEffect(() => {
    if (!controls) return;
    if (!moved.current) place(home());
    const stop = () => {
      moved.current = true;
      goal.current = null;
    };
    controls.addEventListener("start", stop);
    return () => controls.removeEventListener("start", stop);
  }, [controls, home, place]);

  useEffect(() => {
    apiRef.current = {
      zoom: (factor) => {
        const now = current();
        moved.current = true;
        animate({ ...now, radius: Math.min(home().radius * 1.8, Math.max(5, now.radius * factor)) });
      },
      rotate: (radians) => {
        const now = current();
        moved.current = true;
        animate({ ...now, theta: now.theta + radians });
      },
      home: () => {
        moved.current = false;
        animate(home());
      },
      focusOn: (selection) => {
        const point = selectionPoint(scene, snapshotRef.current, selection);
        if (!point) return false;
        const now = current();
        moved.current = true;
        animate({
          target: new Vector3(point[0], 0.5, -point[1]),
          radius: Math.max(9, Math.min(now.radius, home().radius * 0.42)),
          theta: now.theta,
          phi: Math.min(now.phi, 0.98),
        });
        return true;
      },
    };
    return () => {
      apiRef.current = null;
    };
  }, [apiRef, animate, current, home, scene]);

  useFrame((_, delta) => {
    const g = goal.current;
    if (!g || !controls) return;
    const k = 1 - Math.exp(-Math.min(delta, 0.1) * 5.5);
    const s = new Spherical().setFromVector3(camera.position.clone().sub(controls.target));
    controls.target.lerp(g.target, k);
    s.radius += (g.radius - s.radius) * k;
    s.theta += wrapAngle(g.theta - s.theta) * k;
    s.phi += (g.phi - s.phi) * k;
    camera.position.copy(controls.target).add(new Vector3().setFromSpherical(s));
    camera.lookAt(controls.target);
    const settled =
      controls.target.distanceTo(g.target) < 0.01 &&
      Math.abs(g.radius - s.radius) < 0.02 &&
      Math.abs(wrapAngle(g.theta - s.theta)) < 0.001 &&
      Math.abs(g.phi - s.phi) < 0.001;
    if (settled) goal.current = null;
    controls.update();
    invalidate();
  });
  return null;
}

function FloorRect({ rect, color, y = 0, onClick }: { rect: Rect; color: string; y?: number; onClick?: (e: ThreeEvent<MouseEvent>) => void }) {
  const [a, b, c, d] = rect;
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[(a + c) / 2, y, -(b + d) / 2]} receiveShadow onClick={onClick}>
      <planeGeometry args={[c - a, d - b]} />
      <meshStandardMaterial color={color} roughness={0.9} />
    </mesh>
  );
}

function WorkAreaMark({ rect }: { rect: Rect }) {
  const [a, b, c, d] = rect;
  const inset = 0.06;
  const length = Math.min(0.5, (c - a) / 3, (d - b) / 3);
  return (
    <>
      {[
        [a + inset, b + inset, 1, 1],
        [c - inset, b + inset, -1, 1],
        [a + inset, d - inset, 1, -1],
        [c - inset, d - inset, -1, -1],
      ].map(([x, y, dx, dy], i) => (
        <Line
          key={i}
          points={[
            [x + dx * length, 0.035, -y],
            [x, 0.035, -y],
            [x, 0.035, -(y + dy * length)],
          ]}
          color="#e7ad2f"
          lineWidth={2.2}
        />
      ))}
    </>
  );
}

const ToteShape = memo(function ToteShape({ color }: { color: string }) {
  return (
    <>
      <CargoPart size={[0.64, 0.06, 0.49]} position={[0, -0.2, 0]} color={color} />
      {[-1, 1].map((side) => <group key={side}>
        <CargoPart size={[0.64, 0.42, 0.035]} position={[0, 0.015, side * 0.23]} color={color} />
        <CargoPart size={[0.035, 0.42, 0.45]} position={[side * 0.3, 0.015, 0]} color={color} />
        <CargoPart size={[0.68, 0.055, 0.05]} position={[0, 0.235, side * 0.25]} color={color} />
        <CargoPart size={[0.05, 0.055, 0.49]} position={[side * 0.32, 0.235, 0]} color={color} />
        <CargoPart size={[0.21, 0.05, 0.01]} position={[0, 0.14, side * 0.252]} color="#1d2a55" />
        {[-0.23, -0.1, 0.1, 0.23].map((x) => <CargoPart key={x} size={[0.02, 0.26, 0.018]} position={[x, -0.03, side * 0.252]} color={color} />)}
      </group>)}
      <CargoPart size={[0.54, 0.015, 0.39]} position={[0, -0.16, 0]} color="#22306a" />
    </>
  );
});

const Carton = memo(function Carton({ label, selected, variant = 0 }: { label: Texture; selected: boolean; variant?: number }) {
  const color = selected ? "#7d9cf5" : ["#c99a5f", "#d6ad73", "#b98a55", "#ddb983"][variant % 4];
  return <>
    <mesh geometry={cartonGeometry} castShadow receiveShadow><meshStandardMaterial color={color} roughness={0.9} /></mesh>
    <mesh position={[0, 0.152, 0]} rotation={[-Math.PI / 2, 0, 0]}>
      <planeGeometry args={[0.045, 0.3]} /><meshStandardMaterial color="#ead2a6" roughness={0.75} />
    </mesh>
    <mesh position={[0.08, -0.005, 0.157]}>
      <planeGeometry args={[0.18, 0.105]} /><meshBasicMaterial map={label} toneMapped={false} />
    </mesh>
  </>;
});

type HoverHandlers = {
  onOver: (key: string) => void;
  onOut: (key: string) => void;
};

const Equipment = memo(function Equipment({
  item,
  selected,
  hovered,
  onSelect,
  hover,
}: {
  item: Scene["physical_obstacles"][number];
  selected: boolean;
  hovered: boolean;
  onSelect: (s: Selection) => void;
  hover: HoverHandlers;
}) {
  const [a, b, c, d] = item.rect,
    w = c - a,
    z = d - b,
    h = item.height_m;
  const key = `equipment:${item.id}`;
  const rack = item.id.startsWith("RACK"),
    machine = item.id.startsWith("PROCESS"),
    fg = item.id.startsWith("FG"),
    bench = item.id.startsWith("BENCH") || item.id.startsWith("KIT");
  const trim = item.id === "PROCESS_2" ? "#119c88" : item.id === "PROCESS_3" ? "#f29b2f" : palette.accent;
  const frame = selected ? "#1d3fb8" : palette.accent;
  const surface = h > 1.3 ? h * 0.56 : h - 0.12;
  return (
    <group
      position={[(a + c) / 2, 0, -(b + d) / 2]}
      onClick={(e) => {
        e.stopPropagation();
        onSelect({ kind: "equipment", id: item.id });
      }}
      onPointerOver={(e) => {
        e.stopPropagation();
        hover.onOver(key);
      }}
      onPointerOut={() => hover.onOut(key)}
    >
      {rack ? (
        <>
          {[-w / 2 + 0.06, w / 2 - 0.06].flatMap((x) =>
            [-z / 2 + 0.06, z / 2 - 0.06].map((v) => (
              <SoftBox key={`${x}-${v}`} size={[0.1, h, 0.1]} position={[x, h / 2, v]} color={frame} radius={0.035} metalness={0.3} />
            )),
          )}
          {[0.18, h * 0.41, h * 0.73, h - 0.06].map((y) => (
            <group key={y}>
              <SoftBox size={[w - 0.12, 0.055, z - 0.08]} position={[0, y, 0]} color="#d5dbe8" radius={0.02} metalness={0.3} />
              {[-1, 1].map((side) => (
                <SoftBox key={side} size={[w, 0.1, 0.075]} position={[0, y - 0.03, side * (z / 2 - 0.04)]} color="#f29b2f" radius={0.022} roughness={0.5} />
              ))}
            </group>
          ))}
        </>
      ) : bench ? (
        <>
          <SoftBox size={[w, 0.2, z]} position={[0, surface - 0.1, 0]} radius={0.09} color={selected ? "#dbe5ff" : "#f7f8fc"} />
          <SoftBox size={[w * 0.93, 0.07, z * 0.91]} position={[0, surface - 0.22, 0]} radius={0.03} color="#e3e7f0" />
          {[-w / 2 + 0.16, w / 2 - 0.16].flatMap((x) =>
            [-z / 2 + 0.12, z / 2 - 0.12].map((v) => (
              <SoftBox key={`${x}-${v}`} size={[0.15, surface - 0.2, 0.15]} position={[x, (surface - 0.2) / 2, v]} color={palette.steel} radius={0.045} metalness={0.3} />
            )),
          )}
          {h > 1.3 && <SoftBox size={[w * 0.75, h * 0.36, 0.12]} position={[0, h * 0.79, -z * 0.36]} color="#cfd7ea" radius={0.05} />}
          <SoftBox size={[w * 0.28, 0.07, z * 0.34]} position={[-w * 0.24, surface + 0.025, 0]} color="#c8d0e0" radius={0.025} metalness={0.5} />
          <SoftBox size={[w * 0.14, 0.09, z * 0.2]} position={[w * 0.27, surface + 0.035, 0]} color={palette.accent} radius={0.025} />
        </>
      ) : fg ? (
        <>
          <SoftBox size={[w, 0.18, z]} position={[0, h * 0.55, 0]} color={selected ? "#1d3fb8" : palette.steel} radius={0.06} metalness={0.3} />
          {[-1, 1].map((side) => (
            <group key={side}>
              <SoftBox size={[w, 0.13, 0.1]} position={[0, h * 0.76, side * (z / 2 - 0.07)]} color={palette.accent} radius={0.035} />
              {[-w / 2 + 0.18, w / 2 - 0.18].map((x) => (
                <SoftBox key={x} size={[0.18, h * 0.55, 0.18]} position={[x, h * 0.275, side * (z / 2 - 0.18)]} color={palette.steel} radius={0.04} />
              ))}
            </group>
          ))}
          {Array.from({ length: Math.max(3, Math.floor(w / 0.3)) }, (_, i) => (
            <mesh key={i} position={[-w / 2 + 0.2 + i * (w - 0.4) / Math.max(2, Math.floor(w / 0.3) - 1), h * 0.65, 0]} rotation={[Math.PI / 2, 0, 0]} castShadow>
              <cylinderGeometry args={[0.075, 0.075, z - 0.3, 12]} />
              <meshStandardMaterial color="#c9d0dd" metalness={0.55} roughness={0.32} />
            </mesh>
          ))}
        </>
      ) : (
        <>
          <SoftBox size={[w, h - 0.2, z]} position={[0, h / 2 - 0.02, 0]} color={selected ? "#dbe5ff" : "#f6f7fb"} radius={machine ? 0.14 : 0.09} roughness={0.5} />
          <SoftBox size={[w * 0.91, 0.15, z * 0.89]} position={[0, 0.075, 0]} color={palette.steel} radius={0.065} metalness={0.2} />
          <SoftBox size={[w * 0.98, 0.2, z * 0.98]} position={[0, h - 0.1, 0]} color={machine ? trim : "#9aa6c0"} radius={0.065} />
          {machine && (
            <>
              <SoftBox size={[w * 0.69, h * 0.56, 0.1]} position={[-w * 0.09, h * 0.55, z / 2 - 0.055]} color="#c9d1e3" radius={0.045} />
              <SoftBox size={[w * 0.61, h * 0.48, 0.055]} position={[-w * 0.09, h * 0.55, z / 2 - 0.025]} color="#1f2a44" radius={0.046} roughness={0.16} metalness={0.4} />
              <SoftBox size={[w * 0.76, 0.08, 0.06]} position={[0, h * 0.2, z / 2 - 0.025]} color={trim} radius={0.025} />
              <SoftBox size={[w * 0.13, h * 0.2, 0.055]} position={[w * 0.35, h * 0.66, z / 2 - 0.025]} color="#1f2a44" radius={0.035} />
              <SoftBox size={[w * 0.09, h * 0.11, 0.018]} position={[w * 0.35, h * 0.68, z / 2 - 0.015]} color="#7fa2ff" radius={0.015} />
              {[-0.42, 0.42].map((x) => (
                <SoftBox key={x} size={[0.14, h * 0.18, 0.1]} position={[w * x, h * 0.15, z / 2 - 0.055]} color="#f2b02f" radius={0.035} />
              ))}
            </>
          )}
          {!machine && [-0.25, 0.25].map((x) => (
            <group key={x}>
              <SoftBox size={[w * 0.46, h * 0.72, 0.04]} position={[w * x, h * 0.5, z / 2 - 0.03]} color="#e4e8f1" radius={0.04} />
              <SoftBox size={[0.045, h * 0.18, 0.055]} position={[w * (x + 0.14), h * 0.56, z / 2 - 0.03]} color={palette.steel} radius={0.015} />
            </group>
          ))}
        </>
      )}
      {selected && <SelectionBox size={[w + 0.3, h + 0.3, z + 0.3]} y={(h + 0.3) / 2} />}
      {(selected || hovered) && <Tag position={[0, h + 0.55, 0]} title={equipmentLabel(item)} note={item.id} selected={selected} tone="blue" />}
    </group>
  );
});

type MapPoint = Robot["route"][number];
type RouteShift = { kind: RouteChange; from: MapPoint[]; to: MapPoint[]; at: number };
const AHEAD_M = 3.2; // about one and a half floor grid cells past the bumper
const BODY_M = 0.7;
const SHIFT_MS = 3200;
const shiftLabel: Record<RouteChange, string> = { replanned: "경로 재계획", redirected: "목적지 변경" };
const linePlaceholder: [number, number, number][] = [[0, 0, 0], [0, 0, 0.01]];
const toWorld = (points: MapPoint[], y: number) => points.map((p) => [p.x_m, y, -p.y_m] as [number, number, number]);

/** Marching dashed arrow over the next stretch of the route, starting at the body's tweened position. */
function PathAhead({
  body,
  pose,
  route,
  moving,
}: {
  body: MutableRefObject<Group | null>;
  pose: MapPoint;
  route: MapPoint[];
  moving: boolean;
}) {
  const line = useRef<Line2 | LineSegments2>(null);
  const head = useRef<Group>(null);
  const invalidate = useThree((s) => s.invalidate);
  const drawn = useRef<{ x: number; z: number; route: MapPoint[] | null }>({ x: NaN, z: NaN, route: null });
  useEffect(() => invalidate(), [route, moving, invalidate]);
  useFrame((_, delta) => {
    const g = body.current;
    const l = line.current;
    const h = head.current;
    if (!g || !l || !h) return;
    const { x, z } = g.position;
    if (x !== drawn.current.x || z !== drawn.current.z || route !== drawn.current.route) {
      drawn.current = { x, z, route };
      // The body tweens toward the observed pose, so the path runs through that pose first.
      const points = pathAhead({ x_m: x, y_m: -z }, [pose, ...route], AHEAD_M, BODY_M);
      l.visible = h.visible = points.length > 1;
      if (points.length > 1) {
        l.geometry.setPositions(toWorld(points, 0.11).flat());
        l.computeLineDistances();
        const [a, b] = points.slice(-2);
        h.position.set(b.x_m, 0.11, -b.y_m);
        h.rotation.y = Math.atan2(b.y_m - a.y_m, b.x_m - a.x_m);
      }
    }
    if (moving && !reducedMotion()) {
      l.material.dashOffset = (l.material.dashOffset - Math.min(delta, 0.1) * 1.4) % 0.7;
      invalidate();
    }
  });
  const color = moving ? palette.accent : toneColor.gray;
  return (
    <>
      <Line ref={line} points={linePlaceholder} color={color} lineWidth={4} dashed dashSize={0.42} gapSize={0.28} />
      <group ref={head}>
        <mesh geometry={chevronGeometry} raycast={() => null}>
          <meshBasicMaterial color={color} side={DoubleSide} toneMapped={false} />
        </mesh>
      </group>
    </>
  );
}

/** For a few seconds after an observed route change, the old route fades out under the new one. */
function RouteTrail({ shift }: { shift: RouteShift }) {
  const old = useRef<Line2 | LineSegments2>(null);
  const fresh = useRef<Line2 | LineSegments2>(null);
  const invalidate = useThree((s) => s.invalidate);
  const from = useMemo(() => toWorld(shift.from, 0.085), [shift]);
  const to = useMemo(() => toWorld(shift.to, 0.095), [shift]);
  useFrame(() => {
    if (reducedMotion()) return;
    // Hold both routes for the first 40 % of the window, then let the old one fade out.
    const fade = Math.min(1, Math.max(0, (1 - (performance.now() - shift.at) / SHIFT_MS) / 0.6));
    if (old.current) old.current.material.opacity = 0.85 * fade;
    if (fresh.current) fresh.current.material.opacity = 0.3 + 0.7 * fade;
    if (fade > 0) invalidate();
  });
  return (
    <>
      <Line ref={old} points={from} color="#6f7891" lineWidth={3.2} dashed dashSize={0.22} gapSize={0.22} transparent opacity={0.85} />
      <Line ref={fresh} points={to} color={toneColor.amber} lineWidth={3.6} transparent opacity={1} />
    </>
  );
}

const RobotUnit = memo(function RobotUnit({
  robot,
  selected,
  showPath,
  onSelect,
  hover,
  label,
}: {
  robot: Robot;
  selected: boolean;
  showPath: boolean;
  onSelect: (s: Selection) => void;
  hover: HoverHandlers;
  label: Texture;
}) {
  const group = useRef<Group>(null);
  const invalidate = useThree((s) => s.invalidate);
  const motion = useRef({ from: new Vector3(), to: new Vector3(), yawFrom: 0, yawTo: 0, t: 1, placed: false });
  const pose = robot.pose!;
  const state = robotState(robot);
  const tone = toneOf(state);
  const key = `robot:${robot.id}`;
  const [shift, setShift] = useState<RouteShift | null>(null);
  const seen = useRef<{ route: MapPoint[]; pose: MapPoint } | null>(null);

  useEffect(() => {
    const prev = seen.current;
    seen.current = { route: robot.route, pose };
    // A jump (replay seek, reconnect) is a new observation, not a change of plan.
    if (!prev || Math.hypot(prev.pose.x_m - pose.x_m, prev.pose.y_m - pose.y_m) > 4) return;
    const kind = routeChange(prev.route, robot.route, pose);
    if (kind) setShift({ kind, from: [pose, ...prev.route], to: [pose, ...robot.route], at: performance.now() });
  }, [robot.route, pose]);

  useEffect(() => {
    if (!shift) return;
    const timer = setTimeout(() => setShift(null), SHIFT_MS);
    return () => clearTimeout(timer);
  }, [shift]);

  useLayoutEffect(() => {
    const g = group.current;
    if (!g) return;
    const m = motion.current;
    const to = new Vector3(pose.x_m, 0.05, -pose.y_m);
    m.from.copy(g.position);
    m.yawFrom = g.rotation.y;
    m.to.copy(to);
    m.yawTo = pose.yaw_rad;
    // Short observed hops are tweened between polls; long jumps and reduced motion snap.
    if (!m.placed || m.from.distanceTo(to) > 4 || reducedMotion()) {
      g.position.copy(to);
      g.rotation.y = pose.yaw_rad;
      m.t = 1;
      m.placed = true;
    } else {
      m.t = 0;
    }
    invalidate();
  }, [pose.x_m, pose.y_m, pose.yaw_rad, invalidate]);

  useFrame((_, delta) => {
    const g = group.current;
    const m = motion.current;
    if (!g || m.t >= 1) return;
    m.t = Math.min(1, m.t + Math.min(delta, 0.1) / 1.35);
    g.position.lerpVectors(m.from, m.to, m.t);
    g.rotation.y = m.yawFrom + wrapAngle(m.yawTo - m.yawFrom) * Math.min(1, m.t * 2.5);
    invalidate();
  });

  return (
    <>
      <group
        ref={group}
        onClick={(e) => {
          e.stopPropagation();
          onSelect({ kind: "robot", id: robot.id });
        }}
        onPointerOver={(e) => {
          e.stopPropagation();
          hover.onOver(key);
        }}
        onPointerOut={() => hover.onOut(key)}
      >
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.012, 0]} raycast={() => null}>
          <ringGeometry args={[0.84, selected ? 1.04 : 0.95, 48]} />
          <meshBasicMaterial color={selected ? palette.select : tone === "gray" ? "#b9c2da" : toneColor[tone]} />
        </mesh>
        <SoftBox size={[1.08, 0.18, 0.8]} position={[0, 0.13, 0]} color="#262d40" radius={0.08} roughness={0.6} />
        <SoftBox size={[1.14, 0.38, 0.86]} position={[0, 0.36, 0]} color={palette.amr} radius={0.17} roughness={0.35} />
        <SoftBox size={[0.96, 0.08, 0.74]} position={[0, 0.58, 0]} color="#ffffff" radius={0.04} roughness={0.4} />
        <SoftBox size={[0.07, 0.09, 0.5]} position={[0.56, 0.34, 0]} color="#262d40" radius={0.03} />
        <mesh position={[-0.52, 0.42, 0]}>
          <boxGeometry args={[0.05, 0.08, 0.36]} />
          <meshBasicMaterial color={toneColor[tone]} toneMapped={false} />
        </mesh>
        {[-0.42, 0.42].flatMap((x) =>
          [-0.37, 0.37].map((z) => (
            <mesh key={`${x}-${z}`} position={[x, 0.14, z]} rotation={[Math.PI / 2, 0, 0]} castShadow>
              <cylinderGeometry args={[0.13, 0.13, 0.1, 20]} />
              <meshStandardMaterial color="#1c2232" roughness={0.75} />
            </mesh>
          )),
        )}
        <mesh position={[0.32, 0.7, 0]} castShadow>
          <cylinderGeometry args={[0.11, 0.13, 0.16, 24]} />
          <meshStandardMaterial color="#262d40" roughness={0.3} />
        </mesh>
        {robot.payload_id && (
          <group position={[-0.08, 0.86, 0]}>
            {robot.payload_id.startsWith("LOT") ? <group scale={1.9}><Carton label={label} selected={false} /></group> : <ToteShape color="#3d68ee" />}
          </group>
        )}
        {selected && <SelectionBox size={[1.55, 1.5, 1.25]} y={0.75} />}
        <Tag
          position={[0, robot.payload_id ? 1.85 : 1.35, 0]}
          title={robot.label}
          note={shift ? shiftLabel[shift.kind] : readable(state)}
          tone={shift ? "amber" : tone}
          selected={selected}
        />
      </group>
      {showPath && robot.route.length > 0 && (
        <PathAhead body={group} pose={pose} route={robot.route} moving={robot.status === "moving"} />
      )}
      {showPath && shift && <RouteTrail shift={shift} />}
    </>
  );
});

function Walls({ bounds }: { bounds: Rect }) {
  const [a, b, c, d] = bounds;
  const t = 0.28;
  const h = 0.72;
  const w = c - a;
  const depth = d - b;
  const segments: { size: [number, number, number]; position: [number, number, number] }[] = [
    { size: [w + 2 * t, h, t], position: [(a + c) / 2, h / 2, -b + t / 2] },
    { size: [w + 2 * t, h, t], position: [(a + c) / 2, h / 2, -d - t / 2] },
    { size: [t, h, depth], position: [a - t / 2, h / 2, -(b + d) / 2] },
    { size: [t, h, depth], position: [c + t / 2, h / 2, -(b + d) / 2] },
  ];
  return (
    <>
      {segments.map((s, i) => (
        <group key={i}>
          <SoftBox size={s.size} position={s.position} color={palette.wall} radius={0.06} roughness={0.6} />
          <SoftBox
            size={[s.size[0] + 0.02, 0.09, s.size[2] + 0.02]}
            position={[s.position[0], h + 0.02, s.position[2]]}
            color={palette.accent}
            radius={0.03}
          />
        </group>
      ))}
    </>
  );
}

export default memo(function FactoryScene({ scene, snapshot, selection, onSelect, layers, insets, cameraRef }: Props) {
  const [a, b, c, d] = scene.bounds_xy,
    cx = (a + c) / 2,
    cy = (b + d) / 2;
  const inventory = useMemo(() => inventoryDisplay(scene, snapshot), [scene, snapshot]);
  const [hovered, setHovered] = useState<string | null>(null);
  useCursor(hovered !== null);
  const hover = useMemo<HoverHandlers>(
    () => ({ onOver: (key) => setHovered(key), onOut: (key) => setHovered((h) => (h === key ? null : h)) }),
    [],
  );
  const select = useCallback((s: Selection) => onSelect(s), [onSelect]);
  const clearOnClick = useCallback(
    (e: ThreeEvent<MouseEvent>) => {
      if (e.delta < 5) onSelect(null);
    },
    [onSelect],
  );
  const selectedRobot = selection?.kind === "robot" ? snapshot?.robots.find((r) => r.id === selection.id) : undefined;
  const destination = selectedRobot?.destination ? scene.stations.find((s) => s.id === selectedRobot.destination) : undefined;
  // Every station some robot is currently heading to gets a small pin; the selected robot's is large.
  const activeDestinations = scene.stations.filter(
    (s) => s.id !== destination?.id && snapshot?.robots.some((r) => r.task_id && r.destination === s.id),
  );
  const packagingLabel = useMemo(() => {
    const canvas = document.createElement("canvas");
    canvas.width = 128;
    canvas.height = 64;
    const ctx = canvas.getContext("2d");
    if (ctx) {
      ctx.fillStyle = "#f6f4e9"; ctx.fillRect(0, 0, 128, 64);
      ctx.fillStyle = "#40525a"; ctx.font = "bold 14px Arial"; ctx.fillText("LOT", 8, 19);
      for (let i = 0; i < 30; i++) ctx.fillRect(8 + i * 3, 28, i % 3 === 0 ? 2 : 1, 23);
    }
    const texture = new CanvasTexture(canvas);
    texture.colorSpace = SRGBColorSpace;
    return texture;
  }, []);
  useEffect(() => () => packagingLabel.dispose(), [packagingLabel]);
  return (
    <Canvas
      aria-label={scene.traffic_lanes?.length ? "공장 관제 지도. 바닥 화살표는 지정된 차로 진행 방향입니다." : "공장 관제 지도"}
      shadows
      dpr={[1, 1.75]}
      frameloop="demand"
      camera={{ fov: 30, near: 0.5, far: 700, position: [cx + 60, 50, -cy + 60] }}
      gl={{ antialias: true, powerPreference: "high-performance" }}
      onCreated={({ gl }) => {
        gl.toneMapping = NeutralToneMapping;
        gl.toneMappingExposure = 1;
        gl.shadowMap.type = PCFSoftShadowMap;
      }}
      fallback={<div className="scene-fallback">3D 표시를 사용할 수 없습니다. 목록에서 상태를 확인하세요.</div>}
    >
      <color attach="background" args={[palette.ground]} />
      <fog attach="fog" args={[palette.ground, 140, 340]} />
      <OrbitControls
        makeDefault
        enableDamping
        dampingFactor={0.12}
        minDistance={5}
        maxDistance={220}
        minPolarAngle={0.3}
        maxPolarAngle={1.3}
      />
      <CameraRig scene={scene} snapshot={snapshot} insets={insets} apiRef={cameraRef} />
      <ambientLight intensity={0.25} />
      <hemisphereLight args={["#ffffff", "#c6cde3", 1.05]} />
      <Environment resolution={64} frames={1} environmentIntensity={0.35}>
        <Lightformer form="rect" intensity={3} color="#ffffff" position={[0, 14, 0]} rotation={[Math.PI / 2, 0, 0]} scale={[24, 16, 1]} />
        <Lightformer form="rect" intensity={1.6} color="#dfe7ff" position={[-16, 8, 5]} rotation={[0, Math.PI / 2, 0]} scale={[12, 14, 1]} />
      </Environment>
      <directionalLight
        position={[cx - 16, 38, -cy + 20]}
        target-position={[cx, 0, -cy]}
        intensity={1.9}
        color="#fffaf2"
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-left={-34}
        shadow-camera-right={34}
        shadow-camera-top={34}
        shadow-camera-bottom={-34}
        shadow-bias={-0.0005}
        shadow-normalBias={0.035}
        shadow-radius={5}
        onUpdate={(light) => light.target.updateMatrixWorld()}
      />
      <directionalLight position={[cx + 20, 18, -cy - 20]} intensity={0.45} color="#dbe4ff" />

      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[cx, -0.3, -cy]} receiveShadow onClick={clearOnClick}>
        <planeGeometry args={[700, 700]} />
        <meshStandardMaterial color={palette.ground} roughness={1} />
      </mesh>
      <Grid
        position={[cx, -0.29, -cy]}
        args={[400, 400]}
        cellSize={2}
        sectionSize={10}
        cellThickness={0.6}
        sectionThickness={1}
        cellColor="#d3d9ea"
        sectionColor="#c3cbe0"
        fadeDistance={150}
        fadeStrength={1.6}
        infiniteGrid
      />
      <SoftBox size={[c - a + 0.9, 0.3, d - b + 0.9]} position={[cx, -0.15, -cy]} radius={0.12} color={palette.slab} roughness={0.8} />
      <FloorRect rect={scene.bounds_xy} color={palette.floor} y={0.002} onClick={clearOnClick} />
      <Walls bounds={scene.bounds_xy} />
      {layers.zones && scene.visual_zones.map((z) => (
        <FloorRect
          key={z.id}
          rect={z.rect}
          y={0.006}
          onClick={clearOnClick}
          color={z.id === "MaterialZone" ? "#f5ead6" : z.id === "ProcessZone" ? "#dcefeb" : "#e1e7fb"}
        />
      ))}
      {scene.amr_allowed_regions.map((z) => (
        <FloorRect key={z.id} rect={z.rect} color={palette.aisle} y={0.01} onClick={clearOnClick} />
      ))}
      {scene.amr_allowed_regions.filter((z) => z.id === "MAIN_LOW" || z.id === "MAIN_HIGH").map((z) => (
        <Line key={`guide-${z.id}`} points={[[z.rect[0] + 0.3, 0.028, -(z.rect[1] + z.rect[3]) / 2], [z.rect[2] - 0.3, 0.028, -(z.rect[1] + z.rect[3]) / 2]]} color="#ffffff" lineWidth={1.6} dashed dashSize={0.6} gapSize={0.45} />
      ))}
      {laneArrows(scene.traffic_lanes).map((arrow) => (
        <Line key={`lane-${arrow.id}`} points={arrow.points} color="#ffffff" lineWidth={2.6} />
      ))}
      {scene.reserved_work_areas.map((area) => (
        <WorkAreaMark key={area.id} rect={area.rect} />
      ))}
      {layers.parking && Object.entries(scene.pockets).map(([id, xy]) => {
        const dock = snapshot?.charging?.docks.find((item) => item.anchor === id);
        return (
          <group key={id} position={[xy[0], 0.04, -xy[1]]}>
            <Line points={[[-0.7, 0, -0.75], [-0.7, 0, 0.75], [0.7, 0, 0.75], [0.7, 0, -0.75], [-0.7, 0, -0.75]]} color={dock ? "#17a06b" : "#efb43a"} lineWidth={2.2} />
            {dock && <>
              <CargoPart size={[0.22, 0.72, 0.22]} position={[0.92, 0.36, 0]} color={palette.steel} />
              <CargoPart size={[0.23, 0.06, 0.23]} position={[0.92, 0.76, 0]} color={dock.owner ? "#2fd38f" : "#cfd8ea"} />
            </>}
            <Tag position={[0, 0.25, -1.05]} small title={dock ? dock.id : id} note={dock ? (dock.owner ? "충전 중" : "충전대") : undefined} tone={dock ? (dock.owner ? "green" : "gray") : "gray"} />
          </group>
        );
      })}
      {scene.physical_obstacles.map((item) => (
        <Equipment
          key={item.id}
          item={item}
          selected={selection?.kind === "equipment" && selection.id === item.id}
          hovered={hovered === `equipment:${item.id}`}
          onSelect={select}
          hover={hover}
        />
      ))}
      {scene.stations.map((s) => {
        const active = selection?.kind === "station" && selection.id === s.id;
        // A robot standing on the station already carries a tag; keep one label per spot.
        const occupied = snapshot?.robots.some((r) => r.pose && Math.hypot(r.pose.x_m - s.xy[0], r.pose.y_m - s.xy[1]) < 1.4);
        return (
          <group
            key={s.id}
            position={[s.xy[0], 0.04, -s.xy[1]]}
            onClick={(e) => {
              e.stopPropagation();
              onSelect({ kind: "station", id: s.id });
            }}
            onPointerOver={(e) => {
              e.stopPropagation();
              hover.onOver(`station:${s.id}`);
            }}
            onPointerOut={() => hover.onOut(`station:${s.id}`)}
          >
            <mesh rotation={[-Math.PI / 2, 0, 0]}>
              <circleGeometry args={[0.66, 40]} />
              <meshBasicMaterial color={active ? palette.select : zoneColor(s.id)} transparent opacity={active ? 0.22 : 0.14} depthWrite={false} />
            </mesh>
            <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.002, 0]}>
              <ringGeometry args={[0.56, 0.66, 40]} />
              <meshBasicMaterial color={active ? palette.select : zoneColor(s.id)} />
            </mesh>
            {(active || !occupied) && <Tag position={[0, 0.3, -0.95]} small title={stationNames[s.id] ?? s.id} selected={active} tone="gray" />}
          </group>
        );
      })}
      {destination && <Pin x={destination.xy[0]} y={destination.xy[1]} />}
      {activeDestinations.map((s) => <Pin key={s.id} x={s.xy[0]} y={s.xy[1]} scale={0.85} />)}
      {snapshot?.robots.map((r) =>
        r.pose ? (
          <RobotUnit
            key={r.id}
            robot={r}
            selected={selection?.kind === "robot" && selection.id === r.id}
            showPath={layers.routes}
            onSelect={select}
            hover={hover}
            label={packagingLabel}
          />
        ) : null,
      )}
      {layers.routes && selectedRobot?.pose && selectedRobot.route.length > 0 && (
        <Line
          points={toWorld([selectedRobot.pose, ...selectedRobot.route], 0.09)}
          color="#93a9f4"
          lineWidth={2.6}
          dashed
          dashSize={0.35}
          gapSize={0.22}
        />
      )}
      {layers.inventory && inventory.map((item, index) => (
        <group
          key={item.key}
          position={item.position}
          onClick={(e) => {
            e.stopPropagation();
            onSelect({ kind: item.kind, id: item.id });
          }}
        >
          {item.kind === "tote"
            ? <group scale={0.55}><ToteShape color={selection?.kind === "tote" && selection.id === item.id ? "#7d9cf5" : "#3d68ee"} /></group>
            : <Carton label={packagingLabel} selected={selection?.kind === "stock" && selection.id === item.id} variant={index} />}
        </group>
      ))}
    </Canvas>
  );
});
