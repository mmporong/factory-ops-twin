"use client";
import { Canvas, useThree } from "@react-three/fiber";
import {
  Line,
  OrbitControls,
  OrthographicCamera,
  RoundedBox,
  Environment,
  Lightformer,
} from "@react-three/drei";
import type { Scene, Snapshot, Selection, Rect } from "../lib/types";
import { stationNames } from "../lib/types";
import { inventoryDisplay } from "../lib/inventory-display";
import { RoundedBoxGeometry } from "three/examples/jsm/geometries/RoundedBoxGeometry.js";
import { memo, useEffect, useMemo } from "react";
import {
  ACESFilmicToneMapping,
  BoxGeometry,
  CanvasTexture,
  PCFSoftShadowMap,
  SRGBColorSpace,
  type Texture,
} from "three";
const cargoPartGeometry = new BoxGeometry(1, 1, 1);
const cartonGeometry = new RoundedBoxGeometry(0.41, 0.3, 0.31, 2, 0.012);

function CargoPart({ size, position, color }: { size: [number, number, number]; position: [number, number, number]; color: string }) {
  return <mesh geometry={cargoPartGeometry} scale={size} position={position} castShadow receiveShadow><meshStandardMaterial color={color} roughness={0.65} /></mesh>;
}

type Props = {
  scene: Scene;
  snapshot: Snapshot | null;
  selection: Selection | null;
  onSelect: (s: Selection) => void;
  showZones: boolean;
  showParking: boolean;
  showRoutes: boolean;
  showInventory: boolean;
  viewRequest: { version: number; focus: boolean };
};
function SoftBox({
  size,
  position,
  color,
  radius = 0.12,
  roughness = 0.4,
  metalness = 0.08,
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
      <meshPhysicalMaterial
        color={color}
        roughness={roughness}
        metalness={metalness}
        clearcoat={0.24}
        clearcoatRoughness={0.4}
      />
    </RoundedBox>
  );
}
function MapLabel({
  text,
  position,
  selected = false,
  accent = "#43799c",
}: {
  text: string;
  position: [number, number, number];
  selected?: boolean;
  accent?: string;
}) {
  const texture = useMemo(() => {
    const canvas = document.createElement("canvas");
    canvas.width = 512;
    canvas.height = 128;
    const context = canvas.getContext("2d");
    if (context) {
      context.shadowColor = "#203e6426";
      context.shadowBlur = 9;
      context.shadowOffsetY = 4;
      context.fillStyle = selected ? "#2866e5" : "#ffffff";
      context.beginPath();
      context.roundRect(8, 12, 496, 104, 32);
      context.fill();
      context.shadowBlur = 0;
      context.shadowOffsetY = 0;
      context.strokeStyle = selected ? "#2866e5" : "#d3dfec";
      context.lineWidth = 2;
      context.stroke();
      context.fillStyle = selected ? "#ffffff" : accent;
      context.beginPath();
      context.roundRect(24, 32, 9, 64, 4);
      context.fill();
      context.fillStyle = selected ? "#ffffff" : "#254b72";
      context.font = '600 42px Arial, "Malgun Gothic", sans-serif';
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.fillText(text, 256, 66, 470);
    }
    const result = new CanvasTexture(canvas);
    result.colorSpace = SRGBColorSpace;
    return result;
  }, [text, selected, accent]);
  useEffect(() => () => texture.dispose(), [texture]);
  return (
    <sprite position={position} scale={[2.7, 0.675, 1]} renderOrder={10}>
      <spriteMaterial
        map={texture}
        transparent
        depthTest={false}
        depthWrite={false}
        toneMapped={false}
      />
    </sprite>
  );
}
function Camera({ cx, cy, width, depth, focused }: { cx: number; cy: number; width: number; depth: number; focused: boolean }) {
  const { size } = useThree();
  return (
    <>
      <OrthographicCamera
        makeDefault
        position={[cx + 27, 32, -cy + 32]}
        zoom={Math.min(size.width / ((width + depth) * 0.81), (size.height - 105) / ((width + depth) * 0.46)) * 1.08 * (focused ? 3.2 : 1)}
        near={0.1}
        far={200}
      />
      <OrbitControls
        target={[cx, 0, -cy]}
        minZoom={4}
        maxZoom={90}
        maxPolarAngle={Math.PI / 2.3}
        minPolarAngle={0.2}
        enableDamping={false}
      />
    </>
  );
}
function FloorRect({
  rect,
  color,
  y = 0,
}: {
  rect: Rect;
  color: string;
  y?: number;
}) {
  const [a, b, c, d] = rect;
  return (
    <mesh
      rotation={[-Math.PI / 2, 0, 0]}
      position={[(a + c) / 2, y, -(b + d) / 2]}
      receiveShadow
    >
      <planeGeometry args={[c - a, d - b]} />
      <meshStandardMaterial color={color} roughness={0.82} />
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
      ].map(([x, y, dx, dy], i) => {
        return (
          <Line
            key={i}
            points={[
              [x + dx * length, 0.035, -y],
              [x, 0.035, -y],
              [x, 0.035, -(y + dy * length)],
            ]}
            color="#daa32a"
            lineWidth={2.2}
          />
        );
      })}
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
        <CargoPart size={[0.21, 0.05, 0.01]} position={[0, 0.14, side * 0.252]} color="#203d49" />
        {[-0.23, -0.1, 0.1, 0.23].map((x) => <CargoPart key={x} size={[0.02, 0.26, 0.018]} position={[x, -0.03, side * 0.252]} color={color} />)}
      </group>)}
      <CargoPart size={[0.54, 0.015, 0.39]} position={[0, -0.16, 0]} color="#34515a" />
    </>
  );
});
const Carton = memo(function Carton({ label, selected, variant = 0 }: { label: Texture; selected: boolean; variant?: number }) {
  const color = selected ? "#71a1d9" : ["#bd955f", "#ccaa76", "#ad8557", "#d5b789"][variant % 4];
  return <>
    <mesh geometry={cartonGeometry} castShadow receiveShadow><meshStandardMaterial color={color} roughness={0.9} /></mesh>
    <mesh position={[0, 0.152, 0]} rotation={[-Math.PI / 2, 0, 0]}>
      <planeGeometry args={[0.045, 0.3]} /><meshStandardMaterial color="#e3c89c" roughness={0.75} />
    </mesh>
    <mesh position={[0.08, -0.005, 0.157]}>
      <planeGeometry args={[0.18, 0.105]} /><meshBasicMaterial map={label} toneMapped={false} />
    </mesh>
    <mesh position={[-0.1, 0.06, 0.157]}>
      <planeGeometry args={[0.04, 0.05]} /><meshStandardMaterial color="#526d7b" roughness={0.85} />
    </mesh>
  </>;
});
const Equipment = memo(function Equipment({
  item,
  selected,
  onSelect,
}: {
  item: Scene["physical_obstacles"][number];
  selected: boolean;
  onSelect: (s: Selection) => void;
}) {
  const [a, b, c, d] = item.rect,
    w = c - a,
    z = d - b,
    h = item.height_m;
  const rack = item.id.startsWith("RACK"),
    machine = item.id.startsWith("PROCESS"),
    fg = item.id.startsWith("FG"),
    bench = item.id.startsWith("BENCH") || item.id.startsWith("KIT");
  const trim = item.id === "PROCESS_2" ? "#38749b" : item.id === "PROCESS_3" ? "#cc9137" : "#157e7d";
  const frame = selected ? "#2563eb" : "#354f63";
  const surface = h > 1.3 ? h * 0.56 : h - 0.12;
  return (
    <group
      position={[(a + c) / 2, 0, -(b + d) / 2]}
      onClick={(e) => {
        e.stopPropagation();
        onSelect({ kind: "equipment", id: item.id });
      }}
    >
      {rack ? (
        <>
          {[-w / 2 + 0.06, w / 2 - 0.06].flatMap((x) =>
            [-z / 2 + 0.06, z / 2 - 0.06].map((v) => (
              <SoftBox
                key={`${x}-${v}`}
                size={[0.1, h, 0.1]}
                position={[x, h / 2, v]}
                color={frame}
                radius={0.035}
                metalness={0.45}
              />
            )),
          )}
          {[0.18, h * 0.41, h * 0.73, h - 0.06].map((y) => (
            <group key={y}>
              <SoftBox size={[w - 0.12, 0.055, z - 0.08]} position={[0, y, 0]} color="#b6c4c9" radius={0.02} metalness={0.5} />
              {[-1, 1].map((side) => (
                <SoftBox key={side} size={[w, 0.1, 0.075]} position={[0, y - 0.03, side * (z / 2 - 0.04)]} color="#e4a135" radius={0.022} roughness={0.5} />
              ))}
            </group>
          ))}
          {[-1, 1].map((side) => (
            <Line key={side} points={[[side * (w / 2 - 0.04), 0.25, -z / 2 + 0.08], [side * (w / 2 - 0.04), h - 0.15, z / 2 - 0.08]]} color="#78909e" lineWidth={1.6} />
          ))}
        </>
      ) : bench ? (
        <>
          <SoftBox
            size={[w, 0.2, z]}
            position={[0, surface - 0.1, 0]}
            radius={0.09}
            color={selected ? "#547fc2" : "#376c69"}
          />
          <SoftBox
            size={[w * 0.93, 0.07, z * 0.91]}
            position={[0, surface - 0.22, 0]}
            radius={0.03}
            color="#e4e9e4"
          />
          {[-w / 2 + 0.16, w / 2 - 0.16].flatMap((x) =>
            [-z / 2 + 0.12, z / 2 - 0.12].map((v) => (
              <SoftBox
                key={`${x}-${v}`}
                size={[0.15, surface - 0.2, 0.15]}
                position={[x, (surface - 0.2) / 2, v]}
                color={frame}
                radius={0.045}
                metalness={0.4}
              />
            )),
          )}
          {h > 1.3 && (
            <SoftBox
              size={[w * 0.75, h * 0.36, 0.12]}
              position={[0, h * 0.79, -z * 0.36]}
              color="#b2c5c5"
              radius={0.05}
            />
          )}
          <SoftBox size={[w * 0.28, 0.07, z * 0.34]} position={[-w * 0.24, surface + 0.025, 0]} color="#cad3d2" radius={0.025} metalness={0.6} />
          <SoftBox size={[w * 0.14, 0.09, z * 0.2]} position={[w * 0.27, surface + 0.035, 0]} color="#344e5b" radius={0.025} />
        </>
      ) : fg ? (
        <>
          <SoftBox size={[w, 0.18, z]} position={[0, h * 0.55, 0]} color={frame} radius={0.06} metalness={0.4} />
          {[-1, 1].map((side) => (
            <group key={side}>
              <SoftBox size={[w, 0.13, 0.1]} position={[0, h * 0.76, side * (z / 2 - 0.07)]} color="#b0c4c9" radius={0.035} metalness={0.6} />
              {[-w / 2 + 0.18, w / 2 - 0.18].map((x) => (
                <SoftBox key={x} size={[0.18, h * 0.55, 0.18]} position={[x, h * 0.275, side * (z / 2 - 0.18)]} color="#4c6574" radius={0.04} />
              ))}
            </group>
          ))}
          {Array.from({ length: Math.max(3, Math.floor(w / 0.3)) }, (_, i) => (
            <mesh key={i} position={[-w / 2 + 0.2 + i * (w - 0.4) / Math.max(2, Math.floor(w / 0.3) - 1), h * 0.65, 0]} rotation={[Math.PI / 2, 0, 0]} castShadow>
              <cylinderGeometry args={[0.075, 0.075, z - 0.3, 12]} />
              <meshStandardMaterial color="#b8c9ce" metalness={0.65} roughness={0.32} />
            </mesh>
          ))}
          <SoftBox size={[w * 0.22, 0.13, 0.08]} position={[w * 0.27, h * 0.54, z / 2 - 0.025]} color="#c89439" radius={0.025} />
        </>
      ) : (
        <>
          <SoftBox
            size={[w, h - 0.2, z]}
            position={[0, h / 2 - 0.02, 0]}
            color={
              selected
                ? "#b7cce9"
                : "#e0e6e2"
            }
            radius={machine ? 0.14 : 0.09}
            roughness={0.44}
          />
          <SoftBox
            size={[w * 0.91, 0.15, z * 0.89]}
            position={[0, 0.075, 0]}
            color="#3d5362"
            radius={0.065}
            metalness={0.25}
          />
          <SoftBox
            size={[w * 0.98, 0.2, z * 0.98]}
            position={[0, h - 0.1, 0]}
            color={machine ? trim : "#496b7b"}
            radius={0.065}
          />
          {machine && (
            <>
              <SoftBox
                size={[w * 0.69, h * 0.56, 0.1]}
                position={[-w * 0.09, h * 0.55, z / 2 - 0.055]}
                color="#637d85"
                radius={0.045}
              />
              <SoftBox
                size={[w * 0.61, h * 0.48, 0.055]}
                position={[-w * 0.09, h * 0.55, z / 2 - 0.025]}
                color="#233e49"
                radius={0.046}
                roughness={0.16}
                metalness={0.42}
              />
              <SoftBox
                size={[w * 0.5, 0.055, 0.045]}
                position={[-w * 0.09, h * 0.46, z / 2 - 0.013]}
                color="#8babb3"
                radius={0.024}
              />
              <SoftBox
                size={[w * 0.76, 0.08, 0.06]}
                position={[0, h * 0.2, z / 2 - 0.025]}
                color={trim}
                radius={0.025}
              />
              <SoftBox
                size={[w * 0.13, h * 0.2, 0.055]}
                position={[w * 0.35, h * 0.66, z / 2 - 0.025]}
                color="#243f50"
                radius={0.035}
              />
              <SoftBox size={[w * 0.09, h * 0.11, 0.018]} position={[w * 0.35, h * 0.68, z / 2 - 0.015]} color="#7aa7b6" radius={0.015} />
              <SoftBox size={[0.08, h * 0.25, 0.045]} position={[w * 0.23, h * 0.52, z / 2 - 0.025]} color="#b0c0c2" radius={0.025} metalness={0.7} />
              {[-0.42, 0.42].map((x) => (
                <SoftBox key={x} size={[0.14, h * 0.18, 0.1]} position={[w * x, h * 0.15, z / 2 - 0.055]} color="#dca632" radius={0.035} />
              ))}
            </>
          )}
          {!machine && (
            <>
              {[-0.25, 0.25].map((x) => (
                <group key={x}>
                  <SoftBox size={[w * 0.46, h * 0.72, 0.04]} position={[w * x, h * 0.5, z / 2 - 0.03]} color="#c3d0cd" radius={0.04} metalness={0.22} />
                  <SoftBox size={[0.045, h * 0.18, 0.055]} position={[w * (x + 0.14), h * 0.56, z / 2 - 0.03]} color="#415c65" radius={0.015} />
                  {[0.23, 0.28, 0.33].map((y) => (
                    <SoftBox key={y} size={[w * 0.25, 0.028, 0.025]} position={[w * x, h * y, z / 2 - 0.02]} color="#6f8b94" radius={0.008} />
                  ))}
                </group>
              ))}
            </>
          )}
        </>
      )}
      {selected && (
        <>
          <Line points={[[-w / 2, 0.055, -z / 2], [w / 2, 0.055, -z / 2], [w / 2, 0.055, z / 2], [-w / 2, 0.055, z / 2], [-w / 2, 0.055, -z / 2]]} color="#2563eb" lineWidth={2.5} />
          <MapLabel position={[0, h + 0.5, 0]} text={item.id} selected />
        </>
      )}
    </group>
  );
});
export default memo(function FactoryScene({
  scene,
  snapshot,
  selection,
  onSelect,
  showZones,
  showParking,
  showRoutes,
  showInventory,
  viewRequest,
}: Props) {
  const [a, b, c, d] = scene.bounds_xy,
    cx = (a + c) / 2,
    cy = (b + d) / 2;
  const inventory = useMemo(() => inventoryDisplay(scene, snapshot), [scene, snapshot]);
  let focusX = cx, focusY = cy;
  if (viewRequest.focus && selection) {
    if (selection.kind === "robot") {
      const pose = snapshot?.robots.find((r) => r.id === selection.id)?.pose;
      if (pose) { focusX = pose.x_m; focusY = pose.y_m; }
    } else if (selection.kind === "station") {
      const xy = scene.stations.find((s) => s.id === selection.id)?.xy;
      if (xy) [focusX, focusY] = xy;
    } else if (selection.kind === "equipment") {
      const rect = scene.physical_obstacles.find((e) => e.id === selection.id)?.rect;
      if (rect) { focusX = (rect[0] + rect[2]) / 2; focusY = (rect[1] + rect[3]) / 2; }
    } else {
      const symbol = inventory.find((item) => item.kind === selection.kind && item.id === selection.id);
      if (symbol) { focusX = symbol.position[0]; focusY = -symbol.position[2]; }
    }
  }
  const packagingLabel = useMemo(() => {
    const canvas = document.createElement("canvas");
    canvas.width = 128; canvas.height = 64;
    const ctx = canvas.getContext("2d");
    if (ctx) {
      ctx.fillStyle = "#f6f4e9"; ctx.fillRect(0, 0, 128, 64);
      ctx.fillStyle = "#40525a"; ctx.font = "bold 14px Arial"; ctx.fillText("PARTS", 8, 19);
      for (let i = 0; i < 30; i++) ctx.fillRect(8 + i * 3, 28, i % 3 === 0 ? 2 : 1, 23);
      ctx.fillStyle = "#c2c5be"; ctx.fillRect(8, 57, 90, 2);
    }
    const texture = new CanvasTexture(canvas); texture.colorSpace = SRGBColorSpace; return texture;
  }, []);
  useEffect(() => () => packagingLabel.dispose(), [packagingLabel]);
  return (
    <Canvas
      shadows
      dpr={[1, 1.6]}
      frameloop="demand"
      gl={{ antialias: true, alpha: false, powerPreference: "default" }}
      onCreated={({ gl }) => {
        gl.setClearColor("#eaf0f4");
        gl.toneMapping = ACESFilmicToneMapping;
        gl.toneMappingExposure = 1;
        gl.shadowMap.type = PCFSoftShadowMap;
      }}
      fallback={
        <div className="scene-fallback">
          3D 표시를 사용할 수 없습니다. 아래 목록에서 상태를 확인하세요.
        </div>
      }
    >
      <Camera key={viewRequest.version} cx={focusX} cy={focusY} width={c-a} depth={d-b} focused={viewRequest.focus} />
      <ambientLight intensity={0.35} />
      <hemisphereLight args={["#f4f8fa", "#889b9c", 0.55]} />
      <Environment resolution={64} frames={1} environmentIntensity={0.25}>
        <Lightformer
          form="rect"
          intensity={3}
          color="#ffffff"
          position={[0, 14, 0]}
          rotation={[Math.PI / 2, 0, 0]}
          scale={[24, 16, 1]}
        />
        <Lightformer
          form="rect"
          intensity={2}
          color="#d9eaff"
          position={[-16, 8, 5]}
          rotation={[0, Math.PI / 2, 0]}
          scale={[12, 14, 1]}
        />
        <Lightformer
          form="rect"
          intensity={1.5}
          color="#ffffff"
          position={[12, 6, -18]}
          scale={[16, 10, 1]}
        />
      </Environment>
      <directionalLight
        position={[cx - 10, 28, -cy + 12]}
        target-position={[cx, 0, -cy]}
        intensity={1.5}
        color="#fffaf4"
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-left={-40}
        shadow-camera-right={40}
        shadow-camera-top={40}
        shadow-camera-bottom={-40}
        shadow-bias={-0.0005}
        shadow-normalBias={0.035}
        shadow-radius={4}
        onUpdate={(light) => light.target.updateMatrixWorld()}
      />
      <directionalLight
        position={[cx + 18, 16, -cy - 18]}
        intensity={0.65}
        color="#d8eaff"
      />
      <mesh
        rotation={[-Math.PI / 2, 0, 0]}
        position={[cx, -0.56, -cy]}
        receiveShadow
      >
        <planeGeometry args={[200, 200]} />
        <shadowMaterial color="#7195bb" opacity={0.16} transparent />
      </mesh>
      <SoftBox
        size={[c - a + 0.5, 0.5, d - b + 0.5]}
        position={[cx, -0.28, -cy]}
        radius={0.22}
        color="#879da9"
        roughness={0.65}
      />
      <FloorRect rect={scene.bounds_xy} color="#e1e6e3" y={-0.01} />
      {showZones && scene.visual_zones.map((z) => (
        <FloorRect
          key={z.id}
          rect={z.rect}
          color={z.id === "MaterialZone" ? "#c2baa0" : z.id === "ProcessZone" ? "#8eb9aa" : "#a0b6c9"}
        />
      ))}
      {scene.amr_allowed_regions.map((z) => (
        <FloorRect key={z.id} rect={z.rect} color="#91b2c8" y={0.012} />
      ))}
      {scene.amr_allowed_regions.filter((z) => z.id === "MAIN_LOW" || z.id === "MAIN_HIGH").map((z) => (
        <Line key={`guide-${z.id}`} points={[[z.rect[0] + 0.3, 0.028, -(z.rect[1] + z.rect[3]) / 2], [z.rect[2] - 0.3, 0.028, -(z.rect[1] + z.rect[3]) / 2]]} color="#edf4f3" lineWidth={1.5} dashed dashSize={0.55} gapSize={0.45} />
      ))}
      {scene.reserved_work_areas.map((area) => (
        <WorkAreaMark key={area.id} rect={area.rect} />
      ))}
      <Line points={[[a + 0.15, 0.025, -b - 0.15], [c - 0.15, 0.025, -b - 0.15], [c - 0.15, 0.025, -d + 0.15], [a + 0.15, 0.025, -d + 0.15], [a + 0.15, 0.025, -b - 0.15]]} color="#8d9d9a" lineWidth={1} />
      {showParking && Object.entries(scene.pockets).map(([id, xy]) => (
        <group key={id} position={[xy[0], 0.04, -xy[1]]}>
          <Line points={[[-0.65, 0, -0.7], [-0.65, 0, 0.7], [0.65, 0, 0.7], [0.65, 0, -0.7]]} color="#edf4f3" lineWidth={2} />
          <MapLabel position={[0, 0.15, -0.9]} text={`${id} 대기`} accent="#657d8b" />
        </group>
      ))}
      {Array.from({ length: Math.floor(c - a) + 1 }, (_, i) => (
        <Line
          key={`x${i}`}
          points={[
            [a + i, 0.02, -b],
            [a + i, 0.02, -d],
          ]}
          color="#7c989f"
          transparent
          opacity={0.11}
          lineWidth={0.5}
        />
      ))}
      {Array.from({ length: Math.floor(d - b) + 1 }, (_, i) => (
        <Line
          key={`y${i}`}
          points={[
            [a, 0.02, -b - i],
            [c, 0.02, -b - i],
          ]}
          color="#7c989f"
          transparent
          opacity={0.11}
          lineWidth={0.5}
        />
      ))}
      {scene.physical_obstacles.map((item) => (
        <Equipment
          key={item.id}
          item={item}
          selected={selection?.kind === "equipment" && selection.id === item.id}
          onSelect={onSelect}
        />
      ))}
      {scene.stations.map((s) => (
        <group
          key={s.id}
          position={[s.xy[0], 0.04, -s.xy[1]]}
          onClick={(e) => {
            e.stopPropagation();
            onSelect({ kind: "station", id: s.id });
          }}
        >
          <mesh rotation={[-Math.PI / 2, 0, 0]}>
            <ringGeometry args={[0.56, 0.64, 32]} />
            <meshBasicMaterial
              color={selection?.id === s.id ? "#174fe2" : s.id.startsWith("S") ? "#a77429" : s.id === "OUT" ? "#426e9c" : "#177e7c"}
            />
          </mesh>
          <MapLabel
            position={[0, 0.2, -0.8]}
            text={stationNames[s.id] ?? s.id}
            selected={selection?.kind === "station" && selection.id === s.id}
            accent={s.id.startsWith("S") ? "#a77429" : s.id === "OUT" ? "#426e9c" : "#177e7c"}
          />
        </group>
      ))}
      {snapshot?.robots.map(
        (r) =>
          r.pose && (
            <group
              key={r.id}
              position={[r.pose.x_m, 0.05, -r.pose.y_m]}
              rotation={[0, r.pose.yaw_rad, 0]}
              onClick={(e) => {
                e.stopPropagation();
                onSelect({ kind: "robot", id: r.id });
              }}
            >
              <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
                <ringGeometry
                  args={[0.88, selection?.id === r.id ? 1.02 : 0.92, 64]}
                />
                <meshBasicMaterial
                  color={selection?.id === r.id ? "#2166f3" : "#8babca"}
                />
              </mesh>
              <SoftBox
                size={[1.08, 0.17, 0.78]}
                position={[0, 0.12, 0]}
                color="#263d4d"
                radius={0.075}
                roughness={0.6}
              />
              <SoftBox
                size={[1.15, 0.42, 0.85]}
                position={[0, 0.33, 0]}
                color={selection?.id === r.id ? "#245fda" : "#3579bb"}
                radius={0.18}
                roughness={0.28}
                metalness={0.12}
              />
              <SoftBox
                size={[0.92, 0.14, 0.71]}
                position={[0, 0.57, 0]}
                color="#f1f7fc"
                radius={0.06}
                roughness={0.3}
              />
              <SoftBox
                size={[0.06, 0.085, 0.48]}
                position={[0.535, 0.33, 0]}
                color="#e7b94b"
                radius={0.028}
                roughness={0.24}
              />
              {[-0.44, 0.44].flatMap((x) =>
                [-0.36, 0.36].map((z) => (
                  <mesh
                    key={`${x}-${z}`}
                    position={[x, 0.15, z]}
                    rotation={[Math.PI / 2, 0, 0]}
                    castShadow
                  >
                    <cylinderGeometry args={[0.13, 0.13, 0.1, 24]} />
                    <meshStandardMaterial color="#354b64" roughness={0.75} />
                  </mesh>
                )),
              )}
              <mesh position={[-0.35, 0.75, 0]}>
                <cylinderGeometry args={[0.12, 0.14, 0.18, 32]} />
                <meshPhysicalMaterial
                  color="#3b5878"
                  roughness={0.25}
                  metalness={0.2}
                  clearcoat={0.3}
                />
              </mesh>
              {r.payload_id && (
                <group position={[0.08, 0.9, 0]}>
                  {r.payload_id.startsWith("LOT") ? <group scale={2}><Carton label={packagingLabel} selected={selection?.kind === "robot" && selection.id === r.id} /></group> : <ToteShape color="#4f9095" />}
                </group>
              )}
              <MapLabel
                position={[0, 1.5, 0]}
                text={r.label}
                selected={selection?.kind === "robot" && selection.id === r.id}
              />
            </group>
          ),
      )}
      {showRoutes && snapshot?.robots
        .filter(
          (r) =>
            selection?.kind === "robot" &&
            selection.id === r.id &&
            r.pose &&
            r.route.length > 0,
        )
        .map((r) => (
          <Line
            key={r.id}
            points={[r.pose!, ...r.route].map((p) => [p.x_m, 0.09, -p.y_m])}
            color="#2267ef"
            lineWidth={3}
            dashed
            dashSize={0.3}
            gapSize={0.2}
          />
        ))}
      {showInventory && inventory.map((item, index) => <group key={item.key} position={item.position} onClick={(e) => {
        e.stopPropagation(); onSelect({ kind: item.kind, id: item.id });
      }}>
        {item.kind === "tote" ? <group scale={0.55}><ToteShape color={selection?.kind === "tote" && selection.id === item.id ? "#397bdd" : "#4f9095"} /></group> : <Carton label={packagingLabel} selected={selection?.kind === "stock" && selection.id === item.id} variant={index} />}
      </group>)}
    </Canvas>
  );
});
