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
import { useEffect, useMemo } from "react";
import {
  ACESFilmicToneMapping,
  CanvasTexture,
  PCFSoftShadowMap,
  SRGBColorSpace,
} from "three";

type Props = {
  scene: Scene;
  snapshot: Snapshot | null;
  selection: Selection | null;
  onSelect: (s: Selection) => void;
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
}: {
  text: string;
  position: [number, number, number];
  selected?: boolean;
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
      context.fillStyle = selected ? "#ffffff" : "#254b72";
      context.font = '600 42px Arial, "Malgun Gothic", sans-serif';
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.fillText(text, 256, 66, 470);
    }
    const result = new CanvasTexture(canvas);
    result.colorSpace = SRGBColorSpace;
    return result;
  }, [text, selected]);
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
function Camera({ cx, cy }: { cx: number; cy: number }) {
  const { size } = useThree();
  return (
    <>
      <OrthographicCamera
        makeDefault
        position={[cx + 27, 32, -cy + 32]}
        zoom={Math.min(size.width / 42, (size.height - 105) / 24) * 1.08}
        near={0.1}
        far={200}
      />
      <OrbitControls
        target={[cx, 0, -cy]}
        minZoom={4}
        maxZoom={60}
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
function Equipment({
  item,
  selected,
  onSelect,
}: {
  item: Scene["physical_obstacles"][number];
  selected: boolean;
  onSelect: () => void;
}) {
  const [a, b, c, d] = item.rect,
    w = c - a,
    z = d - b,
    h = item.height_m;
  const rack = item.id.startsWith("RACK"),
    machine = item.id.startsWith("PROCESS"),
    fg = item.id.startsWith("FG"),
    bench = item.id.startsWith("BENCH") || item.id.startsWith("KIT");
  return (
    <group
      position={[(a + c) / 2, 0, -(b + d) / 2]}
      onClick={(e) => {
        e.stopPropagation();
        onSelect();
      }}
    >
      {rack ? (
        <>
          <SoftBox
            size={[w, 0.12, z]}
            position={[0, h - 0.06, 0]}
            color={selected ? "#5a91ec" : "#6b8fb2"}
            radius={0.055}
          />
          {[-w / 2 + 0.06, w / 2 - 0.06].flatMap((x) =>
            [-z / 2 + 0.06, z / 2 - 0.06].map((v) => (
              <SoftBox
                key={`${x}-${v}`}
                size={[0.1, h, 0.1]}
                position={[x, h / 2, v]}
                color={selected ? "#4d82d7" : "#7592ad"}
                radius={0.035}
              />
            )),
          )}
          {[0.2, 0.85, 1.5].map((y) => (
            <SoftBox
              key={y}
              size={[w, 0.11, z]}
              position={[0, y, 0]}
              color="#e6eef5"
              radius={0.045}
            />
          ))}
        </>
      ) : bench ? (
        <>
          <SoftBox
            size={[w, 0.2, z]}
            position={[0, h - 0.1, 0]}
            radius={0.09}
            color={selected ? "#91b7ee" : "#e2ecf4"}
          />
          <SoftBox
            size={[w * 0.93, 0.07, z * 0.91]}
            position={[0, h - 0.22, 0]}
            radius={0.03}
            color="#6e99be"
          />
          {[-w / 2 + 0.16, w / 2 - 0.16].flatMap((x) =>
            [-z / 2 + 0.12, z / 2 - 0.12].map((v) => (
              <SoftBox
                key={`${x}-${v}`}
                size={[0.15, h - 0.2, 0.15]}
                position={[x, (h - 0.2) / 2, v]}
                color="#b9cddd"
                radius={0.045}
              />
            )),
          )}
          {h > 1.3 && (
            <SoftBox
              size={[w * 0.42, h * 0.38, 0.12]}
              position={[w * 0.18, h * 0.65, -z * 0.36]}
              color="#7da2c2"
              radius={0.05}
            />
          )}
        </>
      ) : (
        <>
          <SoftBox
            size={[w, h - 0.2, z]}
            position={[0, h / 2 - 0.02, 0]}
            color={
              selected
                ? "#a4c6f4"
                : fg
                  ? "#d8e7f2"
                  : machine
                    ? "#e5eef7"
                    : "#d7e4ef"
            }
            radius={machine ? 0.24 : 0.16}
            roughness={0.36}
          />
          <SoftBox
            size={[w * 0.91, 0.15, z * 0.89]}
            position={[0, 0.075, 0]}
            color="#92abc0"
            radius={0.065}
            metalness={0.25}
          />
          <SoftBox
            size={[w * 0.98, 0.2, z * 0.98]}
            position={[0, h - 0.1, 0]}
            color={machine ? "#4a85d1" : "#a4c0d7"}
            radius={0.065}
          />
          {fg &&
            [-0.28, 0, 0.28].map((x) => (
              <SoftBox
                key={x}
                size={[w * 0.18, 0.05, z * 0.83]}
                position={[w * x, h - 0.003, 0]}
                color="#edf4fa"
                radius={0.024}
              />
            ))}
          {machine && (
            <>
              <SoftBox
                size={[w * 0.78, h * 0.56, 0.1]}
                position={[0, h * 0.55, z / 2 - 0.025]}
                color="#9bb8cf"
                radius={0.045}
              />
              <SoftBox
                size={[w * 0.7, h * 0.48, 0.105]}
                position={[0, h * 0.55, z / 2 - 0.015]}
                color="#365d80"
                radius={0.046}
                roughness={0.2}
                metalness={0.2}
              />
              <SoftBox
                size={[w * 0.22, 0.055, 0.11]}
                position={[w * 0.13, h * 0.59, z / 2 + 0.045]}
                color="#aacfe9"
                radius={0.024}
              />
              <SoftBox
                size={[w * 0.76, 0.08, 0.06]}
                position={[0, h * 0.2, z / 2 + 0.01]}
                color="#c5d7e6"
                radius={0.025}
              />
              <SoftBox
                size={[0.15, 0.15, 0.075]}
                position={[w * 0.35, h * 0.86, z / 2 + 0.005]}
                color="#6fb3df"
                radius={0.035}
              />
            </>
          )}
        </>
      )}
      {selected && (
        <MapLabel position={[0, h + 0.5, 0]} text={item.id} selected />
      )}
    </group>
  );
}
export default function FactoryScene({
  scene,
  snapshot,
  selection,
  onSelect,
}: Props) {
  const [a, b, c, d] = scene.bounds_xy,
    cx = (a + c) / 2,
    cy = (b + d) / 2;
  return (
    <Canvas
      shadows
      dpr={[1, 1.6]}
      frameloop="demand"
      gl={{ antialias: true, alpha: false, powerPreference: "default" }}
      onCreated={({ gl }) => {
        gl.setClearColor("#edf3fa");
        gl.toneMapping = ACESFilmicToneMapping;
        gl.toneMappingExposure = 1.05;
        gl.shadowMap.type = PCFSoftShadowMap;
      }}
      fallback={
        <div className="scene-fallback">
          3D 표시를 사용할 수 없습니다. 아래 목록에서 상태를 확인하세요.
        </div>
      }
    >
      <Camera cx={cx} cy={cy} />
      <ambientLight intensity={0.35} />
      <hemisphereLight args={["#f6faff", "#a7bfd6", 0.65]} />
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
        intensity={1.8}
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
        color="#d2e1ef"
        roughness={0.65}
      />
      <FloorRect rect={scene.bounds_xy} color="#f5f8fc" y={-0.01} />
      {scene.visual_zones.map((z) => (
        <FloorRect key={z.id} rect={z.rect} color="#e8eef5" />
      ))}
      {scene.amr_allowed_regions.map((z) => (
        <FloorRect key={z.id} rect={z.rect} color="#bcd5ef" y={0.012} />
      ))}
      {Array.from({ length: Math.floor(c - a) + 1 }, (_, i) => (
        <Line
          key={`x${i}`}
          points={[
            [a + i, 0.02, -b],
            [a + i, 0.02, -d],
          ]}
          color="#b7c9dc"
          transparent
          opacity={0.18}
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
          color="#b7c9dc"
          transparent
          opacity={0.18}
          lineWidth={0.5}
        />
      ))}
      {scene.physical_obstacles.map((item) => (
        <Equipment
          key={item.id}
          item={item}
          selected={selection?.kind === "equipment" && selection.id === item.id}
          onSelect={() => onSelect({ kind: "equipment", id: item.id })}
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
              color={selection?.id === s.id ? "#174fe2" : "#7199c1"}
            />
          </mesh>
          <MapLabel
            position={[0, 0.2, -0.8]}
            text={stationNames[s.id] ?? s.id}
            selected={selection?.kind === "station" && selection.id === s.id}
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
                color="#2c4664"
                radius={0.075}
                roughness={0.6}
              />
              <SoftBox
                size={[1.15, 0.42, 0.85]}
                position={[0, 0.33, 0]}
                color={selection?.id === r.id ? "#2c6ce8" : "#4c8cdb"}
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
                color="#a9e5ef"
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
                <SoftBox
                  size={[0.6, 0.5, 0.5]}
                  position={[0.08, 0.9, 0]}
                  color="#86c5d2"
                  radius={0.085}
                  roughness={0.36}
                />
              )}
              <MapLabel
                position={[0, 1.5, 0]}
                text={r.label}
                selected={selection?.kind === "robot" && selection.id === r.id}
              />
            </group>
          ),
      )}
      {snapshot?.robots
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
      {snapshot?.inventory.totes.map((t, i) => {
        const station = scene.stations.find(
          (s) => s.id === (t.station_id ?? t.location),
        );
        if (!station || t.robot_id) return null;
        const id = String(t.id ?? t.tote_id ?? i);
        return (
          <group
            key={id}
            position={[station.xy[0], 0.45, -station.xy[1]]}
            onClick={(e) => {
              e.stopPropagation();
              onSelect({ kind: "tote", id });
            }}
          >
            <SoftBox
              size={[0.65, 0.6, 0.5]}
              position={[0, 0, 0]}
              radius={0.085}
              color={
                selection?.kind === "tote" && selection.id === id
                  ? "#5e9ee9"
                  : "#8bc3d0"
              }
              roughness={0.4}
            />
          </group>
        );
      })}
    </Canvas>
  );
}
