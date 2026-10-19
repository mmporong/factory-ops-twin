"use client";
import { Canvas, useThree } from "@react-three/fiber";
import {
  Line,
  OrbitControls,
  OrthographicCamera,
  RoundedBox,
} from "@react-three/drei";
import type { Scene, Snapshot, Selection, Rect } from "../lib/types";
import { stationNames } from "../lib/types";
import { useEffect, useMemo } from "react";
import { CanvasTexture, SRGBColorSpace } from "three";

type Props = {
  scene: Scene;
  snapshot: Snapshot | null;
  selection: Selection | null;
  onSelect: (s: Selection) => void;
};
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
      context.fillStyle = selected ? "#245fe5" : "#ffffff";
      context.beginPath();
      context.roundRect(4, 12, 504, 104, 18);
      context.fill();
      context.strokeStyle = selected ? "#245fe5" : "#9cb9d6";
      context.lineWidth = 4;
      context.stroke();
      context.fillStyle = selected ? "#ffffff" : "#254b72";
      context.font = '600 45px Arial, "Malgun Gothic", sans-serif';
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
        zoom={Math.min(size.width / 40, size.height / 32) * 0.9}
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
  y = 0.015,
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
      <meshStandardMaterial color={color} />
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
    fg = item.id.startsWith("FG");
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
          <mesh position={[0, h, 0]} castShadow>
            <boxGeometry args={[w, 0.09, z]} />
            <meshStandardMaterial color="#527799" />
          </mesh>
          {[-w / 2 + 0.06, w / 2 - 0.06].flatMap((x) =>
            [-z / 2 + 0.06, z / 2 - 0.06].map((v) => (
              <mesh key={`${x}-${v}`} position={[x, h / 2, v]} castShadow>
                <boxGeometry args={[0.08, h, 0.08]} />
                <meshStandardMaterial
                  color={selected ? "#1764f5" : "#5a7794"}
                />
              </mesh>
            )),
          )}
          {[0.2, 0.85, 1.5].map((y) => (
            <mesh key={y} position={[0, y, 0]} castShadow>
              <boxGeometry args={[w, 0.09, z]} />
              <meshStandardMaterial color="#b7c8d8" />
            </mesh>
          ))}
        </>
      ) : (
        <>
          <RoundedBox
            args={[w, h, z]}
            radius={Math.min(0.09, w / 6, z / 6)}
            position={[0, h / 2, 0]}
            castShadow
            receiveShadow
          >
            <meshStandardMaterial
              color={
                selected
                  ? "#91b8fb"
                  : fg
                    ? "#c7d9e7"
                    : machine
                      ? "#e0e8ee"
                      : "#b9cad8"
              }
              roughness={0.65}
            />
          </RoundedBox>
          <mesh position={[0, h + 0.025, 0]}>
            <boxGeometry args={[w * 0.98, 0.05, z * 0.98]} />
            <meshStandardMaterial color={machine ? "#2e6bc1" : "#8ca5bb"} />
          </mesh>
          {machine && (
            <>
              <mesh position={[0, h * 0.58, z / 2 + 0.015]}>
                <boxGeometry args={[w * 0.74, h * 0.52, 0.04]} />
                <meshStandardMaterial color="#1d3c57" roughness={0.25} />
              </mesh>
              <mesh position={[w * 0.35, h * 0.87, z / 2 + 0.05]}>
                <boxGeometry args={[0.13, 0.13, 0.04]} />
                <meshStandardMaterial color="#39a88c" emissive="#24674e" />
              </mesh>
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
      onCreated={({ gl }) => {
        gl.setClearColor("#eaf2fb");
      }}
      fallback={
        <div className="scene-fallback">
          3D 표시를 사용할 수 없습니다. 아래 목록에서 상태를 확인하세요.
        </div>
      }
    >
      <Camera cx={cx} cy={cy} />
      <ambientLight intensity={1.6} />
      <directionalLight
        position={[5, 25, 5]}
        intensity={2.3}
        castShadow
        shadow-mapSize={[2048, 2048]}
        shadow-camera-left={-40}
        shadow-camera-right={40}
        shadow-camera-top={40}
        shadow-camera-bottom={-40}
        shadow-bias={-0.0005}
      />
      <mesh position={[cx, -0.22, -cy]} receiveShadow>
        <boxGeometry args={[c - a + 0.5, 0.4, d - b + 0.5]} />
        <meshStandardMaterial color="#c1d2e4" />
      </mesh>
      <FloorRect rect={scene.bounds_xy} color="#f6f8fb" />
      {scene.visual_zones.map((z, i) => (
        <FloorRect
          key={z.id}
          rect={z.rect}
          color={["#e6edf4", "#e2eaf2", "#e7edf3"][i % 3]}
        />
      ))}
      {scene.amr_allowed_regions.map((z) => (
        <FloorRect key={z.id} rect={z.rect} color="#d3e4f6" y={0.025} />
      ))}
      {Array.from({ length: Math.floor(c - a) + 1 }, (_, i) => (
        <Line
          key={`x${i}`}
          points={[
            [a + i, 0.035, -b],
            [a + i, 0.035, -d],
          ]}
          color="#bccddd"
          transparent
          opacity={0.3}
          lineWidth={0.5}
        />
      ))}
      {Array.from({ length: Math.floor(d - b) + 1 }, (_, i) => (
        <Line
          key={`y${i}`}
          points={[
            [a, 0.035, -b - i],
            [c, 0.035, -b - i],
          ]}
          color="#bccddd"
          transparent
          opacity={0.3}
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
                  args={[0.86, selection?.id === r.id ? 1.01 : 0.9, 40]}
                />
                <meshBasicMaterial
                  color={selection?.id === r.id ? "#2166f3" : "#8babca"}
                />
              </mesh>
              <RoundedBox
                args={[1.15, 0.42, 0.85]}
                radius={0.12}
                position={[0, 0.3, 0]}
                castShadow
              >
                <meshStandardMaterial
                  color={selection?.id === r.id ? "#1a5ee8" : "#397ec5"}
                  metalness={0.12}
                  roughness={0.4}
                />
              </RoundedBox>
              <mesh position={[0, 0.58, 0]} castShadow>
                <boxGeometry args={[0.8, 0.12, 0.65]} />
                <meshStandardMaterial color="#e9f1f7" />
              </mesh>
              <mesh position={[0.58, 0.32, 0]}>
                <boxGeometry args={[0.03, 0.09, 0.5]} />
                <meshStandardMaterial color="#78e5e0" emissive="#287f91" />
              </mesh>
              <mesh position={[-0.35, 0.75, 0]}>
                <cylinderGeometry args={[0.12, 0.12, 0.2, 16]} />
                <meshStandardMaterial color="#23425a" />
              </mesh>
              {r.payload_id && (
                <mesh position={[0.08, 0.9, 0]} castShadow>
                  <boxGeometry args={[0.6, 0.5, 0.5]} />
                  <meshStandardMaterial color="#76c5ce" />
                </mesh>
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
            <RoundedBox args={[0.65, 0.6, 0.5]} radius={0.04} castShadow>
              <meshStandardMaterial
                color={selection?.id === id ? "#2477f5" : "#6cb6c3"}
              />
            </RoundedBox>
          </group>
        );
      })}
    </Canvas>
  );
}
