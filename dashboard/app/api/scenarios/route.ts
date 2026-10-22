import { NextResponse } from "next/server";

import { proxy } from "../../../lib/proxy";

export const dynamic = "force-dynamic";

const SCENARIO_ID = /^[a-z0-9][a-z0-9-]{0,63}-[0-9a-f]{12}$/;

export function GET(request: Request) {
  const search = new URL(request.url).searchParams;
  const scenarioId = search.get("scenario_id");
  const frameValue = search.get("frame_index");

  if (scenarioId === null && frameValue === null) return proxy("scenarios");
  if (scenarioId === null || frameValue === null || !SCENARIO_ID.test(scenarioId)) {
    return NextResponse.json({ error: "기록 또는 프레임 선택이 올바르지 않습니다." }, { status: 400 });
  }

  const frameIndex = Number(frameValue);
  if (!/^\d+$/.test(frameValue) || !Number.isSafeInteger(frameIndex) || frameIndex < 0 || frameIndex >= 120) {
    return NextResponse.json({ error: "프레임 번호가 기록 범위를 벗어났습니다." }, { status: 400 });
  }
  return proxy(`scenarios/${scenarioId}/frames/${frameIndex}`);
}

export function POST(request: Request) {
  return proxy("scenarios", request);
}
