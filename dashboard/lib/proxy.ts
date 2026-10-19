import { NextResponse } from "next/server";
export async function proxy(path: string, request?: Request) {
  try {
    const upstream = await fetch(
      `${process.env.OPS_API_URL ?? "http://127.0.0.1:8765"}/api/${path}`,
      {
        method: request ? "POST" : "GET",
        headers: { "Content-Type": "application/json" },
        body: request ? await request.text() : undefined,
        cache: "no-store",
        signal: AbortSignal.timeout(5000),
      },
    );
    return new NextResponse(await upstream.text(), {
      status: upstream.status,
      headers: {
        "Content-Type": "application/json",
        "Cache-Control": "no-store",
      },
    });
  } catch {
    return NextResponse.json(
      { error: "운영 API에 연결할 수 없습니다. 연결 상태를 확인해 주세요." },
      { status: 503 },
    );
  }
}
