import { proxy } from "../../../lib/proxy";
export function POST(request: Request) {
  return proxy("commands", request);
}
