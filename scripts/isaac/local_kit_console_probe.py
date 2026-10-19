"""Enable a real Kit operator extension and check its normal API actions."""
# pyright: reportMissingImports=false
from __future__ import annotations

import argparse
import gc
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import sys
import time
import traceback
from typing import Any
from urllib.request import ProxyHandler, Request, build_opener


def save(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    if args.output.exists() or not args.output.is_absolute() or not re.fullmatch("[0-9a-f]{32}", args.run_id):
        parser.error("use a new absolute output and32hex run ID")
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root))
    cfg = json.loads(args.config.read_text(encoding="utf-8-sig"))
    if cfg["maximum_wall_time_s"] > 150 or cfg["operation_timeout_s"] > 15:
        parser.error("probe deadlines exceed bounds")
    from factory_ops.kit_client import loopback_url
    api = loopback_url(cfg["api_url"])
    args.output.mkdir(parents=True)
    started = time.monotonic()
    deadline = started + cfg["maximum_wall_time_s"]
    project_sources = ["factory_ops/__init__.py", "factory_ops/kit_client.py",
        "extensions/factory.ops.console/factory_ops_console/__init__.py",
        "extensions/factory.ops.console/factory_ops_console/extension.py"]
    manifest = root / "extensions/factory.ops.console/config/extension.toml"
    report = {"schema_version": 1, "run_id": args.run_id, "status": "starting", "stage": "preflight",
        "started_at_utc": datetime.now(timezone.utc).isoformat(), "scope": cfg["scope"],
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
        "project_source_sha256": {name: hashlib.sha256((root / name).read_bytes()).hexdigest() for name in project_sources},
        "extension_manifest_sha256": hashlib.sha256(manifest.read_bytes()).hexdigest(),
        "isaac_version": (Path(os.environ["ISAAC_PATH"]) / "VERSION").read_text().strip(),
        "python": sys.version, "operations": [], "media": [], "requires_host_closure": True,
        "four_amr_physics_verified": False, "sensor_verified": False, "physical_charging_verified": False}
    app: Any = None
    manager: Any = None
    console: Any = None
    initial = None
    extension_id = "factory.ops.console"
    code = 1
    opener = build_opener(ProxyHandler({}))

    def phase(name):
        report["stage"] = name
        report["elapsed_wall_s"] = time.monotonic() - started
        save(args.output / "result.json", report)

    def snapshot():
        with opener.open(api + "/api/snapshot", timeout=2) as response:
            return json.load(response)

    def until(predicate, label):
        local_deadline = min(deadline, time.monotonic() + cfg["operation_timeout_s"])
        latest = None
        while time.monotonic() < local_deadline:
            app.update()
            latest = console.get_probe_state()
            if predicate(latest):
                return latest
            time.sleep(.03)
        raise TimeoutError(label + ": " + json.dumps(latest, ensure_ascii=True)[:1200])

    phase("sdk_startup")
    try:
        initial = snapshot()
        if initial["mode"] != "local_model" or initial["connection"] != "connected" or initial["paused"]:
            raise ValueError("native console probe requires a connected, running local CPU model")
        report["business_run_id"] = initial["run_id"]
        report["initial_api_snapshot"] = initial
        os.environ["FACTORY_OPS_API_URL"] = api
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "hide_ui": False, "multi_gpu": False,
            "width": cfg["image_size_px"][0], "height": cfg["image_size_px"][1],
            "window_width": cfg["image_size_px"][0], "window_height": cfg["image_size_px"][1]})
        import omni.kit.app
        manager = omni.kit.app.get_app().get_extension_manager()
        phase("enable_extension")
        manager.add_path(str(root / "extensions"))
        manager.set_extension_enabled_immediate(extension_id, True)
        if not manager.is_extension_enabled(extension_id):
            raise RuntimeError("extension manager did not enable the console")
        import factory_ops_console
        console = factory_ops_console.get_instance()
        if console is None:
            raise RuntimeError("extension startup did not expose an active instance")
        phase("await_api")
        state = until(lambda s: s["connection"] == "connected" and s["run_id"] == initial["run_id"]
                      and bool(s["enabled_actions"]), "console API did not connect")
        if not state["active"] or not state["window_visible"] or state["mode"] != "local_model":
            raise RuntimeError("Kit window or mode invalid")
        if not console.select_robot("amr-3") or console.get_probe_state()["selected_robot"] != "amr-3":
            raise RuntimeError("observed AMR selection failed")
        report["selected_state"] = console.get_probe_state()
        for action, parameters in [("pause", None), ("resume", None),
                                   ("create_order", {"source": "S3", "quantity": 1})]:
            phase("action_" + action)
            until(lambda s: action in s["enabled_actions"], "action unavailable: " + action)
            submitted = console.submit_action(action, parameters)
            if not submitted["submitted"]:
                raise RuntimeError("normal console submission rejected: " + str(submitted))
            state = until(lambda s: s["last_receipt"] is not None
                and s["last_receipt"].get("command_id") == submitted["command_id"]
                and s["last_receipt"]["status"] in ("applied", "rejected"), "action receipt missing")
            actual = snapshot()
            if state["last_receipt"]["status"] != "applied" or actual["run_id"] != initial["run_id"]:
                raise RuntimeError("console operation did not apply to the observed run")
            if action in ("pause", "resume") and actual["paused"] != (action == "pause"):
                raise RuntimeError("API pause state differs from console receipt")
            report["operations"].append({"action": action, "submission": submitted,
                "receipt": state["last_receipt"], "api_revision": actual["revision"],
                "api_paused": actual["paused"], "mode": actual["mode"]})
        until(lambda s: s["revision"] >= report["operations"][-1]["api_revision"],
              "console did not observe the applied order revision")
        observed = snapshot()
        active = next((r for r in observed["robots"] if r.get("task_id") is not None), None)
        if active is not None:
            console.select_robot(active["id"])
        phase("capture_window")
        for _ in range(10):
            app.update()
        try:
            import omni.renderer_capture
            capture = omni.renderer_capture.acquire_renderer_capture_interface()
            destination = args.output / "kit-console.png"
            capture.capture_next_frame_swapchain(str(destination))
            for _ in range(15):
                app.update()
                capture.wait_async_capture()
                if destination.is_file():
                    break
            if not destination.is_file():
                raise RuntimeError("headless UI swapchain did not produce an image")
            from PIL import Image
            with Image.open(destination) as picture:
                actual_size = list(picture.size)
                picture.verify()
            report["media"].append({"path": destination.name,
                "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
                "image_size_px": actual_size,
                "source": "native Kit renderer swapchain; headless UI shown, no desktop window",
                "requires_visual_review": True})
        except Exception as error:
            report["capture_error"] = str(error)
            raise
        report["final_state"] = console.get_probe_state()
        report["final_api_snapshot"] = snapshot()
        phase("disable_extension")
        # The SDK calls IExt.on_shutdown(), then clears its __dict__. Inspect
        # manager/workspace/module state after disable instead of the dead IExt.
        import omni.ui as ui
        owned_window = console._window
        visibility_events = []
        owned_window.set_visibility_changed_fn(lambda visible: visibility_events.append(bool(visible)))
        console = None
        manager.set_extension_enabled_immediate(extension_id, False)
        for _ in range(5):
            app.update()
        gc.collect()
        remaining_window = ui.Workspace.get_window("Factory Operations")
        report["workspace_window_after_disable"] = {
            "present": remaining_window is not None,
            "visible": bool(remaining_window.visible) if remaining_window is not None else None,
            "use": "diagnostic; Workspace may retain a destroyed ImGui window handle",
        }
        report["shutdown"] = {"manager_disabled": not manager.is_extension_enabled(extension_id),
            "active_instance_removed": factory_ops_console.get_instance() is None,
            "owned_window_hidden": not owned_window.visible,
            "hide_callback_observed": False in visibility_events}
        report["visibility_events"] = visibility_events
        after_image = args.output / "kit-console-disabled.png"
        capture.capture_next_frame_swapchain(str(after_image))
        for _ in range(15):
            app.update()
            capture.wait_async_capture()
            if after_image.is_file():
                break
        if not after_image.is_file():
            raise RuntimeError("post-disable UI image missing")
        report["media"].append({"path": after_image.name,
            "sha256": hashlib.sha256(after_image.read_bytes()).hexdigest(),
            "source": "native swapchain after extension disable; manual absence check required",
            "requires_visual_review": True})
        owned_window = None
        if not all(report["shutdown"].values()):
            raise RuntimeError("extension shutdown incomplete")
        report["status"] = "passed"
        phase("component_complete")
        code = 0
    except BaseException as error:
        report.update(status="failed", failed_phase=report["stage"], error={"type": type(error).__name__,
            "message": str(error), "traceback": traceback.format_exc(limit=15)})
    finally:
        console = None
        if manager is not None and manager.is_extension_enabled(extension_id):
            manager.set_extension_enabled_immediate(extension_id, False)
        if report["status"] == "failed" and initial is not None and not initial["paused"]:
            try:
                current = snapshot()
                if current["run_id"] == initial["run_id"] and current["paused"]:
                    restore = {"command_id": "kit-probe-restore-" + args.run_id,
                        "expected_revision": current["revision"], "expected_run_id": current["run_id"],
                        "action": "resume"}
                    req = Request(api + "/api/commands", json.dumps(restore).encode(),
                                  headers={"Content-Type": "application/json"})
                    with opener.open(req, timeout=2) as response:
                        report["operator_restore_receipt"] = json.load(response)
            except Exception as error:
                report["operator_restore_error"] = str(error)
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        save(args.output / "result.json", report)
        print("LOCAL_KIT_CONSOLE_RESULT", report["status"], flush=True)
        if app is not None:
            app.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
