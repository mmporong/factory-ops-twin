"""Native Kit window backed exclusively by observed HTTP ledger state."""
from __future__ import annotations

from collections import Counter
import math
import os
from pathlib import Path
import sys
import time

import omni.ext
import omni.kit.app
import omni.ui as ui

from . import _set_instance


def short(value, fallback="Server text (hover)") -> str:
    if value is None:
        return "-"
    text = str(value)
    return text if text.isascii() else fallback


def format_number(value, decimals=1) -> str:
    if type(value) not in (int, float) or not math.isfinite(value):
        return "-"
    return f"{value:.{decimals}f}"


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        project = Path(__file__).resolve().parents[3]
        if str(project) not in sys.path:
            sys.path.insert(0, str(project))
        from factory_ops.kit_client import KitClient

        self._alive = True
        self._selected = None
        self._last_render = 0.
        self._row_ids = None
        self._labels, self._rendered, self._raw, self._robot_buttons = {}, {}, {}, {}
        self._client = KitClient(os.environ.get("FACTORY_OPS_API_URL", "http://127.0.0.1:8765"))
        font = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts/malgun.ttf"
        self._font_path = str(font).replace("\\", "/") if font.is_file() else None
        self._quantity = ui.SimpleIntModel(1)
        self._quantity.set_min(1)
        text_style = {"color": 0xFF49392C, "font_size": 14}
        if self._font_path:
            text_style["font"] = self._font_path
        style = {
            "Label": text_style, "Tooltip": {**text_style, "background_color": 0xFFFFFFFF},
            "Button": {"background_color": 0xFFF6EADC, "border_radius": 5},
            "Button:hovered": {"background_color": 0xFFF5DEC4},
            "Button:pressed": {"background_color": 0xFFEBC8A4},
            "Button:disabled": {"background_color": 0xFFF0ECE7},
            "Button.Label": text_style,
            "Button.Label:hovered": text_style,
            "Button.Label:pressed": text_style,
            "Button.Label:disabled": {**text_style, "color": 0xFF756454},
            "Field": {**text_style, "background_color": 0xFFFFFFFF},
            "ComboBox": {**text_style, "background_color": 0xFFFFFFFF,
                         "secondary_color": 0xFFF6EADC, "selected_color": 0xFFF5DEC4},
            "ComboBox:hovered": {"background_color": 0xFFF5DEC4},
            "ScrollingFrame": {"background_color": 0xFFFFFFFF, "secondary_color": 0xFFB5A899,
                               "scrollbar_size": 8},
        }
        self._window = ui.Window("Factory Operations", width=480, height=720)
        with self._window.frame:
            with ui.ZStack(style=style):
                ui.Rectangle(width=ui.Percent(100), height=ui.Percent(100),
                             style={"background_color": 0xFFFFFFFF})
                with ui.HStack(spacing=0):
                    ui.Spacer(width=10)
                    with ui.VStack(spacing=6):
                        ui.Spacer(height=6)
                        self._label("title", "FACTORY / OPERATOR CONSOLE", 24)
                        self._label("context", "Waiting for API", 42)
                        self._label("connection", "Disconnected - actions disabled", 26)
                        with ui.ScrollingFrame(height=ui.Fraction(1),
                                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                            with ui.VStack(spacing=7, height=0):
                                self._label("scope", "Observed ledger only; no stage/physics controls", 42)
                                self._label("fleet_heading", "OBSERVED VEHICLES", 20)
                                self._robot_frame = ui.Frame(height=0)
                                self._label("detail", "Select an observed vehicle", 132)
                                self._label("inventory", "Inventory / totes: not connected", 76)
                                self._label("charging", "Charging: not connected", 72)
                                self._label("mes", "Mock MES: not connected", 86)
                                self._label("events_heading", "RECENT EVENTS (8)", 20)
                                self._label("events", "Not connected", 150)
                        with ui.VStack(height=0, spacing=6):
                            self._label("actions_heading", "WHOLE OPERATION", 20)
                            with ui.HStack(height=32, spacing=6):
                                self._pause = ui.Button("Pause all operations", clicked_fn=lambda: self.submit_action("pause"))
                                self._resume = ui.Button("Resume all operations", clicked_fn=lambda: self.submit_action("resume"))
                            with ui.HStack(height=32, spacing=6):
                                self._source = ui.ComboBox(0, "S1", "S2", "S3", width=80)
                                ui.IntField(model=self._quantity, width=70)
                                self._order = ui.Button("Create order", clicked_fn=self._create_order)
                            self._label("receipt", "No command submitted", 80)
                        ui.Spacer(height=6)
                    ui.Spacer(width=10)
        self._pause.enabled = self._resume.enabled = self._order.enabled = False
        self._subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update)
        _set_instance(self)

    def _label(self, key, value, height):
        self._labels[key] = ui.Label(value, height=height, word_wrap=True)
        self._rendered[key] = value

    def _set_label(self, key, value, raw=None):
        if self._rendered.get(key) != value:
            self._labels[key].text = value
            self._rendered[key] = value
        self._raw[key] = value if raw is None else raw
        self._labels[key].tooltip = str(self._raw[key])

    def select_robot(self, robot_id):
        state = self._client.get_state()
        robots = (state["snapshot"] or {}).get("robots", [])
        if self._alive and any(r["id"] == robot_id for r in robots):
            self._selected = robot_id
            self._refresh()
            return True
        return False

    def _create_order(self):
        index = self._source.model.get_item_value_model().as_int
        self.submit_action("create_order", {"source": ("S1", "S2", "S3")[index],
                                            "quantity": self._quantity.as_int})

    def submit_action(self, action, parameters=None):
        if not self._alive or not self._window.visible:
            return {"submitted": False, "reason": "Console is not active and visible"}
        result = self._client.submit_action(action, parameters)
        if not result["submitted"]:
            self._set_label("receipt", short(result["reason"]), result["reason"])
        self._refresh()
        return result

    def _on_update(self, event):
        if not self._alive or not self._window.visible:
            return
        self._client.tick()
        if time.monotonic() - self._last_render >= .1:
            self._last_render = time.monotonic()
            self._refresh()

    def _refresh(self):
        if not self._alive:
            return
        state = self._client.get_state()
        snap = state["snapshot"] or {}
        robots = snap.get("robots", [])
        ids = tuple(r["id"] for r in robots)
        if self._selected not in ids:
            self._selected = ids[0] if ids else None
        if ids != self._row_ids:
            self._robot_frame.clear()
            self._robot_buttons = {}
            with self._robot_frame:
                with ui.VStack(spacing=4, height=0):
                    for robot_id in ids:
                        self._robot_buttons[robot_id] = ui.Button(short(robot_id), height=28,
                            clicked_fn=lambda rid=robot_id: self.select_robot(rid))
            self._row_ids = ids
        for robot in robots:
            button = self._robot_buttons[robot["id"]]
            button.text = ("> " if robot["id"] == self._selected else "  ") + short(robot["label"], "Vehicle") + " / " + short(robot["status"])
            button.tooltip = str(robot)
        self._set_label("context", "Run: " + short(snap.get("run_id")) + "\nMode: " + short(snap.get("mode"))
                        + " / rev " + str(snap.get("revision", "-")) + " / sim " + format_number(snap.get("sim_time_s")) + " s",
                        {key: snap.get(key) for key in ("run_id", "mode", "revision", "sim_time_s")})
        age = state["fetch_age_s"]
        self._set_label("connection", state["connection"].upper() + " / fetch age " + ("-" if age is None else f"{age:.1f}s"), state["error"] or state["connection"])
        self._set_label("scope", short(snap.get("model_scope"), "Server scope (hover)") if snap else "Observed ledger only; no physics controls", snap.get("model_scope", "Scope not provided"))
        selected = next((r for r in robots if r["id"] == self._selected), None)
        if selected:
            soc = "Not connected" if selected["battery_pct"] is None else f"{selected['battery_pct']:.1f}%"
            lines = ["Selected: " + short(selected["id"]),
                     "Task: " + short(selected.get("task_id")) + " / " + short(selected.get("task_type")),
                     "Destination: " + short(selected["destination"]) + " / speed: " + format_number(selected["speed_mps"], 2) + " m/s",
                     "SOC: " + soc + " / source: " + short(selected["battery_source"]),
                     "Payload: " + short(selected["payload_id"]), "Wait: " + short(selected["waiting_reason"], "Server wait reason (hover)")]
            self._set_label("detail", "\n".join(lines), selected)
        else:
            self._set_label("detail", "No observed vehicles")
        inventory = snap.get("inventory", {})
        stock, totes = inventory.get("stock", []), inventory.get("totes", [])
        stock_lines = [f"{short(s.get('location'))}: on hand {s.get('on_hand', '?')} / reserved {s.get('reserved', '?')}" for s in stock if isinstance(s, dict)]
        states = Counter(short(t.get("state")) for t in totes if isinstance(t, dict))
        self._set_label("inventory", "INVENTORY / TOTES\n" + ("\n".join(stock_lines) if stock else "Not connected")
                        + "\nTotes: " + (str(dict(states)) if totes else "Not connected"), inventory)
        charging = snap.get("charging")
        charge_lines = [f"{short(d.get('id'))}: {short(d.get('phase'))} / owner {short(d.get('owner'))}"
                        for d in (charging or {}).get("docks", []) if isinstance(d, dict)]
        self._set_label("charging", "CHARGING\n" + ("\n".join(charge_lines) + "\nQueue: " + short((charging or {}).get("queue")) if charging else "Not connected"), charging)
        mes = snap.get("integrations", {}).get("mock_mes")
        if isinstance(mes, dict):
            sources = mes.get("sources", [])
            last = mes.get("receipts", [])[-1:]
            mes_text = "MOCK MES / " + short(mes.get("connection")) + "\n" + "; ".join(f"{short(s.get('id'))}: {format_number(s.get('age_s'))}s {short(s.get('connection'))}" for s in sources)
            mes_text += "\nEquipment observations: " + str(len(mes.get("equipment", []))) + " (informational only)"
            if last:
                mes_text += "\nReceipt: " + short(last[0].get("type")) + " / " + short(last[0].get("status")) + " / " + short(last[0].get("reason"))
        else:
            mes_text = "MOCK MES / Not connected"
        self._set_label("mes", mes_text, mes)
        receipt = state["last_receipt"]
        receipt_text = "Command: " + state["submission_state"]
        if receipt:
            receipt_text += "\n" + short(receipt["status"]) + " / " + short(receipt.get("reason")) + "\nRun: " + short(state["receipt_run_id"]) + "\n" + receipt["command_id"]
        elif state["error"]:
            receipt_text += "\n" + short(state["error"])
        self._set_label("receipt", receipt_text, {"receipt": receipt, "run_id": state["receipt_run_id"], "error": state["error"]})
        events = snap.get("events", [])[-8:]
        self._set_label("events", "\n".join(format_number(e.get("sim_time_s")) + "s / " + short(e.get("message"), short(e.get("type")) + " (hover for server text)") for e in reversed(events)) or "No observed events", events)
        enabled = state["enabled_actions"]
        self._pause.enabled, self._resume.enabled, self._order.enabled = (action in enabled for action in ("pause", "resume", "create_order"))

    def get_probe_state(self):
        state = self._client.get_state()
        snap = state["snapshot"] or {}
        return {"active": self._alive, "window_visible": self._alive and self._window.visible,
                "run_id": snap.get("run_id"), "mode": snap.get("mode"), "connection": state["connection"],
                "revision": snap.get("revision"), "selected_robot": self._selected,
                "rendered_labels": dict(self._rendered), "raw_server_labels": dict(self._raw),
                "enabled_actions": state["enabled_actions"] if self._alive and self._window.visible else [],
                "last_receipt": state["last_receipt"], "receipt_run_id": state["receipt_run_id"],
                "receipt_expected_run_id": state["receipt_expected_run_id"],
                "submission_state": state["submission_state"], "error": state["error"],
                "font_path": self._font_path, "font_glyphs_verified": False,
                "client_source_path": str(Path(sys.modules["factory_ops.kit_client"].__file__).resolve())}

    def on_shutdown(self):
        self._alive = False
        self._subscription = None
        self._client.close()
        self._window.visible = False
        self._window.destroy()
        _set_instance(None)
