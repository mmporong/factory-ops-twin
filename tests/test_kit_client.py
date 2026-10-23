"""Bounded CPU client checks; no SDK, model, HTTP server or real worker needed."""
from __future__ import annotations

import ast
from collections import Counter
from concurrent.futures import Future
from copy import deepcopy
from datetime import datetime, timezone
from http.client import HTTPMessage
from io import BytesIO
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
from urllib.request import Request
import uuid

from factory_ops.kit_client import KitClient, _NoRedirect, loopback_url
from factory_ops.live_control import CONTRACT, LiveControl


def snapshot(run="run-a", revision=3, **changes):
    value = {"schema_version":1, "run_id":run, "revision":revision, "mode":"local_model",
        "sim_time_s":12., "connection":"connected", "paused":False,
        "capabilities":["pause","resume","create_order","set_equipment_availability","set_aisle_closed"],
        "robots":[{"id":"amr-1","label":"AMR 01","status":"idle","task_id":None,
            "destination":None,"speed_mps":0.,"battery_pct":None,"battery_source":"unavailable",
            "payload_id":None,"waiting_reason":None}],
        "inventory":{"stock":[],"totes":[]},"kpis":{"stock_units":None},
        "equipment":{"station_id":"CELL1","label":"검사 CELL1","available":True,"status":"available",
            "reason":None,"active_order_id":"ORD-0001","remaining_service_s":12.,
            "input_waiting_orders":1,"output_waiting_orders":0,"wip_orders":2},
        "traffic":{"closed":False,"requested_closed":False,"closure_pending":False},
        "execution_mode":"operations","dispatch_policy":"fifo",
        "events":[],"commands":[],"tasks":[]}
    value.update(changes)
    return value


def live_snapshot(run="run-a", revision=0, phase="running", **changes):
    value = snapshot(run=run, revision=revision, mode="live", paused=phase == "paused")
    value["robots"] = [dict(value["robots"][0], id="carter" + str(i), label="AMR " + str(i)) for i in range(1,5)]
    value["live_control"] = {"contract":CONTRACT, "enabled":True, "run_id":run,
        "revision":revision, "profile":"development", "primary_evaluation":False,
        "phase":phase, "all_stopped":phase == "paused", "commands":[],
        "robot_ids":[r["id"] for r in value["robots"]]}
    value.update(changes)
    return value


def console_projection(client):
    """Run the actual owner-thread formatter with inert widgets, without importing Kit."""
    path = Path(__file__).resolve().parents[1]/"extensions/factory.ops.console/factory_ops_console/extension.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    extension = next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name == "Extension")
    functions = [node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ("short","format_number")]
    functions += [node for node in extension.body if isinstance(node,ast.FunctionDef) and node.name == "_refresh"]
    namespace = {"math":math, "Counter":Counter}
    exec(compile(ast.Module(body=functions,type_ignores=[]),str(path),"exec"),namespace)
    ids = tuple(r["id"] for r in client.get_state()["snapshot"]["robots"])
    labels = {}
    view = SimpleNamespace(_alive=True,_client=client,_selected=ids[0],_row_ids=ids,
        _native_contract=CONTRACT,_robot_buttons={rid:SimpleNamespace() for rid in ids},
        _pause=SimpleNamespace(),_resume=SimpleNamespace(),_order=SimpleNamespace(),
        _fault=SimpleNamespace(),_recover=SimpleNamespace(),
        _close_aisle=SimpleNamespace(),_open_aisle=SimpleNamespace(),
        _source=SimpleNamespace(),_quantity_field=SimpleNamespace(),
        _set_label=lambda key,text,raw=None:labels.update({key:(text,raw)}))
    namespace["_refresh"](view)
    return view,labels


class ManualExecutor:
    def __init__(self):
        self.jobs = []
        self.closed = None

    def submit(self, function, *args):
        future = Future()
        future.set_running_or_notify_cancel()
        self.jobs.append((future,args))
        return future

    def shutdown(self, **kwargs):
        self.closed = kwargs


class KitClientTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.executor = ManualExecutor()
        self.client = KitClient(clock=lambda:self.now, executor=self.executor)
        self.addCleanup(self.client.close)

    @property
    def observed_snapshot(self):
        value = self.client.snapshot
        assert value is not None
        return value

    @property
    def observed_receipt(self):
        value = self.client.last_receipt
        assert value is not None
        return value

    def deliver(self, value=None, error=None):
        future = self.executor.jobs[-1][0]
        if error is not None: future.set_exception(error)
        else: future.set_result(deepcopy(value))
        self.client.tick()

    def connect(self, **changes):
        self.client.tick()
        self.deliver(snapshot(**changes))

    def poll(self, value):
        self.now += .5
        self.client.tick()
        self.deliver(value)

    def test_loopback_origin_only_and_redirects_are_blocked(self):
        for value in ("http://127.0.0.1:8765", "http://localhost:8765/", "http://[::1]:8765"):
            self.assertEqual(loopback_url(value),value.rstrip("/"))
        for value in ("https://127.0.0.1", "http://example.com", "http://0.0.0.0", "http://localhost.evil",
                      "http://user@localhost", "http://127.0.0.1/path", "http://127.0.0.1?next=x"):
            with self.subTest(value=value), self.assertRaises(ValueError): loopback_url(value)
        with self.assertRaises(ValueError):
            _NoRedirect().redirect_request(Request("http://localhost"), BytesIO(), 302, "", HTTPMessage(), "http://example.com")

    def test_one_background_request_with_short_timeout(self):
        for _ in range(20): self.client.tick()
        self.assertEqual(len(self.executor.jobs),1)
        self.assertEqual(self.executor.jobs[-1][1][1:],("GET","/api/snapshot",None,1.5))

    def test_timeout_retains_values_and_disables_commands(self):
        self.connect()
        self.now += .5
        self.client.tick()
        self.deliver(error=TimeoutError("timed out"))
        self.assertEqual(self.client.connection,"stale")
        self.assertEqual(self.observed_snapshot["run_id"],"run-a")
        self.assertEqual(self.client.enabled_actions(),[])
        self.poll(snapshot())
        self.assertEqual(self.client.connection,"connected")

    def test_fetch_age_and_source_connection_gate(self):
        self.connect()
        self.now += 3.01
        self.assertEqual(self.client.connection,"stale")
        self.assertFalse(self.client.submit_action("pause")["submitted"])
        self.client.tick()
        self.deliver(snapshot(connection="disconnected"))
        self.assertEqual(self.client.enabled_actions(),[])

    def test_run_change_keeps_old_values_until_second_matching_fetch(self):
        self.connect()
        self.poll(snapshot(run="run-b",revision=0))
        self.assertEqual(self.observed_snapshot["run_id"],"run-a")
        self.assertEqual(self.client.connection,"stale")
        self.assertEqual(self.client.enabled_actions(),[])
        self.poll(snapshot(run="run-b",revision=0))
        self.assertEqual(self.observed_snapshot["run_id"],"run-b")
        self.assertEqual(self.client.connection,"connected")

    def test_capability_paused_and_order_parameter_gates(self):
        self.connect(capabilities=[])
        self.assertFalse(self.client.submit_action("pause")["submitted"])
        self.poll(snapshot())
        self.assertIn("pause",self.client.enabled_actions())
        self.assertNotIn("resume",self.client.enabled_actions())
        for parameters in ({"source":"S4","quantity":1},{"source":"S1","quantity":True},{"source":"S1","quantity":0}):
            self.assertFalse(self.client.submit_action("create_order",parameters)["submitted"])

    def test_command_captures_uuid_observed_run_and_revision(self):
        self.connect()
        result = self.client.submit_action("create_order",{"source":"S2","quantity":2})
        self.assertTrue(result["submitted"])
        uuid.UUID(result["command_id"])
        method,path,payload,timeout = self.executor.jobs[-1][1][1:]
        self.assertEqual((method,path),("POST","/api/commands"))
        self.assertEqual(payload["expected_run_id"],"run-a")
        self.assertEqual(payload["expected_revision"],3)
        self.assertEqual(payload["parameters"],{"source":"S2","quantity":2})
        self.assertNotIn("run_id",payload)
        self.assertNotIn("requested_at",payload)
        self.assertFalse(self.client.submit_action("pause")["submitted"])

    def test_operations_equipment_and_aisle_commands_use_same_run_receipts(self):
        self.connect()
        fault = self.client.submit_action("set_equipment_availability",
            {"available":False,"reason":"  검사기 온도 상한  "})
        self.assertTrue(fault["submitted"])
        payload = self.executor.jobs[-1][1][3]
        self.assertEqual(payload["expected_run_id"],"run-a")
        self.assertEqual(payload["expected_revision"],3)
        self.assertEqual(payload["parameters"],{"available":False,"reason":"검사기 온도 상한"})
        receipt = {"command_id":fault["command_id"],"run_id":"run-a","status":"applied",
                   "reason":None,"revision":4}
        self.deliver(receipt)
        self.assertEqual(self.client.submission_state,"awaiting_snapshot")
        down = deepcopy(snapshot(revision=4,commands=[receipt]))
        down["equipment"].update(available=False,status="down",reason="검사기 온도 상한")
        self.deliver(down)
        self.assertEqual(self.client.submission_state,"confirmed_snapshot")
        self.assertIn("set_aisle_closed",self.client.enabled_actions())
        close = self.client.submit_action("set_aisle_closed",{"closed":True})
        queued = {"command_id":close["command_id"],"run_id":"run-a","status":"queued",
                  "reason":"승인된 운송과 통로 이탈 후 적용합니다","revision":4}
        self.deliver(queued)
        self.deliver(dict(down,commands=[receipt,queued]))
        self.assertEqual(self.client.submission_state,"pending_server")
        self.assertEqual(self.observed_receipt["status"],"queued")

    def test_operations_command_parameter_gates(self):
        self.connect()
        invalid = [
            ("set_equipment_availability",{}),
            ("set_equipment_availability",{"available":0}),
            ("set_equipment_availability",{"available":False,"reason":" "}),
            ("set_equipment_availability",{"available":False,"station_id":"CELL1"}),
            ("set_aisle_closed",{}),
            ("set_aisle_closed",{"closed":1}),
        ]
        for action,parameters in invalid:
            with self.subTest(action=action,parameters=parameters):
                self.assertFalse(self.client.submit_action(action,parameters)["submitted"])

    def test_native_request_is_accepted_by_existing_next_anchor_contract(self):
        self.client.tick()
        self.deliver(live_snapshot())
        self.assertEqual(self.client.enabled_actions(),["pause"])
        before = datetime.now(timezone.utc)
        result = self.client.submit_action("pause")
        payload = self.executor.jobs[-1][1][3]
        self.assertEqual(set(payload),{"command_id","run_id","expected_run_id","expected_revision","action","parameters","requested_at"})
        self.assertEqual(payload["expected_run_id"],payload["run_id"])
        self.assertEqual(payload["expected_revision"],0)
        requested = datetime.fromisoformat(payload["requested_at"])
        self.assertEqual(requested.tzinfo,timezone.utc)
        self.assertLessEqual(before,requested)
        self.assertLessEqual(requested,datetime.now(timezone.utc))
        control = LiveControl("run-a",tuple("carter" + str(i) for i in range(1,5)))
        receipt = control.submit(payload,now_s=12.,wall_now=datetime.now(timezone.utc))
        self.assertEqual((receipt["command_id"],receipt["status"]),(result["command_id"],"pending"))

    def test_live_requires_valid_contract_and_advertised_capability(self):
        self.client.tick()
        self.deliver(live_snapshot())
        invalid = [live_snapshot(live_control=None),live_snapshot(capabilities=[]),
                   live_snapshot(phase="pause_pending"),live_snapshot(connection="stale"),
                   live_snapshot(connection="disconnected")]
        for key,value in (("contract","unknown"),("run_id","old-run"),("revision",99),("enabled",False)):
            row = live_snapshot()
            row["live_control"][key] = value
            invalid.append(row)
        for row in invalid:
            with self.subTest(row=row):
                self.poll(row)
                self.assertEqual(self.client.enabled_actions(),[])
                self.assertFalse(self.client.submit_action("create_order",{"source":"S1","quantity":1})["submitted"])
        self.poll(live_snapshot())
        self.assertEqual(self.client.enabled_actions(),["pause"])
        self.poll(live_snapshot(phase="paused"))
        self.assertEqual(self.client.enabled_actions(),["resume"])

    def test_native_ui_distinguishes_reply_pending_and_source_applied(self):
        self.client.tick()
        self.deliver(live_snapshot())
        command_id = self.client.submit_action("pause")["command_id"]
        receipt = {"command_id":command_id,"run_id":"run-a","status":"applied","reason":None,"revision":1}
        self.deliver(receipt)
        self.assertEqual(self.client.submission_state,"awaiting_snapshot")
        view,labels = console_projection(self.client)
        self.assertIn("API reply (unconfirmed): applied",labels["receipt"][0])
        self.assertFalse(view._resume.enabled)
        receipt.update(status="pending")
        self.deliver(live_snapshot(revision=1,phase="pause_pending",commands=[receipt]))
        view,labels = console_projection(self.client)
        self.assertEqual(self.client.submission_state,"pending_server")
        self.assertIn("next reserved anchors",labels["control_scope"][0])
        self.assertIn("Simulation / ROS / MES / virtual charging clocks continue",labels["control_scope"][0])
        self.assertIn("Source phase: pause_pending",labels["control_scope"][0])
        self.assertIn("Pending:",labels["receipt"][0])
        self.assertIn("SOC: Not connected / source: unavailable",labels["detail"][0])
        self.assertFalse(view._pause.enabled or view._resume.enabled or view._order.enabled or view._source.enabled or view._quantity_field.enabled)
        receipt.update(status="applied",revision=2)
        self.poll(live_snapshot(revision=2,phase="paused",commands=[receipt]))
        view,labels = console_projection(self.client)
        self.assertEqual(self.client.submission_state,"confirmed_snapshot")
        self.assertIn("Source receipt: applied",labels["receipt"][0])
        self.assertTrue(view._resume.enabled)
        self.assertFalse(view._order.enabled)

    def test_native_receipt_without_source_run_cannot_confirm(self):
        self.client.tick()
        self.deliver(live_snapshot())
        command_id = self.client.submit_action("pause")["command_id"]
        receipt = {"command_id":command_id,"status":"applied","revision":1}
        self.deliver(receipt)
        self.assertEqual(self.client.submission_state,"delivery_uncertain")
        self.assertIsNone(self.client.last_receipt)
        self.deliver(live_snapshot(revision=1,phase="paused",commands=[receipt]))
        self.assertEqual(self.client.connection,"stale")
        self.assertIsNone(self.client.last_receipt)
        self.assertEqual(self.client.enabled_actions(),[])

    def test_native_run_switch_discards_pending_and_confirmed_cache_without_resend(self):
        self.client.tick()
        self.deliver(live_snapshot())
        command_id = self.client.submit_action("pause")["command_id"]
        self.deliver({"command_id":command_id,"run_id":"run-a","status":"pending","revision":0})
        self.deliver(live_snapshot(run="run-b"))
        self.assertEqual(self.client.submission_state,"previous_run")
        self.assertIsNone(self.client.last_receipt)
        self.assertIsNone(self.client.receipt_run_id)
        self.assertIsNone(self.client.receipt_expected_run_id)
        self.assertEqual(self.client.enabled_actions(),[])
        self.poll(live_snapshot(run="run-b"))
        self.assertEqual(self.client.enabled_actions(),["pause"])
        new_id = self.client.submit_action("pause")["command_id"]
        self.assertNotEqual(command_id,new_id)
        receipt = {"command_id":new_id,"run_id":"run-b","status":"applied","revision":1}
        self.deliver(receipt)
        self.deliver(live_snapshot(run="run-b",revision=1,phase="paused",commands=[receipt]))
        self.assertEqual(self.client.submission_state,"confirmed_snapshot")
        self.poll(live_snapshot(run="run-c"))
        self.assertIsNone(self.client.last_receipt)
        self.assertIsNone(self.client.receipt_expected_run_id)
        self.poll(live_snapshot(run="run-c"))
        self.assertEqual(self.client.enabled_actions(),["pause"])
        self.assertEqual(sum(job[1][1]=="POST" for job in self.executor.jobs),2)

    def test_replay_is_read_only_and_shows_recorded_equipment_receipt(self):
        self.connect(mode="replay",capabilities=[])
        self.assertEqual(self.client.enabled_actions(),[])
        recorded = {"command_id":"cell-down","run_id":"run-a","status":"applied",
                    "reason":None,"revision":3}
        replay = snapshot(mode="replay",capabilities=list(snapshot()["capabilities"]),commands=[recorded])
        replay["equipment"].update(available=False,status="down",reason="검사기 온도 상한")
        self.poll(replay)
        self.assertEqual(self.client.enabled_actions(),[])
        self.assertFalse(self.client.submit_action("pause")["submitted"])
        view,labels = console_projection(self.client)
        self.assertIn("RECORDED / READ ONLY",labels["actions_heading"][0])
        self.assertIn("Recorded command (read only): applied",labels["receipt"][0])
        self.assertIn("State: down",labels["equipment"][0])
        self.assertIn("검사기 온도 상한",str(labels["equipment"][1]))
        self.assertFalse(view._fault.enabled or view._recover.enabled
                         or view._close_aisle.enabled or view._open_aisle.enabled)

    def test_pending_is_confirmed_only_by_matching_snapshot_receipt(self):
        self.connect()
        command_id = self.client.submit_action("pause")["command_id"]
        receipt = {"command_id":command_id,"status":"pending","reason":None,"revision":3}
        self.deliver(receipt)
        self.assertEqual(self.observed_receipt["status"],"pending")
        self.deliver(snapshot(commands=[receipt]))
        self.assertEqual(self.client.submission_state,"pending_server")
        self.assertEqual(self.client.enabled_actions(),[])
        receipt.update(status="applied",revision=4)
        self.poll(snapshot(revision=4,paused=True,commands=[receipt]))
        self.assertEqual(self.client.submission_state,"confirmed_snapshot")
        self.assertIn("resume",self.client.enabled_actions())
        second = self.client.submit_action("resume")
        self.assertNotEqual(second["command_id"],command_id)

    def test_post_timeout_is_uncertain_and_never_automatically_retried(self):
        self.connect()
        command_id = self.client.submit_action("pause")["command_id"]
        self.deliver(error=TimeoutError("delivery unknown"))
        self.assertEqual(self.client.submission_state,"delivery_uncertain")
        self.assertIsNone(self.client.last_receipt)
        self.deliver(snapshot())
        self.assertEqual(self.client.enabled_actions(),[])
        receipt = {"command_id":command_id,"status":"rejected","reason":"state changed","revision":3}
        self.poll(snapshot(commands=[receipt]))
        self.assertEqual(self.observed_receipt["status"],"rejected")
        self.assertEqual(sum(job[1][1]=="POST" for job in self.executor.jobs),1)

    def test_foreign_receipt_or_regressed_revision_cannot_confirm(self):
        self.connect()
        command_id = self.client.submit_action("pause")["command_id"]
        self.deliver({"command_id":command_id,"run_id":"other-run","status":"applied","revision":4})
        self.assertEqual(self.client.submission_state,"delivery_uncertain")
        self.assertIsNone(self.client.last_receipt)
        self.deliver(snapshot(revision=2))
        self.assertEqual(self.observed_snapshot["revision"],3)
        self.assertEqual(self.client.connection,"stale")

    def test_invalid_snapshot_does_not_replace_observed_data(self):
        self.connect()
        invalid = snapshot(); invalid["robots"][0]["battery_pct"] = float("nan")
        self.poll(invalid)
        self.assertIsNone(self.observed_snapshot["robots"][0]["battery_pct"])
        self.assertEqual(self.client.connection,"stale")

    def test_new_run_rejection_is_shown_without_accepting_foreign_applied(self):
        self.connect()
        command_id = self.client.submit_action("pause")["command_id"]
        self.deliver({"command_id":command_id,"run_id":"run-b","status":"rejected",
                      "reason":"run changed","revision":0})
        self.assertEqual(self.observed_receipt["status"],"rejected")
        self.assertEqual(self.client.receipt_run_id,"run-b")
        self.assertEqual(self.client.receipt_expected_run_id,"run-a")
        self.deliver(snapshot(run="run-b",revision=0))
        self.assertEqual(self.client.connection,"stale")
        self.poll(snapshot(run="run-b",revision=0))
        self.assertEqual(self.client.connection,"connected")

    def test_snapshot_receipt_cannot_confirm_a_future_revision(self):
        self.connect()
        command_id = self.client.submit_action("pause")["command_id"]
        self.deliver({"command_id":command_id,"status":"pending","reason":None,"revision":3})
        self.deliver(snapshot(commands=[{"command_id":command_id,"status":"applied","reason":None,"revision":99}]))
        self.assertEqual(self.client.connection,"stale")
        self.assertEqual(self.observed_receipt["status"],"pending")
        self.assertEqual(self.client.enabled_actions(),[])

    def test_close_ignores_late_completion(self):
        self.connect()
        self.now += .5
        self.client.tick()
        future = self.executor.jobs[-1][0]
        self.client.close()
        future.set_result(snapshot(run="late-run"))
        self.client.tick()
        self.assertEqual(self.observed_snapshot["run_id"],"run-a")
        self.assertEqual(self.client.connection,"closed")
        self.assertEqual(self.executor.closed,{"wait":False,"cancel_futures":True})

    def test_python310_syntax_without_importing_sdk(self):
        root = Path(__file__).resolve().parents[1]
        for path in ("factory_ops/kit_client.py", "extensions/factory.ops.console/factory_ops_console/__init__.py",
                     "extensions/factory.ops.console/factory_ops_console/extension.py"):
            ast.parse((root/path).read_text(encoding="utf-8"),feature_version=(3,10))


if __name__ == "__main__":
    unittest.main()
