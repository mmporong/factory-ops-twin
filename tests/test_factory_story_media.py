"""Synthetic JSON/hash fixtures only. No real image, SDK, encoder or native PASS."""
from __future__ import annotations

import ast
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from factory_ops.material_visual import make_material_visual_state
from factory_ops.operation_story import OperationStory, PHASES
from factory_ops.visual_state import canonical_sha256

PATH = Path(__file__).resolve().parents[1]/"scripts/media/export_factory_story.py"
SPEC = importlib.util.spec_from_file_location("factory_story_export_145",PATH)
assert SPEC is not None and SPEC.loader is not None
MEDIA = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MEDIA)
RUN = "a"*32
ROBOTS = ["carter1","carter2","carter3","carter4"]
PROFILE = {"schema_version":1,"profiles":["development"],"window_frames":240,
           "sample_every_physics_steps":1,"maximum_windows":6}


def write(root,relative,value):
    path = root/relative
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",encoding="utf-8",newline="\n") as stream:
        stream.write(json.dumps(value,ensure_ascii=False,allow_nan=False)+"\n")
    return {"path":relative,"sha256":MEDIA.sha(path),"bytes":path.stat().st_size}


def fixture(stage,counts=None,bad_stock=False):
    """Write invented metadata for admission tests; overview bytes are NOT PNGs."""
    run = stage/"results/run"
    materials = [{"order_id":"O"+str(i),"tote_id":"F4-TOTE-"+str(i),"source":"S"+str(i),
                  "sku":"kit","quantity":2} for i in range(1,4)]
    session = {"scenario_id":"fleet4_transport_normal","profile":"development","primary_evaluation":False,
        "workload":{"orders":[{"order_id":m["order_id"]} for m in materials],"initial_robots":dict.fromkeys(ROBOTS,"P1"),
                    "material_flow":{"materials":materials},"load":{"ci_capacity":2}}}
    session_ref = write(stage,"inputs/session.json",session)
    write(stage,"inputs/config.json",{"g3_runtime":{"operation_story":PROFILE}})
    write(stage,"launch.json",{"run_id":RUN,"source_commit":"b"*40})
    host = {"status":"passed","run_id":RUN,"source_commit":"b"*40,"windows_closed":True,"linux_closed":True,
        "cleanup_errors":[],"windows_end":{"run_id":RUN,"closed":True,"remaining_owned_pids":[],
            "assignment_confirmed":True,"kill_on_job_close":True},
        "linux_end":{"run_id":RUN,"closure_verified":True,"cgroup_populated":False,"cgroup_pids":[],
                     "live_known_processes":[],"client":{"closed":True}}}
    write(stage,"pair-host.json",host)
    write(run,"mission-result.json",{"run_id":RUN,"status":"passed","scenario_id":session["scenario_id"],
        "origin_sim_time_s":90.,"robot_ids":ROBOTS,"session_sha256":session_ref["sha256"],
        "mission":{"completed":True,"failed":False,"censored":False,"completed_order_count":3},
        "material_flow":{"run_id":RUN,"completed":True}})
    write(run,"local45-fleet4-transport-scene.json",{"run_id":RUN,"scenario_id":session["scenario_id"],"status":"validated"})
    frames = []
    for i,phase in enumerate(PHASES):
        start,first_frame = 100.+i*10.,6000+i*600
        payloads = []
        for j,m in enumerate(materials):
            state = ("inbound_loaded","processing","outbound_loaded","complete","complete","complete")[i] if j == 0 or i == 5 else "source_waiting"
            location = {"inbound_loaded":"robot_carter1","processing":"cell","outbound_loaded":"robot_carter1","complete":"out","source_waiting":"source_"+m["source"]}[state]
            payloads.append({"order_id":m["order_id"],"payload_id":"PAY"+str(j),"state":state,
                "state_version":i,"owner":"carter1" if "loaded" in state else location,
                "location":location,"slot":0,"robot_id":"carter1" if "loaded" in state else None})
        signals = {"ci_slots_used":0,"co_slots_used":0,"cell_processing":i == 1,"orders_completed":3 if i == 5 else 1 if i >= 3 else 0}
        projection = {"scope":"synthetic CPU fixture","payloads":payloads,"signals":signals,
                      "physical_loading_validated":False,"ground_truth_control_input":False}
        visual = {"schema_version":1,"run_id":RUN,"scenario_id":session["scenario_id"],"sequence":i,
            "emitted_sim_time_ns":round((start-.05)*1e9),"ledger_seq":i+1,"event_hash":str(i)*64,
            "projection_sha256":canonical_sha256(projection),"projection":projection}
        state_hash = canonical_sha256(visual)
        visual_ref = write(run,"visual-state-snapshots/%08d-%s.json" % (i,state_hash),
            {"schema_version":1,"state_hash":state_hash,"visual_state":visual})
        event = {"run_id":RUN,"scenario_id":session["scenario_id"],"source_sequence":i,
            "source_state_hash":state_hash,"source_projection_sha256":visual["projection_sha256"],
            "source_ledger_seq":i+1,"source_event_hash":visual["event_hash"],
            "source_emitted_sim_time_ns":visual["emitted_sim_time_ns"],"source_state_snapshot":visual_ref,
            "applied_sim_time_ns":round((start-.02)*1e9),"applied_physics_frame":first_frame-1,
            "signals":signals,"displayed_payloads":[{k:p[k] for k in ("order_id","payload_id","state","location","slot")} for p in payloads]}
        totes = [{"tote_id":m["tote_id"],"state":"available" if i == 5 else "filled","location":m["source"],
                  "owner":None,"request_id":None if i == 5 else m["order_id"]} for m in materials]
        trips,loads,intents = [],{},{}
        if i in (4,5):
            for j,m in enumerate(materials if i == 5 else materials[:1]):
                trips.append({"tote_id":m["tote_id"],"order_id":m["order_id"],"source":"CI","destination":m["source"],
                    "robot_id":"carter3","state":"complete" if i == 5 else "started","task_id":"empty:"+m["order_id"]})
            if i == 4:
                totes[0].update(state="returning",location="IN_TRANSIT",owner="carter3")
                loads = {"carter3":materials[0]["tote_id"]}
                intents = {"carter3":{"robot_id":"carter3","payload_id":materials[0]["tote_id"],
                    "task_id":"empty:O1","transition":"drop","station":"S1"}}
        source = {"run_id":RUN,"revision":i,"completed":i == 5,"empty_trips":trips,"active_intents":intents,
            "inventory":{"totes":totes,"robot_loads":loads,"stock":[{"location":"S"+str(j),"sku":"kit",
                "on_hand":True if bad_stock and i == 0 else 20-i,"reserved":0} for j in range(1,4)]}}
        material = make_material_visual_state(source,run_id=RUN,scenario_id=session["scenario_id"],source_sim_time_s=start-.05,
            materials=materials,robot_ids=ROBOTS,ci_capacity=2)
        write(run,"material-visual-snapshots/%08d-%s.json" % (i,canonical_sha256(material)),material)
        dashboard = {"schema_version":1,"run_id":RUN,"revision":i,"mode":"live","connection":"connected","paused":False,
            "sim_time_s":start-.01-90.,"capabilities":[],"tasks":[],"commands":[],"events":[],"inventory":{"stock":[],"totes":[]},"kpis":{},
            "robots":[{"id":r,"label":r,"status":"moving","task_id":"T"+str(i),"task_type":"inbound","task_label":"자재 운반",
                "destination":"CI","payload_id":"PAY0","speed_mps":.2,"battery_pct":None if r == "carter4" else 72.5,
                "battery_source":"unavailable" if r == "carter4" else "authored_virtual_energy_model",
                "waiting_reason":None,"observed_sim_time_s":start-.01} for r in ROBOTS]}
        raw = (json.dumps(dashboard,ensure_ascii=False)+"\n").encode()
        import hashlib
        digest = hashlib.sha256(raw).hexdigest()
        dash_ref = write(run,"factory-story-state/"+digest+".json",dashboard)
        for offset in range((counts or [4]*6)[i]):
            stamp,frame = start+offset/60,first_frame+offset
            image = run/("media/test-%04d.bin" % len(frames))
            image.parent.mkdir(exist_ok=True)
            image.write_bytes(("CPU metadata only %d" % len(frames)).encode())
            tag = {"run_id":RUN,"window_id":"factory-"+phase,"phase":phase,"start_frame":first_frame,
                "start_sim_time_ns":round(start*1e9),"physics_frame":frame,"sim_time_ns":round(stamp*1e9),
                "sample_index":offset,"profile_sha256":OperationStory(PROFILE,RUN,3).profile_hash,
                "source_visual_sequence":i,"source_material_revision":i,
                "dashboard":{"status":"observed",**dash_ref,"source_world_time_s":start-.01,"age_s":stamp-(start-.01)}}
            gt = {r:{"position_xyz_m":[offset*.05 if r in {"carter1","carter3"} else 0.,0.,.1],
                     "orientation_wxyz":[1.,0.,0.,0.]} for r in ROBOTS}
            frames.append({"index":len(frames),"sim_time_s":stamp,"physics_frame":frame,"ground_truth":gt,
                "sources":{"overview":{"path":image.relative_to(run).as_posix(),"sha256":MEDIA.sha(image),"bytes":image.stat().st_size,"width":1280,"height":720}},
                "visual_state":{"run_id":RUN,"scenario_id":session["scenario_id"],"applied_sequence":i,"applied_state_hash":state_hash,
                    "applied_snapshot":visual_ref,"latest_event":event,"factory_story":tag,
                    "native_empty_totes":{"captured_physics_frame":frame,"captured_sim_time_s":stamp,
                        "applied":{"document":material,"applied_sim_time_s":start-.02,"applied_physics_frame":first_frame-1}}}})
    write(run,"media/frames.json",{"schema_version":1,"frames":frames})
    return frames


class FactoryStoryMediaTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.stage = Path(self.temp.name)/"closed-stage"
        self.frames = fixture(self.stage)
        self.run = self.stage/"results/run"

    def replace_index(self):
        write(self.run,"media/frames.json",{"schema_version":1,"frames":self.frames})

    def test_other_moving_robot_cannot_substitute_for_stationary_loaded_carrier(self):
        for frame in self.frames:
            if frame["visual_state"]["factory_story"]["phase"] == "finished_goods_transport":
                frame["ground_truth"]["carter1"]["position_xyz_m"] = [0.,0.,.1]
        self.replace_index()
        with self.assertRaisesRegex(ValueError,"own loaded carrier"):
            MEDIA.prepare(self.stage)

    def test_loaded_in_place_rotation_is_valid_observed_carrier_motion(self):
        rows=[{"carrier_poses":{"carter2":{"position_xyz_m":[0.,0.,.1],
              "orientation_wxyz":[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]}}} for yaw in (0.,.5)]
        proof=MEDIA.carrier_motion(rows)
        self.assertEqual(proof["robots"]["carter2"]["maximum_translation_from_first_m"],0.)
        self.assertAlmostEqual(proof["robots"]["carter2"]["maximum_rotation_from_first_rad"],.5)

    def test_parallel_clip_requires_two_observed_moving_robots(self):
        rows=[{"carrier_poses":{r:{"position_xyz_m":[offset if r == "carter1" else 0.,0.,.1],
              "orientation_wxyz":[1.,0.,0.,0.]} for r in ("carter1","carter4")}} for offset in (0.,.5)]
        with self.assertRaisesRegex(ValueError,"observed motion"):
            MEDIA.carrier_motion(rows, minimum_robots=2)
        rows[-1]["carrier_poses"]["carter4"]["position_xyz_m"][0]=.5
        self.assertEqual(MEDIA.carrier_motion(rows, minimum_robots=2)["qualifying_robot_count"],2)

    def test_cold_prepare_uses_each_applied_state_and_never_final_values(self):
        with patch.object(MEDIA.subprocess,"Popen",side_effect=AssertionError("no processes allowed")), patch.dict("sys.modules",{"PIL":None,"isaacsim":None}):
            plan = MEDIA.prepare(self.stage)
        self.assertEqual([c["phase"] for c in plan["chapters"]],list(PHASES))
        self.assertEqual(plan["frames"][0]["overlay"]["stock_units"],60)
        self.assertEqual(plan["frames"][-1]["overlay"]["stock_units"],45)
        self.assertEqual(plan["frames"][0]["overlay"]["shipped_orders"],0)
        self.assertEqual(plan["frames"][-1]["overlay"]["shipped_orders"],3)
        self.assertEqual(plan["frames"][0]["overlay"]["returned_totes"],0)
        self.assertEqual(plan["frames"][-1]["overlay"]["returned_totes"],3)
        self.assertIsNone(plan["frames"][0]["overlay"]["robots"][3]["battery_pct"])
        self.assertTrue(all(c["partial_window"] for c in plan["chapters"]))
        self.assertEqual(plan["native_overview_size"],[1280,720])
        self.assertIs(plan["interpolated_or_upsampled_motion"],False)

    def test_stale_missing_dashboard_and_old_robot_fields_are_unobserved(self):
        tag = self.frames[0]["visual_state"]["factory_story"]
        (self.run/tag["dashboard"]["path"]).unlink()
        self.frames[4]["visual_state"]["factory_story"]["dashboard"] = {"status":"stale_or_future","age_s":3.}
        self.replace_index()
        # An unrelated final snapshot must never serve as a fallback.
        write(self.run,"dashboard-state.json",{"run_id":RUN,"kpis":{"stock_units":9999}})
        plan = MEDIA.prepare(self.stage)
        self.assertEqual(plan["frames"][0]["dashboard_source"]["status"],"missing_checkpoint")
        self.assertTrue(all(not r["observed"] and r["speed_mps"] is None for r in plan["frames"][0]["overlay"]["robots"]))
        self.assertEqual(plan["frames"][0]["overlay"]["stock_units"],60)
        robots = deepcopy(plan["frames"][-1]["overlay"]["robots"])
        dashboard = {"robots":[{**r,"observed_sim_time_s":0.} for r in robots]}
        view = MEDIA.overlay_view({"signals":{"orders_completed":0,"cell_processing":False,"ci_slots_used":0,"co_slots_used":0}},None,dashboard,ROBOTS,100.)
        self.assertTrue(all(not r["observed"] for r in view["robots"]))

    def test_dashboard_clock_conversion_hash_and_run_boundaries(self):
        tag = deepcopy(self.frames[0]["visual_state"]["factory_story"])
        # Actual elapsed=9.99 and World=99.99 are valid with origin90.
        self.assertIsNotNone(MEDIA.dashboard_view(self.run,tag,self.frames[0],RUN,ROBOTS,90.,{},{} )[0])
        with self.assertRaisesRegex(ValueError,"origin mapping"):
            MEDIA.dashboard_view(self.run,tag,self.frames[0],RUN,ROBOTS,0.,{},{} )
        tag["dashboard"]["sha256"] = "f"*64
        with self.assertRaises(ValueError):
            MEDIA.dashboard_view(self.run,tag,self.frames[0],RUN,ROBOTS,90.,{},{} )
        tag = deepcopy(self.frames[0]["visual_state"]["factory_story"])
        with self.assertRaisesRegex(ValueError,"another run"):
            MEDIA.dashboard_view(self.run,tag,self.frames[0],"other",ROBOTS,90.,{},{} )

    def test_closed_ownership_and_success_required_before_media(self):
        path = self.stage/"pair-host.json"
        host = json.loads(path.read_text())
        for field in ("windows_closed","linux_closed"):
            changed = {**host,field:False}
            write(self.stage,"pair-host.json",changed)
            with self.subTest(field=field), self.assertRaisesRegex(ValueError,"not closed"):
                MEDIA.prepare(self.stage)
        write(self.stage,"pair-host.json",host)
        host["windows_end"]["remaining_owned_pids"] = [123]
        write(self.stage,"pair-host.json",host)
        with self.assertRaisesRegex(ValueError,"owned OS trees"):
            MEDIA.prepare(self.stage)
        host["windows_end"]["remaining_owned_pids"] = []
        host["status"] = "failed"
        write(self.stage,"pair-host.json",host)
        with self.assertRaisesRegex(ValueError,"complete normal"):
            MEDIA.prepare(self.stage)

    def test_closed_relay_owner_cannot_hide_failed_endpoint(self):
        host = json.loads((self.stage/"pair-host.json").read_text())
        relay = {"run_id": RUN, "status": "closed", "windows_closed": True, "linux_closed": True,
                 "cleanup_errors": [], "endpoints": {
                     side: {"run_id": RUN, "side": side, "status": "stopped", "closed": True,
                            "cleanup_errors": []} for side in ("windows", "linux")}}
        host.update(isolated_topic_relay=True, relay_pair=relay)
        write(self.stage,"pair-host.json",host)
        self.assertEqual(MEDIA.prepare(self.stage)["run_id"], RUN)
        for field, value in (("status", "failed"), ("closed", False), ("run_id", "foreign"),
                             ("error", "peer failed"), ("cleanup_errors", ["stop failed"])):
            changed = deepcopy(host)
            changed["relay_pair"]["endpoints"]["windows"][field] = value
            write(self.stage,"pair-host.json",changed)
            with self.subTest(field=field), self.assertRaisesRegex(ValueError,"relay endpoints"):
                MEDIA.prepare(self.stage)
        changed = deepcopy(host)
        changed["relay_pair"]["endpoints"].pop("linux")
        write(self.stage,"pair-host.json",changed)
        with self.assertRaisesRegex(ValueError,"relay endpoints"):
            MEDIA.prepare(self.stage)

    def test_missing_phase_sparse_window_or_wrong_source_image_refused(self):
        original = deepcopy(self.frames)
        self.frames = self.frames[:-4]
        self.replace_index()
        with self.assertRaisesRegex(ValueError,"missing phases"):
            MEDIA.prepare(self.stage)
        self.frames = deepcopy(original)
        self.frames[1]["physics_frame"] += 1
        self.replace_index()
        with self.assertRaises(ValueError):
            MEDIA.prepare(self.stage)
        self.frames = deepcopy(original)
        self.frames[0]["sources"]["overview"]["width"] = 640
        self.replace_index()
        with self.assertRaisesRegex(ValueError,"1280x720"):
            MEDIA.prepare(self.stage)
        self.frames = original
        (self.run/self.frames[0]["sources"]["overview"]["path"]).write_bytes(b"changed")
        self.replace_index()
        with self.assertRaisesRegex(ValueError,"hash/size"):
            MEDIA.prepare(self.stage)

    def test_applied_visual_material_hashes_future_state_and_bad_counts_fail(self):
        row = self.frames[0]
        row["visual_state"]["latest_event"]["signals"]["orders_completed"] = 999
        self.replace_index()
        with self.assertRaises(ValueError):
            MEDIA.prepare(self.stage)
        self.frames = fixture(self.stage)
        self.frames[0]["visual_state"]["native_empty_totes"]["applied"]["document"]["source_sha256"] = "f"*64
        self.replace_index()
        with self.assertRaises((ValueError,FileNotFoundError)):
            MEDIA.prepare(self.stage)
        self.frames = fixture(self.stage)
        self.frames[0]["visual_state"]["native_empty_totes"]["applied"]["applied_sim_time_s"] = 101.
        self.replace_index()
        with self.assertRaisesRegex(ValueError,"after capture"):
            MEDIA.prepare(self.stage)
        fixture(self.stage,bad_stock=True)
        with self.assertRaisesRegex(ValueError,"nonnegative integer"):
            MEDIA.prepare(self.stage)

    def test_three_second_cap_preserves_original_frame_order_and_partial_windows(self):
        fixture(self.stage,counts=[240,4,4,4,4,4])
        plan = MEDIA.prepare(self.stage)
        chapter = plan["chapters"][0]
        self.assertEqual((chapter["source_frames"],chapter["selected_frames"]),(240,180))
        self.assertFalse(chapter["partial_window"])
        selected = [r for r in plan["frames"] if r["phase"] == PHASES[0]]
        self.assertEqual([r["physics_frame"] for r in selected],list(range(6000,6180)))
        self.assertEqual(plan["playback_speed"],1)
        self.assertTrue(plan["chapter_cuts"])

    def test_gif_delays_and_output_ownership_without_encoding(self):
        delays = MEDIA.gif_delays([{"duration_time":str(v)} for v in (.02,.03)*20])
        self.assertEqual(delays["sum_ms"],1000)
        for rows in ([],[{"duration_time":"0.1"}],[{"duration_time":"nan"}]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                MEDIA.gif_delays(rows)
        for output in (self.stage,self.stage/"new-output"):
            with self.assertRaisesRegex(ValueError,"outside"):
                MEDIA.export(self.stage,output,Path("unused-font"))
        with self.assertRaises(ValueError):
            MEDIA.safe_file(self.run,"../mission-result.json")

    def test_python310_ast_without_pillow_or_sdk(self):
        for path in (PATH,Path(__file__)):
            ast.parse(path.read_text(encoding="utf-8"),feature_version=(3,10))

    def test_media_id_projection_uses_recorded_camera_and_rejects_outside(self):
        camera={"profile_camera_id":"portfolio_overview","readback_status":"verified","coordinate_frame":"world",
            "prim_path":"/World/FactoryOverviewCamera","local_transform_row_major":[1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,2.,3.,4.,1.],
            "focal_length_mm":24.,"horizontal_aperture_mm":24.,"vertical_aperture_mm":13.5}
        scene={"scene_extension":{"camera_overrides":{"cameras":[camera]}}}
        observed=MEDIA.overview_readback(scene)
        self.assertEqual(MEDIA.project_id([2.,3.,-6.],observed),[640.,360.])
        self.assertEqual(MEDIA.project_id([3.,3.,-6.],observed),[768.,360.])
        self.assertIsNone(MEDIA.project_id([2.,3.,5.],observed))
        self.assertIsNone(MEDIA.project_id([100.,3.,-6.],observed))
        self.assertIsNone(MEDIA.project_id([2.,3.,-6.],None))


if __name__ == "__main__":
    unittest.main()
