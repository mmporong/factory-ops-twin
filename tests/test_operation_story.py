import unittest
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest.mock import patch
from factory_ops.operation_story import OperationStory


PROFILE = {"schema_version":1,"profiles":["development"],"window_frames":240,
           "sample_every_physics_steps":1,"maximum_windows":6}


class OperationStoryTests(unittest.TestCase):
    def state(self, state="inbound_loaded", shipped=0, processing=False):
        return {"run_id":"r","sequence":3,"projection":{"payloads":[{"state":state}],
            "signals":{"orders_completed":shipped,"cell_processing":processing}}}

    def material(self, state="empty", complete=False):
        return {"run_id":"r","material_revision":2,"empty_totes":[{"state":state}],
                "source_snapshot":{"completed":complete}}

    def test_no_first_motion_coupon_and_exact_bounded_source_frames(self):
        s=OperationStory(PROFILE,"r",3)
        s.observe(1,10,self.state("source_waiting"),None,moving=True)
        self.assertIsNone(s.due())
        s.observe(2,20,self.state(),None,moving=False)
        self.assertIsNone(s.due())
        for frame in range(3,243):
            s.observe(frame,frame*100,self.state(),None,moving=True)
            sample=s.due()
            self.assertEqual(sample["sample_index"],frame-3)
            self.assertIsNone(s.due())
        s.observe(243,24300,self.state(),None,moving=True)
        self.assertIsNone(s.due())
        self.assertEqual(s.emitted,240)
        self.assertEqual([e["event"] for e in s.events],["window_started","window_ended"])

    def test_all_six_phases_follow_source_state_and_completion_needs_returns(self):
        s=OperationStory(PROFILE,"r",3)
        cases=[(self.state(),self.material(),"material_supply"),
               (self.state("processing",processing=True),self.material(),"cell_processing"),
               (self.state("outbound_loaded"),self.material(),"finished_goods_transport"),
               (self.state("complete",shipped=1),self.material(),"first_shipment"),
               (self.state("complete",shipped=2),self.material("returning"),"empty_container_return")]
        frame=0
        for v,m,phase in cases:
            s.observe(frame,frame*100,v,m,moving=True)
            self.assertEqual(s.due()["phase"],phase)
            frame+=240
        s.observe(frame,frame*100,self.state("complete",shipped=3),self.material("available"),moving=False)
        self.assertIsNone(s.due())
        s.observe(frame+1,(frame+1)*100,self.state("complete",shipped=3),self.material("available",True),moving=False)
        self.assertEqual(s.due()["phase"],"closed_loop_complete")
        s.close()
        self.assertEqual(s.events[-1]["event"],"window_terminal_ended")

    def test_run_mismatch_and_broadened_budget_rejected(self):
        with self.assertRaises(ValueError):OperationStory({**PROFILE,"maximum_windows":20},"r",3)
        s=OperationStory(PROFILE,"r",3);v=self.state();v["run_id"]="other"
        with self.assertRaises(ValueError):s.observe(1,10,v,None,moving=True)

    def test_local_runtime_tags_only_the_hash_matched_applied_snapshot(self):
        from factory_ops.fleet_transport import prepare_bundle
        from scripts.isaac.local45_fleet4_transport_scene import Local45FleetLifecycle
        run="a"*32
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder); bundle=root/"bundle"
            prepare_bundle(bundle,config_dir=Path(__file__).resolve().parents[1]/"configs/factory-fleet4-normal3")
            life=Local45FleetLifecycle(bundle,run,lidar_backend="physx_planar")
            life.output=root/"run";life.output.mkdir()
            doc=self.state();doc["run_id"]=run
            from factory_ops.visual_state import canonical_sha256
            state_hash=canonical_sha256(doc)
            raw=json.dumps({"schema_version":1,"state_hash":state_hash,"visual_state":doc}).encode();snapshot=life.output/"applied.json";snapshot.write_bytes(raw)
            life.visuals=SimpleNamespace(applied_sequence=3,applied_state_hash=state_hash,last_event={"source_state_snapshot":{
                "path":"applied.json","sha256":hashlib.sha256(raw).hexdigest()}})
            life.story_motion={"carter1":(9,True)}
            with patch("scripts.isaac.local45_fleet4_transport_scene.FleetTransportLifecycle.step"), \
                 patch("scripts.isaac.local45_fleet4_transport_scene.FleetTransportLifecycle.after_control"), \
                 patch("scripts.isaac.local45_fleet4_transport_scene.FleetTransportLifecycle.capture_due",return_value=False), \
                 patch("scripts.isaac.local45_fleet4_transport_scene.FleetTransportLifecycle.capture_metadata",return_value={}):
                life.step(1.,10)
                life.after_control("carter1",{"physics_frame":11,"sim_time_s":1.1,
                    "authorization":{"mode":"navigate","valid":True},
                    "motion_profile":{"input_command":{"linear_m_s":.2,"angular_rad_s":0.}}})
                self.assertTrue(life.capture_due())
                meta=life.capture_metadata()["factory_story"]
                self.assertEqual(meta["phase"],"material_supply")
                self.assertEqual(meta["physics_frame"],11)
                self.assertEqual(meta["sim_time_ns"],1_100_000_000)
                self.assertFalse(life.capture_due())
                snapshot.write_text("{}",encoding="utf-8")
                life.visuals.applied_sequence=4
                with self.assertRaisesRegex(ValueError,"bytes differ"):life.step(1.1,11)


if __name__ == "__main__":unittest.main()
