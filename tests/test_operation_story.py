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
        return {"run_id":"r","sequence":3,"projection":{"payloads":[{"state":state,"robot_id":"carter1"}],
            "signals":{"orders_completed":shipped,"cell_processing":processing}}}

    def material(self, state="empty", complete=False):
        return {"run_id":"r","material_revision":2,"empty_totes":[{"state":state,"robot_id":"carter1"}],
                "source_snapshot":{"completed":complete}}

    def test_no_first_motion_coupon_and_exact_bounded_source_frames(self):
        s=OperationStory(PROFILE,"r",3)
        s.observe(1,10,self.state("source_waiting"),None,moving_robot_ids={"carter1"})
        self.assertIsNone(s.due())
        s.observe(2,20,self.state(),None,moving_robot_ids=set())
        self.assertIsNone(s.due())
        for frame in range(3,243):
            s.observe(frame,frame*100,self.state(),None,moving_robot_ids={"carter1"})
            sample=s.due()
            self.assertEqual(sample["sample_index"],frame-3)
            self.assertIsNone(s.due())
        s.observe(243,24300,self.state(),None,moving_robot_ids={"carter1"})
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
            s.observe(frame,frame*100,v,m,moving_robot_ids={"carter1","carter2"})
            self.assertEqual(s.due()["phase"],phase)
            frame+=240
        s.observe(frame,frame*100,self.state("complete",shipped=3),self.material("available"),moving_robot_ids=set())
        self.assertIsNone(s.due())
        s.observe(frame+1,(frame+1)*100,self.state("complete",shipped=3),self.material("available",True),moving_robot_ids=set())
        self.assertEqual(s.due()["phase"],"closed_loop_complete")
        s.close()
        self.assertEqual(s.events[-1]["event"],"window_terminal_ended")

    def test_three_second_windows_keep_all_six_phases_at_1080_source_frames(self):
        s=OperationStory({**PROFILE,"window_frames":180},"r",3)
        cases=[(self.state(),self.material()),
               (self.state("processing",processing=True),self.material()),
               (self.state("outbound_loaded"),self.material()),
               (self.state("complete",shipped=1),self.material()),
               (self.state("complete",shipped=2),self.material("returning")),
               (self.state("complete",shipped=3),self.material("available",True))]
        for phase,(visual,material) in enumerate(cases):
            for offset in range(180):
                frame=phase*180+offset
                s.observe(frame,frame*100,visual,material,moving_robot_ids={"carter1","carter2"})
                self.assertEqual(s.due()["sample_index"],offset)
        s.observe(1080,108000,cases[-1][0],cases[-1][1],moving_robot_ids=set())
        self.assertIsNone(s.due())
        self.assertEqual((len(s.seen),s.emitted),(6,1080))
        with self.assertRaises(ValueError):
            OperationStory({**PROFILE,"window_frames":360},"r",3)

    def test_run_mismatch_and_broadened_budget_rejected(self):
        with self.assertRaises(ValueError):OperationStory({**PROFILE,"maximum_windows":20},"r",3)
        s=OperationStory(PROFILE,"r",3);v=self.state();v["run_id"]="other"
        with self.assertRaises(ValueError):s.observe(1,10,v,None,moving_robot_ids={"carter1"})

    def test_other_robot_motion_cannot_trigger_loaded_or_empty_transport(self):
        for state,material in (("inbound_loaded",None),("outbound_loaded",None),
                               ("complete",self.material("returning"))):
            with self.subTest(state=state):
                s=OperationStory(PROFILE,"r",3)
                s.observe(1,10,self.state(state),material,moving_robot_ids={"carter2"})
                self.assertIsNone(s.due())
                s.observe(2,20,self.state(state),material,moving_robot_ids={"carter1"})
                self.assertIsNotNone(s.due())

    def test_after_shipment_waits_for_two_robots_without_adding_a_window(self):
        s=OperationStory(PROFILE,"r",3)
        s.observe(1,10,self.state("complete",shipped=1),None,moving_robot_ids={"carter1"})
        self.assertIsNone(s.due())
        s.observe(2,20,self.state("complete",shipped=1),None,moving_robot_ids={"carter1","carter2"})
        self.assertEqual(s.due()["phase"],"first_shipment")
        self.assertEqual(s.seen,{"first_shipment"})

    def test_requested_command_without_measured_motion_does_not_arm_capture(self):
        from scripts.isaac.local45_fleet4_transport_scene import Local45FleetLifecycle
        life=object.__new__(Local45FleetLifecycle)
        life.operation_story=True;life.story_motion={}
        sample={"physics_frame":10,"sim_time_s":1.,"authorization":{"mode":"navigate","valid":True},
                "motion_profile":{"input_command":{"linear_m_s":.2}},"odometry":{"linear_m_s":0.}}
        with patch("scripts.isaac.local45_fleet4_transport_scene.FleetTransportLifecycle.after_control"):
            life.after_control("carter1",sample)
            self.assertFalse(life.story_motion["carter1"][1])
            sample["odometry"]["linear_m_s"]=.2
            life.after_control("carter1",sample)
            self.assertTrue(life.story_motion["carter1"][1])

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
                    "motion_profile":{"input_command":{"linear_m_s":.2,"angular_rad_s":0.}},
                    "odometry":{"linear_m_s":.2,"angular_rad_s":0.}})
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
