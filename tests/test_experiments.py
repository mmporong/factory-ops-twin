from __future__ import annotations

from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from factory_ops.experiments import (
    FIXTURE_BACKEND,
    assert_execution_ready,
    build_matrix,
    build_workload,
    canonical_hash,
    load_experiment_config,
    main,
    plan_manifest,
    released_orders,
    summarize,
    validate_settings,
    validate_workload,
    verify_plan_manifest,
)


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs" / "factory-v3" / "g3-experiment.json"
NON_EVIDENCE_SHA = "1" * 64
NON_EVIDENCE_COMMIT = "2" * 40


def fixture_result(
    trial: dict,
    *,
    status: str = "completed",
    completed_delta: int = 0,
    policy_speedup_s: float = 0.0,
) -> dict:
    """Build an explicitly labeled non-evidence row for aggregation regression tests."""
    total = len(trial["workload"]["orders"])
    completed = total if status == "completed" else max(0, total - 1 + completed_delta)
    mean_cycle = None if completed == 0 else 100.0 - policy_speedup_s
    makespan = 300.0 - policy_speedup_s if status == "completed" else None
    return {
        "schema_version": 1,
        "trial_id": trial["trial_id"],
        "condition": trial["condition"],
        "policy": trial["policy"],
        "seed": trial["seed"],
        "status": status,
        "backend": FIXTURE_BACKEND,
        "source": {
            "commit": NON_EVIDENCE_COMMIT,
            "experiment_config_sha256": trial["source_config_sha256"],
        },
        "input": {
            "trial_input_sha256": trial["trial_input_sha256"],
            "workload_sha256": trial["workload_sha256"],
        },
        "evidence": {
            "classification": "explicit_non_evidence_test_fixture",
            "g3_evidence_id": f"fixture-{trial['trial_id']}",
            "manifest_sha256": NON_EVIDENCE_SHA,
            "artifact_sha256": "3" * 64,
        },
        "metrics": {
            "orders_total": total,
            "orders_completed": completed,
            "horizon_s": trial["workload"]["horizon_s"],
            "mean_completed_cycle_time_s": mean_cycle,
            "makespan_s": makespan,
        },
        "failure_reason": None if status == "completed" else f"fixture_{status}",
    }


class G3ExperimentPlanTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = load_experiment_config(CONFIG)

    def test_matrix_is_deterministic_unique_and_policy_fair(self) -> None:
        first = build_matrix(self.settings)
        second = build_matrix(self.settings)
        self.assertEqual(first, second)
        self.assertEqual(len(first), 60)
        self.assertEqual(len({trial["trial_id"] for trial in first}), 60)
        self.assertEqual(len({trial["trial_input_sha256"] for trial in first}), 60)
        self.assertTrue(all(trial["status"] == "pending_not_executed" for trial in first))
        self.assertTrue(all(trial["execution_gate"] == "calibration_required_not_supplied" for trial in first))
        indexed = {(trial["condition"], trial["policy"], trial["seed"]): trial for trial in first}
        for condition in ("normal", "closure", "burst"):
            condition_rows = [trial for trial in first if trial["condition"] == condition]
            for seed in range(10):
                pair = [trial["policy"] for trial in condition_rows if trial["seed"] == seed]
                self.assertEqual(pair, ["eta", "fifo"] if seed % 2 == 0 else ["fifo", "eta"])
        for condition in ("normal", "closure", "burst"):
            for seed in range(10):
                fifo = indexed[(condition, "fifo", seed)]
                eta = indexed[(condition, "eta", seed)]
                self.assertEqual(fifo["workload_sha256"], eta["workload_sha256"])
                self.assertEqual(fifo["workload"], eta["workload"])
                self.assertNotIn("policy", fifo["workload"])

    def test_seed_changes_sources_and_release_jitter_without_future_dispatch_input(self) -> None:
        normal = [build_workload(seed, "normal", self.settings) for seed in range(10)]
        signatures = {
            (
                tuple(order["source_station"] for order in workload["orders"]),
                tuple(order["ready_time"] for order in workload["orders"]),
            )
            for workload in normal
        }
        self.assertGreater(len(signatures), 1)
        workload = normal[0]
        first_ready = workload["orders"][0]["ready_time"]
        self.assertEqual(released_orders(workload, first_ready - 0.001), [])
        visible = released_orders(workload, first_ready)
        self.assertEqual(len(visible), 1)
        self.assertTrue(all(order["ready_time"] <= first_ready for order in visible))
        self.assertEqual(
            workload["dispatch_visibility"],
            {
                "rule": "release_feed_current_time_only",
                "future_ready_times_exposed": False,
                "ground_truth_exposed": False,
            },
        )
        self.assertNotIn("ground_truth", json.dumps(visible, sort_keys=True))

    def test_burst_concentrates_releases_and_closure_exposes_only_current_boolean(self) -> None:
        burst = build_workload(4, "burst", self.settings)
        times = [order["ready_time"] for order in burst["orders"]]
        self.assertLessEqual(max(times) - min(times), self.settings["workloads"]["pilot"]["burst_window_s"])
        closure = build_workload(4, "closure", self.settings)
        self.assertEqual(closure["condition_data"], {"current_closed": True})
        serialized = json.dumps(closure["condition_data"], sort_keys=True)
        self.assertNotIn("600", serialized)
        self.assertNotIn("end", serialized)
        self.assertEqual(build_workload(4, "normal", self.settings)["condition_data"], {"current_closed": False})

    def test_qualification_is_two_orders_from_s1_and_s3(self) -> None:
        workload = build_workload(0, "normal", self.settings, profile="qualification")
        self.assertEqual(len(workload["orders"]), 2)
        self.assertEqual([order["source_station"] for order in workload["orders"]], ["S1", "S3"])

    def test_source_bindings_match_authored_files(self) -> None:
        for binding in self.settings["source_bindings"]:
            digest = hashlib.sha256((ROOT / binding["path"]).read_bytes()).hexdigest()
            self.assertEqual(digest, binding["sha256"])
        closure = self.settings["conditions"]["closure"]
        self.assertEqual(closure["model"], "physical_barrier_plus_route_disable")
        self.assertEqual(closure["route_graph_effect"], "disable_C_LOW-E_LOW")
        self.assertEqual(closure["barrier_provenance"]["layout_object_id"], "blocked_aisle")

    def test_canonical_hash_and_validation_reject_nonfinite_negative_and_duplicates(self) -> None:
        with self.assertRaises(ValueError):
            canonical_hash({"bad": float("nan")})
        invalid = deepcopy(self.settings)
        invalid["load"]["nominal_speed_m_s"] = -0.1
        with self.assertRaises(ValueError):
            validate_settings(invalid)
        invalid_scope = deepcopy(self.settings)
        invalid_scope["scope"] = "actual physical improvement proven"
        with self.assertRaisesRegex(ValueError, "G3a pending inputs"):
            validate_settings(invalid_scope)
        workload = build_workload(0, "normal", self.settings)
        workload["orders"][0]["ready_time"] = float("inf")
        with self.assertRaises(ValueError):
            validate_workload(workload, self.settings)
        duplicate = build_workload(0, "normal", self.settings)
        duplicate["orders"][1]["order_id"] = duplicate["orders"][0]["order_id"]
        with self.assertRaises(ValueError):
            validate_workload(duplicate, self.settings)

    def test_pending_manifest_materializes_and_verifies_but_is_not_evidence(self) -> None:
        manifest = plan_manifest(self.settings)
        self.assertEqual(manifest["trial_count"], 60)
        self.assertIn("not_physical_evidence", manifest["classification"])
        verify_plan_manifest(manifest, self.settings)
        changed = deepcopy(manifest)
        changed["trials"][0]["seed"] = 99
        with self.assertRaises(ValueError):
            verify_plan_manifest(changed, self.settings)
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "manifest.json"
            self.assertEqual(main([str(CONFIG), "--output", str(output)]), 0)
            self.assertEqual(main([str(CONFIG), "--verify", str(output)]), 0)

    def test_full_matrix_execution_is_gated_until_g3b_calibration_is_bound(self) -> None:
        self.assertEqual(self.settings["calibration"]["status"], "required_not_supplied")
        self.assertEqual(self.settings["scope"], "g3a_pending_inputs_not_physical_evidence")
        with self.assertRaisesRegex(ValueError, "actual Isaac Sim/ROS 2/Nav2 calibration artifact"):
            assert_execution_ready(self.settings)


class G3ExperimentSummaryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.settings = load_experiment_config(CONFIG)
        self.matrix = build_matrix(self.settings)
        self.results = [
            fixture_result(
                trial,
                policy_speedup_s=10.0 if trial["policy"] == "eta" else 0.0,
            )
            for trial in self.matrix
        ]

    def test_explicit_non_evidence_summary_preserves_sixty_trial_denominator(self) -> None:
        summary = summarize(self.results, self.settings, allow_explicit_non_evidence=True)
        self.assertEqual(summary["trial_count"], 60)
        self.assertEqual(summary["classification"], "explicit_non_evidence_test_fixture")
        self.assertEqual(summary["denominator_policy"], "all_planned_trials_including_failed_and_incomplete")
        for condition in ("normal", "closure", "burst"):
            paired = summary["paired_comparison"][condition]
            self.assertEqual(paired["primary_pair_count"], 10)
            self.assertEqual(paired["restricted_full_completion_pair_count"], 10)
            self.assertEqual(paired["restricted_mean_eta_minus_fifo_makespan_s"], -10.0)

    def test_cpu_or_fixture_rows_cannot_be_relabelled_as_physical_evidence(self) -> None:
        with self.assertRaisesRegex(ValueError, "calibration artifact"):
            summarize(self.results, self.settings)
        relabelled = deepcopy(self.results)
        for row in relabelled:
            row["backend"] = "isaac_sim_ros2_nav2"
            row["evidence"]["classification"] = "g3_physical_trial"
        with self.assertRaisesRegex(ValueError, "calibration artifact"):
            summarize(relabelled, self.settings)
        cpu = deepcopy(self.results)
        cpu[0]["backend"] = "cpu_smoke"
        with self.assertRaises(ValueError):
            summarize(cpu, self.settings, allow_explicit_non_evidence=True)

    def test_missing_failed_result_deletion_and_duplicate_are_rejected(self) -> None:
        failed = deepcopy(self.results)
        failed[0] = fixture_result(self.matrix[0], status="failed")
        with self.assertRaisesRegex(ValueError, "all 60"):
            summarize(failed[1:], self.settings, allow_explicit_non_evidence=True)
        duplicate = deepcopy(failed)
        duplicate[-1] = deepcopy(duplicate[0])
        duplicate[-1]["evidence"]["g3_evidence_id"] = "fixture-duplicate"
        with self.assertRaises(ValueError):
            summarize(duplicate, self.settings, allow_explicit_non_evidence=True)

    def test_foreign_seed_hash_and_source_are_rejected(self) -> None:
        foreign_seed = deepcopy(self.results)
        foreign_seed[0]["seed"] = 10
        with self.assertRaises(ValueError):
            summarize(foreign_seed, self.settings, allow_explicit_non_evidence=True)
        foreign_hash = deepcopy(self.results)
        foreign_hash[0]["input"]["workload_sha256"] = "f" * 64
        with self.assertRaises(ValueError):
            summarize(foreign_hash, self.settings, allow_explicit_non_evidence=True)
        foreign_source = deepcopy(self.results)
        foreign_source[0]["source"]["experiment_config_sha256"] = "e" * 64
        with self.assertRaises(ValueError):
            summarize(foreign_source, self.settings, allow_explicit_non_evidence=True)
        mixed_commits = deepcopy(self.results)
        mixed_commits[0]["source"]["commit"] = "4" * 40
        with self.assertRaisesRegex(ValueError, "one measurement source commit"):
            summarize(mixed_commits, self.settings, allow_explicit_non_evidence=True)

    def test_paired_counterexamples_report_failure_incomplete_and_restricted_lead(self) -> None:
        results = deepcopy(self.results)
        index = {
            (trial["condition"], trial["policy"], trial["seed"]): position
            for position, trial in enumerate(self.matrix)
        }
        fifo_failed = index[("normal", "fifo", 0)]
        eta_incomplete = index[("normal", "eta", 1)]
        results[fifo_failed] = fixture_result(self.matrix[fifo_failed], status="failed")
        results[eta_incomplete] = fixture_result(self.matrix[eta_incomplete], status="incomplete")
        summary = summarize(results, self.settings, allow_explicit_non_evidence=True)
        normal = summary["paired_comparison"]["normal"]
        self.assertEqual(normal["primary_pair_count"], 10)
        self.assertEqual(normal["primary_excluded_pair_count"], 0)
        self.assertEqual(normal["primary_pairs_with_failure"], 1)
        self.assertEqual(normal["primary_pairs_with_incomplete"], 1)
        self.assertTrue(normal["pairs"][0]["primary_denominator_included"])
        self.assertTrue(normal["pairs"][0]["failure_present"])
        self.assertTrue(normal["pairs"][1]["primary_denominator_included"])
        self.assertFalse(normal["pairs"][1]["both_full_workloads_completed"])
        self.assertIsNone(normal["pairs"][1]["eta_minus_fifo_makespan_s"])
        group = summary["groups"]["normal"]["eta"]
        self.assertEqual(group["trial_counts"]["incomplete_at_horizon"], 1)
        self.assertGreater(group["horizon_order_counts"]["censored"], 0)
        self.assertEqual(group["completed_orders_only_label"], "censoring_biased_not_a_full_workload_lead")

    def test_failed_eta_remains_in_primary_denominator_instead_of_biasing_survivors(self) -> None:
        results = [fixture_result(trial) for trial in self.matrix]
        index = {
            (trial["condition"], trial["policy"], trial["seed"]): position
            for position, trial in enumerate(self.matrix)
        }
        failed_eta = index[("normal", "eta", 0)]
        results[failed_eta] = fixture_result(
            self.matrix[failed_eta], status="failed", completed_delta=-99
        )
        summary = summarize(results, self.settings, allow_explicit_non_evidence=True)
        normal = summary["paired_comparison"]["normal"]
        self.assertEqual(normal["primary_pair_count"], 10)
        self.assertEqual(normal["primary_pairs_with_failure"], 1)
        self.assertLess(normal["primary_mean_delta"], 0.0)
        self.assertEqual(normal["restricted_full_completion_pair_count"], 9)


if __name__ == "__main__":
    unittest.main()
