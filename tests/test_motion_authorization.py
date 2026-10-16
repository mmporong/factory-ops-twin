from copy import deepcopy
import unittest

from factory_ops.motion_authorization import AuthorizationError, LeaseGuard


def permission(sequence=1, mode="navigate", issued=1.0):
    return {"schema_version": 1, "run_id": "run", "robot_id": "carter1",
            "authorization_seq": sequence, "issued_sim_time_s": issued,
            "valid_until_sim_time_s": issued + 0.5, "mode": mode,
            "scope": {"scenario_id": "F1", "route_id": "cross",
                      "goal_uuid": "1c4cf21c362e43df901ae945dc48e663", "reservation_id": 1,
                      "resources": ["Z_UPPER"], "exit_id": "Z_UPPER:east"}}


class LeaseTests(unittest.TestCase):
    def test_permission_is_idempotent_but_cannot_extend_expiry(self):
        guard = LeaseGuard("run", "carter1")
        value = permission()
        self.assertEqual(guard.evaluate(value, now_s=1).mode, "navigate")
        self.assertTrue(guard.evaluate(value, now_s=1.4).valid)
        self.assertEqual(guard.evaluate(value, now_s=1.5).reason, "authorization_expired")
        value["valid_until_sim_time_s"] = 1.6
        with self.assertRaises(AuthorizationError):
            guard.evaluate(value, now_s=1.6)

    def test_missing_and_expired_do_not_release_any_ownership(self):
        guard = LeaseGuard("run", "carter1")
        self.assertFalse(guard.evaluate(None, now_s=0).valid)
        state = guard.evaluate(permission(mode="hold"), now_s=1.1)
        self.assertEqual(state.mode, "hold")
        assert state.payload is not None
        self.assertEqual(state.payload["scope"]["reservation_id"], 1)
        self.assertEqual(guard.evaluate(permission(mode="hold"), now_s=1.6).mode, "emergency")

    def test_identity_replay_future_and_invalid_bundles_fail(self):
        cases = [("run_id", "other"), ("robot_id", "carter2"), ("authorization_seq", True),
                 ("issued_sim_time_s", 2), ("valid_until_sim_time_s", float("nan")),
                 ("valid_until_sim_time_s", 1.51), ("mode", "go")]
        cases.append(("issued_sim_time_s", 10 ** 400))
        for key, value in cases:
            with self.subTest(key=key, value=value):
                data = permission()
                data[key] = value
                with self.assertRaises(AuthorizationError):
                    LeaseGuard("run", "carter1").evaluate(data, now_s=1)
        for key, value in (("goal_uuid", "fake"), ("reservation_id", None),
                           ("resources", []), ("exit_id", "J_SHIP:east")):
            data = permission()
            data["scope"][key] = value
            with self.assertRaises(AuthorizationError):
                LeaseGuard("run", "carter1").evaluate(data, now_s=1)
        guard = LeaseGuard("run", "carter1")
        guard.evaluate(permission(2), now_s=1)
        with self.assertRaises(AuthorizationError):
            guard.evaluate(permission(1), now_s=1)

    def test_reused_sequence_cannot_change_mode_and_copy_cannot_mutate_guard(self):
        guard = LeaseGuard("run", "carter1")
        value = permission()
        state = guard.evaluate(value, now_s=1)
        assert state.payload is not None
        value["scope"]["route_id"] = "mutated"
        self.assertEqual(state.payload["scope"]["route_id"], "cross")
        changed = deepcopy(permission())
        changed["mode"] = "hold"
        with self.assertRaises(AuthorizationError):
            guard.evaluate(changed, now_s=1.1)


if __name__ == "__main__":
    unittest.main()
