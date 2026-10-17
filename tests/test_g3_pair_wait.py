"""Host lifecycle helper tests; no Docker or GPU execution."""
from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from typing import Callable

from scripts.brev.g3_pair_wait import ContainerState, validate_pair, validate_record_path, wait_for_pair


NAV = "factory-ops-g3-q-normal-fifo-000-nav"
SIM = "factory-ops-g3-q-normal-fifo-000-sim"


class FakeRuntime:
    def __init__(self, nav: ContainerState, sim: ContainerState) -> None:
        self.states = {NAV: nav, SIM: sim}
        self.time = 0.0
        self.signals: list[str] = []
        self.stops: list[tuple[str, int]] = []
        self.on_sleep: Callable[[FakeRuntime], None] | None = None

    def inspect(self, name: str) -> ContainerState:
        return self.states[name]

    def signal_int(self, name: str) -> None:
        self.signals.append(name)

    def stop(self, name: str, timeout_s: int) -> None:
        self.stops.append((name, timeout_s))

    def monotonic(self) -> float:
        return self.time

    def utc_now(self) -> str:
        return f"2026-01-01T00:00:{int(self.time):02d}+00:00"

    def sleep(self, seconds: float) -> None:
        self.time += seconds
        if self.on_sleep is not None:
            self.on_sleep(self)

    def run(self, **overrides):
        values = {"poll_s": 1.0, "sim_success_grace_s": 4.0,
                  "interrupt_grace_s": 3.0, "stop_timeout_s": 2}
        values.update(overrides)
        return wait_for_pair(
            NAV, SIM, inspect=self.inspect, signal_int=self.signal_int, stop=self.stop,
            monotonic=self.monotonic, utc_now=self.utc_now, sleep=self.sleep, **values,
        )


class G3PairWaitTests(unittest.TestCase):
    def test_normal_nav_exit_returns_without_touching_sim(self) -> None:
        runtime = FakeRuntime(ContainerState(False, 0), ContainerState(True, None))
        result = runtime.run()
        self.assertEqual("nav_exited", result["status"])
        self.assertEqual(0, result["nav_exit_code"])
        self.assertEqual([], runtime.signals)
        self.assertEqual([], runtime.stops)

    def test_nav_failure_first_returns_for_bash_exit_handling(self) -> None:
        runtime = FakeRuntime(ContainerState(False, 7), ContainerState(True, None))
        result = runtime.run()
        self.assertEqual(7, result["nav_exit_code"])
        self.assertIsNone(result["abort_reason"])
        self.assertEqual([], runtime.signals)
        self.assertEqual([], runtime.stops)

    def test_sim_failure_requests_immediate_nav_interrupt(self) -> None:
        runtime = FakeRuntime(ContainerState(True, None), ContainerState(False, 9))

        def exit_after_signal(fake: FakeRuntime) -> None:
            if fake.signals:
                fake.states[NAV] = ContainerState(False, 130)

        runtime.on_sleep = exit_after_signal
        result = runtime.run()
        self.assertEqual([NAV], runtime.signals)
        self.assertEqual([], runtime.stops)
        self.assertEqual("sim_nonzero_before_nav_exit", result["abort_reason"])

    def test_sim_zero_allows_closing_grace_before_interrupt(self) -> None:
        runtime = FakeRuntime(ContainerState(True, None), ContainerState(False, 0))

        def exit_after_signal(fake: FakeRuntime) -> None:
            if fake.signals:
                fake.states[NAV] = ContainerState(False, 130)

        runtime.on_sleep = exit_after_signal
        result = runtime.run(sim_success_grace_s=4.0)
        self.assertGreaterEqual(runtime.time, 5.0)
        self.assertEqual([NAV], runtime.signals)
        self.assertEqual("sim_zero_nav_grace_expired", result["abort_reason"])

    def test_nav_can_close_normally_during_sim_zero_grace(self) -> None:
        runtime = FakeRuntime(ContainerState(True, None), ContainerState(False, 0))

        def exit_during_grace(fake: FakeRuntime) -> None:
            if fake.time >= 2.0:
                fake.states[NAV] = ContainerState(False, 0)

        runtime.on_sleep = exit_during_grace
        result = runtime.run(sim_success_grace_s=4.0)
        self.assertEqual([], runtime.signals)
        self.assertEqual([], runtime.stops)
        self.assertIsNone(result["abort_reason"])

    def test_hung_nav_gets_bounded_stop_fallback(self) -> None:
        runtime = FakeRuntime(ContainerState(True, None), ContainerState(False, 5))

        def stop_and_exit(name: str, timeout_s: int) -> None:
            runtime.stops.append((name, timeout_s))
            runtime.states[NAV] = ContainerState(False, 137)

        runtime.stop = stop_and_exit  # type: ignore[method-assign]
        result = runtime.run(interrupt_grace_s=3.0, stop_timeout_s=2)
        self.assertEqual([NAV], runtime.signals)
        self.assertEqual([(NAV, 2)], runtime.stops)
        self.assertTrue(result["stop_requested"])

    def test_invalid_scope_and_reused_record_are_rejected(self) -> None:
        for nav, sim in (("unowned-nav", SIM),
                         (NAV, "factory-ops-g3-q-normal-fifo-001-sim"),
                         (NAV.replace("-nav", "-sim"), SIM)):
            with self.subTest(nav=nav, sim=sim), self.assertRaises(ValueError):
                validate_pair(nav, sim)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder).resolve()
            record = root / "lifecycle.json"
            self.assertEqual(record, validate_record_path(record))
            record.write_text("{}\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "new path"):
                validate_record_path(record)
            with self.assertRaises(ValueError):
                validate_record_path(Path("relative.json"))


if __name__ == "__main__":
    unittest.main()
