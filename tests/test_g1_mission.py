from __future__ import annotations

from types import SimpleNamespace
import unittest

from scripts.ros.g1_mission import LifecycleStatePoller


class _Future:
    def __init__(self, state: int, *, done: bool) -> None:
        self.state = state
        self.complete = done

    def done(self) -> bool:
        return self.complete

    def result(self) -> object:
        return SimpleNamespace(current_state=SimpleNamespace(id=self.state))


class _Client:
    def __init__(self, futures: list[_Future]) -> None:
        self.futures = futures
        self.requests: list[object] = []

    @staticmethod
    def service_is_ready() -> bool:
        return True

    def call_async(self, request: object) -> _Future:
        self.requests.append(request)
        return self.futures.pop(0)


class LifecycleStatePollerTests(unittest.TestCase):
    def test_immediate_responses_remain_bounded_to_two_requests_per_second(self) -> None:
        client = _Client([_Future(1, done=True), _Future(1, done=True), _Future(1, done=True)])
        poller = LifecycleStatePoller(["controller_server"])

        for index in range(100):
            poller.poll(index / 100.0, {"controller_server": client}, object)

        self.assertEqual(len(client.requests), 2)

    def test_pending_future_is_not_resent_after_interval(self) -> None:
        pending = _Future(1, done=False)
        client = _Client([pending])
        poller = LifecycleStatePoller(["controller_server"])

        poller.poll(0.0, {"controller_server": client}, object)
        poller.poll(5.0, {"controller_server": client}, object)

        self.assertEqual(len(client.requests), 1)
        self.assertIsNone(poller.states["controller_server"])

    def test_active_state_is_cached_without_further_requests(self) -> None:
        client = _Client([_Future(3, done=True)])
        poller = LifecycleStatePoller(["controller_server"])

        poller.poll(0.0, {"controller_server": client}, object)
        states = poller.poll(0.01, {"controller_server": client}, object)
        poller.poll(10.0, {"controller_server": client}, object)

        self.assertEqual(states, {"controller_server": 3})
        self.assertEqual(len(client.requests), 1)


if __name__ == "__main__":
    unittest.main()
