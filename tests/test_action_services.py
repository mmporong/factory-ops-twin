"""Public action RPC request/response tests; no ROS transport is synthesized."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
import unittest

from scripts.ros.action_services import IntrospectedActionClient


class Service:
    @staticmethod
    def Request():
        return NS(goal_id=None, goal=None, goal_info=NS(goal_id=None))


class Client:
    def __init__(self):
        self.configured = None
        self.calls = []
        self.futures = []
        self.ready = True

    def configure_introspection(self, *args):
        self.configured = args

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        future = Future(); self.futures.append(future)
        return future


class Node:
    def __init__(self):
        self.clients = {}

    def create_client(self, service_type, name, *, qos_profile):
        client = Client(); self.clients[name] = client
        return client

    def get_clock(self):
        return "clock"

    def get_namespace(self):
        return "/carter1"


class ActionServiceTests(unittest.TestCase):
    def setUp(self):
        self.node = Node()
        action = NS(Impl=NS(SendGoalService=Service, GetResultService=Service))
        self.client = IntrospectedActionClient(self.node, action, "follow_path", cancel_type=Service, event_qos="qos", introspection_state="CONTENTS")
        self.uuid = NS(uuid=list(range(1, 17)))
        self.goal = NS(path=NS(points=[(4., 4.), (12., 4.)]), controller_id="FollowPath")

    def accept(self, accepted=True):
        future = self.client.send_goal_async(self.goal, goal_uuid=self.uuid)
        self.client.send_client.futures[-1].set_result(NS(accepted=accepted, stamp=NS(sec=12, nanosec=1)))
        return future.result()

    def test_real_call_owner_and_introspection_topics_are_explicit(self):
        self.assertTrue(self.client.server_is_ready())
        self.assertEqual(3, len(self.node.clients))
        self.assertTrue(all(c.configured == ("clock", "qos", "CONTENTS") for c in self.node.clients.values()))
        self.assertEqual(("/carter1/follow_path/_action/send_goal/_service_event", "/carter1/follow_path/_action/get_result/_service_event", "/carter1/follow_path/_action/cancel_goal/_service_event"), self.client.service_event_topics)
        self.client.result_client.ready = False
        self.assertFalse(self.client.server_is_ready())

    def test_send_request_is_deep_copied_and_result_cancel_keep_uuid(self):
        handle = self.accept()
        request = self.client.send_client.calls[0]
        self.goal.path.points.append((99., 99.)); self.uuid.uuid[0] = 99
        self.assertEqual([(4., 4.), (12., 4.)], request.goal.path.points)
        self.assertEqual(list(range(1, 17)), request.goal_id.uuid)
        result = handle.get_result_async()
        self.assertIs(result, handle.get_result_async())
        handle.cancel_goal_async()
        self.assertEqual(request.goal_id.uuid, self.client.result_client.calls[0].goal_id.uuid)
        self.assertEqual(request.goal_id.uuid, self.client.cancel_client.calls[0].goal_info.goal_id.uuid)
        result.set_result(NS(status=4, result=NS(error_code=0)))
        self.assertIsNone(self.client.active_uuid)

    def test_rejected_goal_does_not_request_a_result(self):
        handle = self.accept(False)
        self.assertFalse(handle.accepted)
        with self.assertRaises(RuntimeError):
            handle.get_result_async()
        self.assertEqual([], self.client.result_client.calls)

    def test_uuid_is_nonzero_unique_and_preemption_is_forbidden(self):
        for value in ([], [0]*16, [1]*15):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.client.send_goal_async(self.goal, goal_uuid=NS(uuid=value))
        handle = self.accept()
        with self.assertRaises(RuntimeError):
            self.client.send_goal_async(self.goal, goal_uuid=NS(uuid=[2]*16))
        handle.get_result_async().set_result(NS(status=4))
        with self.assertRaises(ValueError):
            self.client.send_goal_async(self.goal, goal_uuid=self.uuid)

    def test_send_transport_error_propagates_and_remains_fail_closed(self):
        future = self.client.send_goal_async(self.goal, goal_uuid=self.uuid)
        self.client.send_client.futures[-1].set_exception(RuntimeError("DDS unavailable"))
        with self.assertRaisesRegex(RuntimeError, "DDS unavailable"):
            future.result()
        self.assertIsNotNone(self.client.active_uuid)


if __name__ == "__main__":
    unittest.main()
