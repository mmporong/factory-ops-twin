"""Public typed ROS action RPCs with ordinary-client service introspection.

Jazzy ActionClient lacks introspection. These clients own the real SendGoal,
GetResult and CancelGoal calls instead of observing or imitating another client.
"""
from __future__ import annotations

from concurrent.futures import Future
from copy import deepcopy
from typing import Any


class ServiceGoalHandle:
    def __init__(self, owner: "IntrospectedActionClient", goal_id: Any, response: Any) -> None:
        self._owner = owner
        self.goal_id = deepcopy(goal_id)
        self.accepted = bool(response.accepted)
        self.accepted_stamp = deepcopy(response.stamp)
        self._result_future: Any = None

    def get_result_async(self) -> Any:
        if not self.accepted:
            raise RuntimeError("a rejected goal has no requested execution result")
        if self._result_future is not None:
            return self._result_future
        request = self._owner.result_type.Request()
        request.goal_id = deepcopy(self.goal_id)
        self._result_future = self._owner.result_client.call_async(request)

        def terminal(future: Any) -> None:
            if not future.cancelled() and future.exception() is None and future.result() is not None and future.result().status in (4, 5, 6):
                self._owner.active_uuid = None

        self._result_future.add_done_callback(terminal)
        return self._result_future

    def cancel_goal_async(self) -> Any:
        request = self._owner.cancel_type.Request()
        request.goal_info.goal_id = deepcopy(self.goal_id)
        return self._owner.cancel_client.call_async(request)


class IntrospectedActionClient:
    """Minimal non-preempting action client over public generated service types."""

    def __init__(self, node: Any, action_type: Any, action_name: str, *, cancel_type: Any,
                 event_qos: Any, introspection_state: Any) -> None:
        self.send_type = action_type.Impl.SendGoalService
        self.result_type = action_type.Impl.GetResultService
        self.cancel_type = cancel_type
        self.send_client = node.create_client(self.send_type, f"{action_name}/_action/send_goal", qos_profile=event_qos)
        self.result_client = node.create_client(self.result_type, f"{action_name}/_action/get_result", qos_profile=event_qos)
        self.cancel_client = node.create_client(cancel_type, f"{action_name}/_action/cancel_goal", qos_profile=event_qos)
        for client in (self.send_client, self.result_client, self.cancel_client):
            client.configure_introspection(node.get_clock(), event_qos, introspection_state)
        namespace = node.get_namespace().rstrip("/")
        self.service_event_topics = tuple(f"{namespace}/{action_name}/_action/{name}/_service_event" for name in ("send_goal", "get_result", "cancel_goal"))
        self._issued: set[bytes] = set()
        self.active_uuid: bytes | None = None

    def server_is_ready(self) -> bool:
        return all(client.service_is_ready() for client in (self.send_client, self.result_client, self.cancel_client))

    def send_goal_async(self, goal: Any, *, goal_uuid: Any) -> Future:
        identity = bytes(goal_uuid.uuid)
        if self.active_uuid is not None:
            raise RuntimeError("an action is already pending or active; path preemption is disabled")
        if len(identity) != 16 or identity == bytes(16) or identity in self._issued:
            raise ValueError("action request requires a unique nonzero 16-byte goal UUID")
        self._issued.add(identity)
        self.active_uuid = identity
        request = self.send_type.Request()
        request.goal_id = deepcopy(goal_uuid)
        request.goal = deepcopy(goal)
        output: Future = Future()
        transport = self.send_client.call_async(request)

        def completed(future: Any) -> None:
            if output.cancelled():
                return
            try:
                response = future.result()
                if response is None:
                    raise RuntimeError("SendGoal returned no service response")
                if not response.accepted:
                    self.active_uuid = None
                output.set_result(ServiceGoalHandle(self, request.goal_id, response))
            except BaseException as error:
                output.set_exception(error)

        transport.add_done_callback(completed)
        return output
