"""Own FlatScan output at its native execution event; cold import needs no SDK."""
from __future__ import annotations

from copy import deepcopy
from collections.abc import Callable, Mapping
import threading
from types import MappingProxyType
from typing import Any
from uuid import uuid4

from .flat_scan_ros import LOCAL_COMPLETION_SEQUENCE_KEY


FIELDS = ('timeStamp', 'linearDepthData', 'intensitiesData', 'azimuthRange', 'depthRange',
          'horizontalFov', 'horizontalResolution', 'numCols', 'numRows', 'rotationRate')
_OWNED: dict[str, 'NativeFlatScanCompletion'] = {}


def capture_complete_scan(token: str, payload: Mapping[str, Any]) -> bool:
    """Called only by the connected native execution sink, never by a poller."""
    owner = _OWNED.get(token)
    if owner is None:
        return False
    try:
        # Copy while the connected node is executing. This owns returned bytes;
        # it does not assert an atomic implementation inside the SDK producer.
        snapshot = deepcopy({field: payload[field] for field in FIELDS})
        context = dict(owner._context()) if owner._context is not None else None
        with owner._lock:
            sequence = owner.callback_sequence + 1
            snapshot[LOCAL_COMPLETION_SEQUENCE_KEY] = sequence
            owner._snapshot = MappingProxyType(snapshot)
            owner.callback_sequence = sequence
            owner.last_callback_context = context
        return True
    except Exception as error:
        with owner._lock:
            owner._error = type(error).__name__ + ': ' + str(error)
        return False


class NativeFlatScanCompletion:
    """Connect one owned ScriptNode to an already attached native annotator.

    The consumer receives the latest owned completed-event payload. Re-reading
    that payload is not a new sensor acquisition and does not refresh its stamp.
    Native timestamp monotonicity, conversion and admission remain the caller's
    existing policy. callback_sequence is a local count, not a hardware frame.
    """

    def __init__(self, annotator: Any, *, context: Callable[[], Mapping[str, Any]] | None = None) -> None:
        import omni.graph.core as og
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension('omni.graph.scriptnode')
        self._og, self._node = og, None
        self._token = uuid4().hex
        self._lock = threading.RLock()
        self._context = context
        self._snapshot: Mapping[str, Any] | None = None
        self._error: str | None = None
        self.callback_sequence = 0
        self.last_callback_context: dict[str, Any] | None = None
        source = annotator.get_node()
        graph = source.get_graph()
        self._graph = graph
        _OWNED[self._token] = self
        try:
            self._node = og.Controller.create_node(
                ('FactoryFullScan_' + self._token, graph), 'omni.graph.scriptnode.ScriptNode')
            for field in FIELDS:
                upstream = source.get_attribute('outputs:' + field)
                if not upstream.is_valid():
                    raise RuntimeError('native FlatScan output missing: ' + field)
                if not self._node.create_attribute('inputs:' + field, upstream.get_resolved_type(),
                        og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT):
                    raise RuntimeError('native FlatScan input creation failed: ' + field)
                downstream = self._node.get_attribute('inputs:' + field)
                if not upstream.connect(downstream, True):
                    raise RuntimeError('native FlatScan input connection failed: ' + field)
            script = ('def compute(db):\n'
                      '    from factory_ops.flat_scan_completion import capture_complete_scan\n'
                      '    return capture_complete_scan(' + repr(self._token) + ', {\n'
                      + ''.join('        ' + repr(field) + ': db.inputs.' + field + ',\n' for field in FIELDS)
                      + '    })\n')
            og.Controller.set(self._node.get_attribute('inputs:script'), script)
            execution = source.get_attribute('outputs:exec')
            if (not execution.is_valid()
                    or execution.get_resolved_type().get_role_name() != 'execution'
                    or not execution.connect(self._node.get_attribute('inputs:execIn'), True)):
                raise RuntimeError('native FlatScan execution connection failed')
        except BaseException:
            self.close()
            raise

    def read(self) -> Mapping[str, Any]:
        with self._lock:
            if self._token not in _OWNED:
                raise RuntimeError('native completion sink is closed')
            if self._error is not None:
                raise RuntimeError('native completion callback failed: ' + self._error)
            return self._snapshot if self._snapshot is not None else {}

    def close(self) -> None:
        if self._node is not None:
            self._og.Controller.edit(self._graph, {self._og.Controller.Keys.DELETE_NODES: [self._node]})
            self._node = None
        _OWNED.pop(self._token, None)
        with self._lock:
            self._snapshot = None
            self._context = None
