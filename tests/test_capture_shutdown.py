from types import SimpleNamespace
import unittest

from scripts.isaac.robot_qualification import CaptureSession
from scripts.isaac.stage_smoke import SmokeError


class CaptureShutdownTests(unittest.TestCase):
    def session(self, *, opt_in=True, stop_error=False, drain_error=False):
        events = []
        state = {'stopped': not opt_in}

        def stop():
            events.append('stop')
            if stop_error:
                raise RuntimeError('stop rejected')
            state['stopped'] = True

        def drain():
            events.append('drain')
            if not state['stopped'] or drain_error:
                raise RuntimeError('generation not stopped or writer failed')

        session = CaptureSession.__new__(CaptureSession)
        session.closed = False
        session.stop_before_drain = opt_in
        session.rep = SimpleNamespace(orchestrator=SimpleNamespace(stop=stop, wait_until_complete=drain))
        session.annotators = [SimpleNamespace(detach=lambda: events.append('detach'))]
        session.products = [SimpleNamespace(destroy=lambda: events.append('destroy'))]
        return session, events

    def test_paused_generation_stops_before_writer_drain_and_is_idempotent(self):
        session, events = self.session()
        session.close()
        session.close()
        self.assertEqual(events, ['stop', 'drain', 'detach', 'destroy'])

    def test_default_drain_does_not_request_generation_stop(self):
        session, events = self.session(opt_in=False)
        session.close()
        self.assertEqual(events, ['drain', 'detach', 'destroy'])

    def test_stop_failure_cleans_resources_without_entering_unbounded_drain(self):
        session, events = self.session(stop_error=True)
        with self.assertRaisesRegex(SmokeError, 'generation stop'):
            session.close()
        self.assertEqual(events, ['stop', 'detach', 'destroy'])

    def test_writer_failure_still_cleans_resources_and_fails(self):
        session, events = self.session(drain_error=True)
        with self.assertRaisesRegex(SmokeError, 'writer drain'):
            session.close()
        self.assertEqual(events, ['stop', 'drain', 'detach', 'destroy'])
