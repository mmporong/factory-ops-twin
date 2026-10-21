"""Bounded dispatch regressions; no SDK, DDS or physical replay claim."""
import copy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from scripts.ros.g3_mission import sample_localization_batch


FIXTURE = json.loads((Path(__file__).parent / 'fixtures/local45-038-tf-freshness.json').read_text())


def transform(ns):
    sec, nanosec = divmod(ns, 10**9)
    return NS(header=NS(frame_id='map', stamp=NS(sec=sec, nanosec=nanosec)),
        child_frame_id='base_link', transform=NS(translation=NS(x=0.,y=0.,z=0.),
        rotation=NS(x=0.,y=0.,z=0.,w=1.)))


class LocalizationDispatchTests(unittest.TestCase):
    def sample(self, clocks, stamps, **changes):
        calls = {'drain': Mock(return_value={'wall_budget_reached': True}),
            'read_clock_ns': Mock(side_effect=clocks),
            'lookup': Mock(side_effect=[transform(x) if x is not None else None for x in stamps]),
            'cached': {'carter1': None}, 'ttl_s': .5, 'require_fresh': True,
            'max_attempts': 3, 'checkpoint': Mock()}
        calls.update(changes)
        result = sample_localization_batch(('carter1',), **calls)
        return result, calls

    def test_closed038_newer_callback_can_recover_with_actual_clock_reread(self):
        f = FIXTURE
        result, calls = self.sample([f['failure_clock_ns'],f['next_recorded_clock_ns']],
            [f['stale_composed_tf_ns'],f['newer_recorded_odom_tf_ns']])
        self.assertEqual(result[0], f['next_recorded_clock_ns'])
        self.assertEqual(result[1]['carter1']['stamp_s'], f['newer_recorded_odom_tf_ns']/1e9)
        self.assertEqual(calls['drain'].call_count, 2)
        state = calls['checkpoint'].call_args.args[0]
        self.assertEqual(state['status'], 'recovered')
        self.assertEqual(state['ttl_s'], .5)

    def test_persistent_stale_pose_never_qualifies(self):
        diagnostic = Mock()
        with self.assertRaisesRegex(RuntimeError, 'fresh map pose unavailable'):
            self.sample([1_000_000_000]*3, [400_000_000]*3, checkpoint=diagnostic)
        self.assertEqual(diagnostic.call_args.args[0]['status'], 'failed')
        self.assertEqual(len(diagnostic.call_args.args[0]['attempts']), 3)

    def test_newer_transform_must_be_fresh_against_new_clock(self):
        with self.assertRaises(RuntimeError):
            self.sample([1_000_000_000,2_000_000_000,3_000_000_000],
                        [400_000_000,1_000_000_000,2_000_000_000])

    def test_missing_and_future_transforms_fail_closed(self):
        for stamps in ([None]*3, [2_000_000_000]*3):
            with self.subTest(stamps=stamps), self.assertRaises(RuntimeError):
                self.sample([1_000_000_000]*3, stamps)

    def test_existing_fresh_cache_and_single_attempt_behavior_are_preserved(self):
        cache = {'carter1': {'stamp_s': .8}}
        prior = copy.deepcopy(cache)
        result, calls = self.sample([1_000_000_000], [None], cached=cache, max_attempts=1)
        self.assertEqual(result[1], {'carter1': None})
        self.assertEqual(cache, prior)
        calls['checkpoint'].assert_not_called()
        with self.assertRaises(RuntimeError):
            self.sample([1_000_000_000], [None], max_attempts=1)

    def test_clock_backward_and_malformed_transform_do_not_retry_through_error(self):
        with self.assertRaisesRegex(ValueError, 'monotonic'):
            self.sample([1_000_000_000,900_000_000], [None])
        invalid = transform(1_000_000_000)
        invalid.header.frame_id = 'wrong'
        reader = Mock(return_value=invalid)
        with self.assertRaisesRegex(ValueError, 'invalid frame'):
            self.sample([1_000_000_000], [], lookup=reader)
        self.assertEqual(reader.call_count, 1)

    def test_original_dependency_deadline_guard_still_aborts(self):
        drain = Mock(side_effect=[{}, RuntimeError('original deadline')])
        with self.assertRaisesRegex(RuntimeError, 'original deadline'):
            self.sample([1_000_000_000], [None], drain=drain)

    def test_partial_batch_never_mutates_cache_or_returns_partial_acceptance(self):
        cache = {'a': None, 'b': None}
        drain, diagnostic = Mock(return_value={}), Mock()
        lookup = Mock(side_effect=[transform(1_000_000_000), None]*3)
        with self.assertRaisesRegex(RuntimeError, 'b'):
            sample_localization_batch(('a','b'), drain=drain,
                read_clock_ns=lambda: 1_000_000_000, lookup=lookup, cached=cache,
                ttl_s=.5, require_fresh=True, max_attempts=3, checkpoint=diagnostic)
        self.assertEqual(cache, {'a': None, 'b': None})
        self.assertEqual(drain.call_count, 3)


if __name__ == '__main__':
    unittest.main()
