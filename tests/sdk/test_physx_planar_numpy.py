"""Explicit SDK Python/NumPy equivalence tests, outside CPU-only discovery."""
from copy import deepcopy
import unittest
from unittest.mock import patch

import numpy as np

from factory_ops import physx_planar as planar
from tests.test_physx_planar import fixture


def native_fixture(dtype=np.float32):
    value = fixture()
    value['linear_depth_data'] = np.array(value['linear_depth_data'], dtype=dtype)
    value['intensity_data'] = np.array(value['intensity_data'], dtype=np.uint8)
    value['azimuth_rad'] = np.array(value['azimuth_rad'], dtype=dtype)
    value['zenith_rad'] = np.array(value['zenith_rad'], dtype=dtype)
    value['prim_paths'] = tuple('/World/Wall' for _ in range(720))
    value['raw_shapes'] = {name: tuple(value[name].shape) for name in
        ('linear_depth_data','intensity_data','azimuth_rad','zenith_rad')}
    value['raw_dtypes'] = {name: str(value[name].dtype) for name in value['raw_shapes']}
    return value


class NumpyPlanarTests(unittest.TestCase):
    def scalar(self, value):
        with patch.object(planar, '_numpy_vectors', return_value=None):
            return planar._validate(value)

    def test_finite_native_arrays_keep_values_sort_signature_and_ownership(self):
        for dtype in (np.float32, np.float64, '>f4'):
            for reverse in (False, True):
                value = native_fixture(dtype)
                value['linear_depth_data'][0:3, 0] = (.1, 50., 30.011854)
                value['intensity_data'][17, 0] = 0
                value['linear_depth_data'][17, 0] = -0.
                if reverse:
                    for name in ('linear_depth_data','intensity_data','azimuth_rad'):
                        value[name] = value[name][::-1]
                before = {name: value[name].tobytes() for name in value['raw_shapes']}
                self.assertIsNotNone(planar._numpy_vectors(value['linear_depth_data'],
                    value['intensity_data'], value['azimuth_rad']))
                self.assertEqual(self.scalar(value), planar._validate(value))
                self.assertEqual(before, {name: value[name].tobytes() for name in before})

    def test_invalid_native_samples_retain_scalar_rejection_details(self):
        for change in (
            lambda v: v['linear_depth_data'].__setitem__((23, 0), np.float32(50.01)),
            lambda v: v['linear_depth_data'].__setitem__((23, 0), np.nan),
            lambda v: v['intensity_data'].__setitem__((23, 0), 1),
            lambda v: v['azimuth_rad'].__setitem__(23, np.inf),
            lambda v: v.__setitem__('azimuth_rad', v['azimuth_rad'].astype(np.float16)),
            lambda v: v.__setitem__('intensity_data', v['intensity_data'].astype(bool)),
        ):
            value = native_fixture(); change(value)
            errors = []
            for scalar in (True, False):
                try:
                    self.scalar(value) if scalar else planar._validate(value)
                except ValueError as error:
                    errors.append((type(error), str(error), dict(getattr(error, 'details', {}))))
            self.assertEqual(2, len(errors))
            self.assertEqual(errors[0], errors[1])

    def test_nohit_nonfinite_and_changed_same_frame_are_not_hidden(self):
        value = native_fixture()
        value['intensity_data'][17, 0] = 0
        value['linear_depth_data'][17, 0] = np.nan
        self.assertEqual(self.scalar(value)['signature'], planar._validate(value)['signature'])
        value = native_fixture()
        publisher = planar.PhysxPlanarPublisher(lambda fields: None, 'scan')
        publisher.publish(value)
        changed = deepcopy(value)
        changed['linear_depth_data'][17, 0] += .1
        with self.assertRaisesRegex(ValueError, 'same-frame acquisition payload changed'):
            publisher.publish(changed)
