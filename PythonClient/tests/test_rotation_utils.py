"""Rotation utility regressions; no simulator needed."""
import math
import unittest
import numpy as np
from cosysairsim import utils
from cosysairsim.types import Quaternionr, Vector3r


class RotationUtilsTests(unittest.TestCase):
    def test_quarter_turns(self):
        cases = [((0, 0, math.pi / 2), [1, 0, 0], [0, 1, 0]),
                 ((0, math.pi / 2, 0), [1, 0, 0], [0, 0, -1]),
                 ((math.pi / 2, 0, 0), [0, 1, 0], [0, 0, 1])]
        for angles, vector, expected in cases:
            with self.subTest(angles=angles):
                matrix = np.asarray(utils.euler_to_rotation_matrix(*angles))
                np.testing.assert_allclose(matrix @ vector, expected, atol=1e-12)

    def test_combined_angles_match_quaternion_rotation(self):
        for angles in [(0.3, -0.4, 0.7), (-0.8, 0.5, -1.2), (0, 0, 0)]:
            with self.subTest(angles=angles):
                q = utils.euler_to_quaternion(*angles)
                v = Quaternionr(2, -3, 5, 0)
                expected = q * v * q.conjugate()
                matrix = np.asarray(utils.euler_to_rotation_matrix(*angles))
                np.testing.assert_allclose(matrix @ [2, -3, 5],
                    [expected.x_val, expected.y_val, expected.z_val], atol=1e-12)
                np.testing.assert_allclose(matrix.T @ matrix, np.eye(3), atol=1e-12)
                self.assertAlmostEqual(np.linalg.det(matrix), 1.0)

    def test_apply_rotation_offset_keeps_argument_order(self):
        # Public helper takes pitch, yaw, roll (not roll, pitch, yaw).
        actual = utils.apply_rotation_offset(Vector3r(1, 0, 0), 0, math.pi / 2, 0)
        np.testing.assert_allclose([actual.x_val, actual.y_val, actual.z_val], [0, 1, 0], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
