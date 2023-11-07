#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from unittest import TestCase
import numpy as np

from ctu_bosch_sr450.circle_circle_intersection import circle_circle_intersection


class TestCircleCircleIntersection(TestCase):
    def test_circle_circle_intersection(self):
        """Test circle_circle_intersection function."""

        ints = circle_circle_intersection([0, 0], 1, [1, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        if by > ay:
            ay, by = by, ay
        self.assertAlmostEqual(ax, 0.5)
        self.assertAlmostEqual(ay, np.sqrt(1 - 0.5**2))
        self.assertAlmostEqual(bx, 0.5)
        self.assertAlmostEqual(by, -np.sqrt(1 - 0.5**2))

        ints = circle_circle_intersection([0, 0], 1, [2, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        self.assertAlmostEqual(ax, 1.0)
        self.assertAlmostEqual(ay, 0.0)
        self.assertAlmostEqual(bx, 1.0)
        self.assertAlmostEqual(by, 0.0)

        ints = circle_circle_intersection([0, 0], 1, [0, 0], 1)
        self.assertEqual(len(ints), 2)
        a, b = ints
        self.assertAlmostEqual(np.linalg.norm(a), 1.0)
        self.assertAlmostEqual(np.linalg.norm(b), 1.0)
