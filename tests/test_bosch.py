#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-6
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import unittest
import numpy as np
from ctu_bosch_sr450 import RobotBosch


class TestBosch(unittest.TestCase):
    def test_fk(self):
        robot = RobotBosch(tty_dev=None)
        x, y, _, phi = robot.fk([0, 0, 0, 0])
        self.assertAlmostEqual(x, np.sum(robot.link_lengths))
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(phi, 0)

        self.assertAlmostEqual(robot.fk([0, 0, 0, np.deg2rad(10)])[-1], np.deg2rad(10))
        self.assertAlmostEqual(
            robot.fk([0, 0, 0, np.deg2rad(10 - 360)])[-1], np.deg2rad(10)
        )
        self.assertAlmostEqual(
            robot.fk([0, 0, 0, np.deg2rad(10 + 360)])[-1], np.deg2rad(10)
        )
        self.assertAlmostEqual(
            robot.fk([0, 0, 0, np.deg2rad(10 + 2 * 360)])[-1], np.deg2rad(10)
        )

    def test_ik_xyz(self):
        np.random.seed(0)
        robot = RobotBosch(tty_dev=None)
        for _ in range(1000):
            q = np.random.uniform(robot.q_min, robot.q_max)
            x, y, z, phi = robot.fk(q)
            ik_sols = robot.ik_xyz(x, y, z, q[3])
            self.assertTrue(any(np.allclose(q, ik_sol) for ik_sol in ik_sols))

            for q_sol in ik_sols:
                x_sol, y_sol, z_sol, phi_sol = robot.fk(q_sol)
                self.assertAlmostEqual(x, x_sol)
                self.assertAlmostEqual(y, y_sol)
                self.assertAlmostEqual(z, z_sol)
                self.assertAlmostEqual(phi, phi_sol)

    def test_ik_xyzphi(self):
        np.random.seed(0)
        robot = RobotBosch(tty_dev=None)
        for _ in range(1000):
            q = np.random.uniform(robot.q_min, robot.q_max)
            x, y, z, phi = robot.fk(q)
            ik_sols = robot.ik(x, y, z, phi)
            self.assertTrue(any(np.allclose(q, ik_sol) for ik_sol in ik_sols))

            for q_sol in ik_sols:
                x_sol, y_sol, z_sol, phi_sol = robot.fk(q_sol)
                self.assertAlmostEqual(x, x_sol)
                self.assertAlmostEqual(y, y_sol)
                self.assertAlmostEqual(z, z_sol)
                self.assertAlmostEqual(phi, phi_sol)


if __name__ == "__main__":
    unittest.main()
