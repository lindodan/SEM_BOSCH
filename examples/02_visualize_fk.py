#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-3
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import matplotlib.pyplot as plt

from ctu_bosh_sr450 import RobotBosh

robot = RobotBosh()

x, y, z, phi = robot.fk([0, 0, 0, 0])

fig, ax = plt.subplots(1, 1, squeeze=True)  # type: plt.Figure, plt.Axes
ax.plot()
