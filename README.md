# ctu_bosh_sr450

Package to control CTU/CIIRC robot Bosh SR450 via MARS control unit.
This code is based on `https://github.com/cvut/pyrocon` but adds packaging,
documentation, and some testing.

## Installation

```bash
pip install ctu_bosh_sr450
```

## Usage

```python
from ctu_bosh_sr450 import RobotBosh

robot = RobotBosh(tty_dev=None)
robot.initialize()
robot.move_to_q([0.1, 0.0, 0.0, 0.0])
robot.close()
```

## Coordinate systems

The library uses meters and radians for all values.
Variable __q__ is used to denote joint configuration, i.e. the array of joint
angles/joint distance for revolute and prismatic joints, respectively.
Variables __x__, __y__, __z__, and __phi__ are used to denote position and orientation
of the end-effector in the base frame. The orientation is given as rotation around the
z-axis of the base frame.
The reference base frame is located as shown in the figure below.

![](https://raw.githubusercontent.com/CTURobotics/ctu_bosh_sr450/main/doc/base_frame.png)

## Joint configuration

The joint configuration is defined as follows:

- the first joint is revolute and its angle is measured w.r.t. the x-axis of the base
  frame
- the second joint is revolute and is measured w.r.t. the previous link
- the third joint is prismatic and controls the height (i.e. motion in z-axis)
- the last joint is revolute and measured w.r.t. the x-axis of the base frame (i.e. *
  *not** w.r.t. the previous link)
