#!/usr/bin/env python

import numpy as np
from pathlib import Path
from ctu_bosch_sr450 import RobotBosch

# Initialize the robot
robot = RobotBosch()
robot.initialize()

# Path to the saved coordinates file
script_dir = Path(__file__).parent.parent
print(script_dir)
coordinates_file = script_dir / "drawings" / "robot_coordinates.txt"

# Read the coordinates from the file
with open(coordinates_file, "r") as f:
    coordinates = [tuple(map(float, line.strip().split(","))) for line in f]
q0 = robot.get_q()
# Iterate through each coordinate and generate trajectory
for x, y in coordinates:
    z = 0.5  # Assuming a fixed Z height for simplicity
    phi = np.deg2rad(45)  # Assuming a fixed orientation

    # Solve inverse kinematics for the coordinate
    ik_solutions = robot.ik(x=x, y=y, z=z, phi=phi)
    if len(ik_solutions) > 0:
        closest_solution = min(ik_solutions,key=lambda q:np.linalg.norm(q-q0))

    if not ik_solutions:
        print(f"No IK solution found for point ({x}, {y}, {z})")
        continue

    # Move to the first valid solution
    for solution in ik_solutions:
        print(f"Moving to joint configuration: {solution}")
        robot.move_to_q(solution)
        robot.wait_for_motion_stop()
        break  # Move to the first valid solution only

# Close the robot connection
robot.close()
