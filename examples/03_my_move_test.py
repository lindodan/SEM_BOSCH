#!/usr/bin/env python

import numpy as np
from pathlib import Path
from scipy.spatial import KDTree
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

def approximate_path(points,num_points):
    tree = KDTree(points)
    sample_indicies = np.linspace(0,len(points)-1,num_points)
    sample_points = [points[i] for i in sample_indicies]
    return sample_points

reduced_coordinates = approximate_path(coordinates,num_points=30)

q0 = robot.get_q()


x_start, y_start = reduced_coordinates[0]
z_start = 0.2  # Fixed Z height
phi = np.deg2rad(45)  # Fixed orientation

ik_solutions = robot.ik(x=x_start, y=y_start, z=z_start, phi=phi)

if len(ik_solutions) > 0:
    initial_solution = min(ik_solutions,key=lambda q:np.linalg.norm(q-q0))


if not ik_solutions:
    print(f"No IK solution found for the starting point ({x_start}, {y_start}, {z_start})")
    robot.close()
    exit(1)

# Use the first valid solution for the entire path
print(f"Using initial joint configuration: {initial_solution}")
robot.move_to_q(initial_solution)
robot.wait_for_motion_stop()

# Iterate through the reduced trajectory using the initial IK solution
for x, y in reduced_coordinates:
    z = 0.2  # Assuming a fixed Z height for simplicity

    # Validate the initial solution for the current point
    if not robot.is_valid_solution(initial_solution, x=x, y=y, z=z, phi=phi):
        print(f"Initial solution is not valid for point ({x}, {y}, {z})")
        continue

    print(f"Moving to ({x}, {y}, {z}) with the initial configuration.")
    robot.move_to_q(initial_solution)
    robot.wait_for_motion_stop()

# Close the robot connection
robot.close()
