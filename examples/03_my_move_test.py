#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
import cv2
from pathlib import Path
from scipy.spatial import KDTree
from ctu_bosch_sr450 import RobotBosch

# Initialize the robot
robot = RobotBosch()
robot.initialize()

# Path to the saved coordinates file
script_dir = Path(__file__).parent.parent
coordinates_file = script_dir / "drawings" / "robot_coordinates.txt"

# Read the coordinates from the file
with open(coordinates_file, "r") as f:
    coordinates = [tuple(map(float, line.strip().split(","))) for line in f]

# Function to approximate the path with fewer points
def approximate_path(points, num_points):
    tree = KDTree(points)
    sampled_indices = np.linspace(0, len(points) - 1, num_points, dtype=int)
    sampled_points = [points[i] for i in sampled_indices]
    return sampled_points

# Reduce the number of points for the trajectory
reduced_coordinates = approximate_path(coordinates, num_points=50)

# Visualize the reduced path using OpenCV
img_size = 1000
img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

# Normalize coordinates for visualization
def normalize_coordinates(points, img_size):
    x_coords, y_coords = zip(*points)
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    def scale(val, min_val, max_val):
        return int((val - min_val) / (max_val - min_val) * (img_size - 20) + 10)

    normalized_points = [(scale(x, x_min, x_max), img_size - scale(y, y_min, y_max)) for x, y in points]
    return normalized_points

normalized_coordinates = normalize_coordinates(reduced_coordinates, img_size)

# Draw the path
for i in range(len(normalized_coordinates) - 1):
    cv2.line(img, normalized_coordinates[i], normalized_coordinates[i + 1], (0, 255, 0), 2)

# Draw points
for point in normalized_coordinates:
    cv2.circle(img, point, 5, (0, 0, 255), -1)

# Show the visualization
cv2.imshow("Reduced Path Visualization", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Solve IK for the first point to get the initial configuration
x_start, y_start = reduced_coordinates[0]
z_start = 0.4  # Fixed Z height
phi = np.deg2rad(45)  # Fixed orientation

initial_ik_solutions = robot.ik(x=x_start, y=y_start, z=z_start, phi=phi)

if not initial_ik_solutions:
    print(f"No IK solution found for the starting point ({x_start}, {y_start}, {z_start})")
    robot.close()
    exit(1)

# Use the first valid solution for the entire path
initial_solution = initial_ik_solutions[0]
print(f"Using initial joint configuration: {initial_solution}")
robot.move_to_q(initial_solution)
robot.wait_for_motion_stop()

# Iterate through the reduced trajectory using the initial IK solution
for x, y in reduced_coordinates:
    z = 0.2  # Assuming a fixed Z height for simplicity

    print(f"Moving to ({x}, {y}, {z}) with the initial configuration.")
    robot.move_to_q(initial_solution)
    robot.wait_for_motion_stop()

# Close the robot connection
robot.close()
