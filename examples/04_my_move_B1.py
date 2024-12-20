#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
import numpy as np
import cv2
from pathlib import Path
from ctu_bosch_sr450 import RobotBosch

# Initialize the robot
robot = RobotBosch(tty_dev=None)
#robot.initialize()

# Path to the saved coordinates file
script_dir = Path(__file__).parent.parent
coordinates_file = script_dir / "drawings" / "robot_coordinates.txt"


# Function to read coordinates from file
def read_coordinates(file_path):
    with open(file_path, "r") as f:
        return [tuple(map(float, line.strip().split(","))) for line in f]

def euclidean_distance(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# Function to approximate the path with fewer points
def approximate_path(points, num_points):
    indices = np.linspace(0, len(points) - 1, num_points, dtype=int)
    return [points[i] for i in indices]

# Function to reorder points by connecting the closest pair
def reorder_points_by_distance(points):
    ordered_points = [points[0]]  # Start with the first point
    remaining_points = points[1:]  # The rest of the points

    while remaining_points:
        last_point = ordered_points[-1]
        # Find the closest point to the last ordered point
        closest_point = min(remaining_points, key=lambda p: euclidean_distance(last_point, p))
        ordered_points.append(closest_point)
        remaining_points.remove(closest_point)

    return ordered_points

# Function to map coordinates for visualization
def map_coordinates(points, img_size, padding=10):
    x_coords, y_coords = zip(*points)
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    # Uniform scaling factor to preserve aspect ratio
    scale = min(
        (img_size - 2 * padding) / (x_max - x_min),
        (img_size - 2 * padding) / (y_max - y_min),
    )

    return [
        (int((x - x_min) * scale + padding),
         int(img_size - ((y - y_min) * scale + padding)))  # Flip Y-axis
        for x, y in points
    ]


# Function to visualize path
def visualize_path(points, img_size=500):
    img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
    mapped_points = map_coordinates(points, img_size)

    for i in range(len(mapped_points) - 1):
        cv2.line(img, mapped_points[i], mapped_points[i + 1], (0, 255, 0), 2)

    for idx, point in enumerate(mapped_points):
        cv2.circle(img, point, 2, (0, 0, 255), -1)
        cv2.putText(img, str(idx), (point[0] + 5, point[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    cv2.imshow("Path Visualization", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# Main program logic
def main():
    # Step 1: Load and reduce trajectory points
    coordinates = read_coordinates(coordinates_file)
    reduced_coordinates = approximate_path(coordinates, num_points=20)

    print("Reduced Coordinates:")
    for coord in reduced_coordinates:
        print(coord)

    # Step 2: Reorder the reduced coordinates by connecting closest points
    ordered_coordinates = reorder_points_by_distance(reduced_coordinates)

    print("Ordered Coordinates by Proximity:")
    for coord in ordered_coordinates:
        print(coord)

    # Step 3: Visualize the reordered path
    visualize_path(ordered_coordinates)
    # Step 4: Solve IK for the first point to fix initial configuration
    x_start, y_start = ordered_coordinates[0]
    z_start = 0.4  # Fixed Z height
    phi = np.deg2rad(45)  # Fixed orientation

    ik_solutions = robot.ik_xyz(x=x_start, y=y_start, z=z_start)
    print(f"IK solutions: {len(ik_solutions)}")
    if not ik_solutions:
        print(f"No IK solution for start point ({x_start}, {y_start}, {z_start})")
        robot.close()
        return

    # Use the solution that is valid for entire trajectory
    reference_solution = ik_solutions[0]
    print(f"Using fixed joint configuration: {reference_solution}")
    #robot.move_to_q(reference_solution)
    #robot.wait_for_motion_stop()

    # Step 5: Execute trajectory with fixed IK solution
    for x, y in ordered_coordinates:
        z = 0.2  # Constant Z height for simplicity
        ik_solutions = robot.ik_xyz(x=x, y=y, z=z)
        print(f"IK solutions: {len(ik_solutions)}")
        if not ik_solutions:
            print(f"No IK solution for point ({x}, {y}, {z})")
            continue

        # Select the solution closest to the reference solution
        closest_solution = min(
            ik_solutions, key=lambda q: np.linalg.norm(q - reference_solution)
        )

        print(f"Moving to ({x}, {y}, {z}) with closest IK solution: {closest_solution}")
        #robot.move_to_q(closest_solution)
        #robot.wait_for_motion_stop()

        # Update reference solution

        reference_solution = closest_solution

    # Step 6: Close the robot connection
    robot.soft_home()
    robot.close()
    print("Trajectory execution complete.")


if __name__ == "__main__":
    main()