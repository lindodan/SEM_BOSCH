#!/usr/bin/env python

import numpy as np
import cv2
from pathlib import Path
from ctu_bosch_sr450 import RobotBosch

# Initialize the robot
robot = RobotBosch(tty_dev=None)

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
        closest_point = min(remaining_points, key=lambda p: euclidean_distance(last_point, p))
        ordered_points.append(closest_point)
        remaining_points.remove(closest_point)

    return ordered_points

# Function to map coordinates for visualization
def map_coordinates(points, img_size, padding=10):
    x_coords, y_coords = zip(*points)
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

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
    coordinates = read_coordinates(coordinates_file)
    reduced_coordinates = approximate_path(coordinates, num_points=20)

    ordered_coordinates = reorder_points_by_distance(reduced_coordinates)
    visualize_path(ordered_coordinates)

    reference_solution = None
    zone_3_solution = None
    zone_2_revisit_points = []

    for idx, (x, y) in enumerate(ordered_coordinates):
        z_drawing = 0.2  # Height for drawing
        z_rise = 0.5  # Safe height above the desk
        ik_solutions = robot.ik_xyz(x=x, y=y, z=z_drawing)

        if not ik_solutions:
            print(f"No IK solution for point ({x}, {y}, {z_drawing}). Skipping...")
            continue

        if len(ik_solutions) == 1:
            chosen_solution = ik_solutions[0]
            zone = 1 if reference_solution is None or np.array_equal(reference_solution, chosen_solution) else 3
        else:
            if reference_solution is None:
                chosen_solution = ik_solutions[0]
            else:
                chosen_solution = min(
                    ik_solutions, key=lambda q: np.linalg.norm(q - reference_solution)
                )
            zone = 2

        print(f"Point {idx}: Zone {zone}, Moving to ({x}, {y}, {z_drawing}) with solution: {chosen_solution}")

        if reference_solution is not None and not np.array_equal(reference_solution, chosen_solution):
            # If changing IK solutions, rise up before moving
            print(f"Rising up to safe height before changing solution...")
           # robot.move_to_q(robot.ik_xyz(x=x, y=y, z=z_rise)[0])  # Safe height
            #robot.wait_for_motion_stop()

        # Move to the new position with the chosen IK solution
        #robot.move_to_q(chosen_solution)
        #robot.wait_for_motion_stop()

        if reference_solution is not None and not np.array_equal(reference_solution, chosen_solution):
            # Lower back down to drawing height after changing solution
            print(f"Lowering back to drawing height...")
            #robot.move_to_q(robot.ik_xyz(x=x, y=y, z=z_drawing)[0])
            #robot.wait_for_motion_stop()

        if zone == 3:
            zone_3_solution = chosen_solution
            zone_2_revisit_points.append((idx, (x, y, z_drawing)))

        reference_solution = chosen_solution

    # Handle Zone 2 revisit points
    if zone_3_solution:
        print("Revisiting Zone 2 points with Zone 3 solution compatibility...")
        for idx, (x, y, z) in zone_2_revisit_points:
            ik_solutions = robot.ik_xyz(x=x, y=y, z=z)
            valid_solutions = [
                q for q in ik_solutions if np.array_equal(q, zone_3_solution)
            ]
            if valid_solutions:
                chosen_solution = valid_solutions[0]
                print(f"Revisiting Point {idx} with solution: {chosen_solution}")
                # Uncomment to move the robot
                # robot.move_to_q(chosen_solution)
                # robot.wait_for_motion_stop()

    robot.soft_home()
    robot.close()
    print("Trajectory execution complete.")

if __name__ == "__main__":
    main()
