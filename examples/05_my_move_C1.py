#!/usr/bin/env python

import numpy as np
import cv2
from pathlib import Path
from ctu_bosch_sr450 import RobotBosch

simulation = True
if simulation:
    robot = RobotBosch(tty_dev=None)
else:
    # Initialize the robot
    robot = RobotBosch()
    robot.initialize()

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
    # Initialize variables for tracking zones
    previous_zone = None
    zone_3_solution = None
    last_zone_2_points = []  # To store the last two Zone 2 points with their original indexes
    z_drawing = 0.2

    for idx, (x, y) in enumerate(ordered_coordinates):
        z_drawing = 0.2  # Drawing height
        z_rise = 0.5  # Safe height
        ik_solutions = robot.ik_xyz(x=x, y=y, z=z_drawing)

        if not ik_solutions:
            print(f"No IK solution for point ({x}, {y}, {z_drawing}). Skipping...")
            continue

        # Determine the zone and the chosen IK solution
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

        # Handle transition from Zone 2 to Zone 3
        if previous_zone == 2 and zone == 3:
            print("Transitioning from Zone 2 to Zone 3: Revisiting Zone 2 points using Zone 3 IK solution...")
            for revisit_idx, (original_idx, px, py, _) in enumerate(last_zone_2_points):
                revisit_ik_solutions = robot.ik_xyz(x=px, y=py, z=z_drawing)
                print(f"revisit ik solutions : {revisit_ik_solutions}")
                if revisit_ik_solutions:
                    zone_3_revisit_solution = min(
                    revisit_ik_solutions, key=lambda q: np.linalg.norm(q - chosen_solution))  # Use the first solution from Zone 3
                    print(f"Revisiting Zone 2 Point {revisit_idx} (Original Index: {original_idx}) with Zone 3 solution: {zone_3_revisit_solution}")
                    robot.move_to_q(robot.ik_xyz(x=px, y=py, z=z_rise)[0])  # Move up
                    robot.wait_for_motion_stop()
                    robot.move_to_q(zone_3_revisit_solution)
                    robot.wait_for_motion_stop()

            print("Rising up to safe height before entering Zone 3...")
            robot.move_to_q(robot.ik_xyz(x=x, y=y, z=z_rise)[0])  # Move up to safe height
            robot.wait_for_motion_stop()

        # Move to the point with the chosen IK solution
        robot.move_to_q(chosen_solution)
        robot.wait_for_motion_stop()

        # Lower back to drawing height after transition
        if previous_zone == 2 and zone == 3:
            print("Lowering back to drawing height...")
            robot.move_to_q(robot.ik_xyz(x=x, y=y, z=z_drawing)[0])
            robot.wait_for_motion_stop()

        # Track Zone 2 points
        if zone == 2:
            last_zone_2_points.append((idx, x, y, chosen_solution))  # Include original index
            if len(last_zone_2_points) > 2:
                last_zone_2_points.pop(0)  # Keep only the last two points

        # Track the previous zone
        previous_zone = zone

        # Update reference solution
        reference_solution = chosen_solution

    # Revisit the last two points from Zone 2
    if last_zone_2_points:
        print("\nRevisiting last two points from Zone 2...")
        for revisit_idx, (original_idx, x, y, solution) in enumerate(last_zone_2_points):
            print(
                f"Revisiting Zone 2 Point {revisit_idx} (Original Index: {original_idx}): ({x}, {y}, {z_drawing}) with solution: {solution}")
            robot.move_to_q(solution)
            robot.wait_for_motion_stop()

    robot.soft_home()


if __name__ == "__main__":
    main()
