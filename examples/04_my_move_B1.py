#!/usr/bin/env python
#

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



def read_coordinates(file_path):
    '''
    Function that maps coordinates from a file to numpy array
    :param file_path:
    :return: coordinates
    '''
    with open(file_path, "r") as f:
        return [tuple(map(float, line.strip().split(","))) for line in f]

def euclidean_distance(p1, p2):
    '''
    Function that calculates the Euclidean distance between two points
    :param p1:
    :param p2:
    :return: Distance between the two points
    '''
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def approximate_path(points, num_points):
    '''
    Function that approximates the path so the robot does not move from pixel to pixel
    and can move more smoothly
    :param points:
    :param num_points:
    :return:
    '''
    indices = np.linspace(0, len(points) - 1, num_points, dtype=int)
    return [points[i] for i in indices]


def select_initial_point(points):
    '''
    Selects the initial point based on the lowest x-value
    and within ±15 pixels on the y-axis from the current first point.
    :param points: List of points (tuples)
    :return: The selected initial point
    '''
    # Take the point with the lowest x-value, filtering by the ±15 y-axis constraint
    initial_y = points[0][1]
    filtered_points = [p for p in points if abs(p[1] - initial_y) <= 15]

    if not filtered_points:
        raise ValueError("No points within ±15 pixels of the y-axis from the initial point.")

    return min(filtered_points, key=lambda p: p[0])


def reorder_points_by_distance(points):
    '''
    Function that reorders the points according to the distance
    so we connect the two closest ones, starting from the selected initial point.
    :param points:
    :return: ordered points
    '''
    # Select the initial point
    initial_point = select_initial_point(points)
    points.remove(initial_point)

    ordered_points = [initial_point]  # Start with the selected initial point
    remaining_points = points  # The rest of the points

    while remaining_points:
        last_point = ordered_points[-1]
        # Find the closest point to the last ordered point by Euclidean distance
        closest_point = min(remaining_points, key=lambda p: euclidean_distance(last_point, p))
        ordered_points.append(closest_point)
        remaining_points.remove(closest_point)

    return ordered_points


def map_coordinates(points, img_size, padding=100):
    '''
    Function that maps coordinates to numpy array for visualization purposes
    :param points:
    :param img_size:
    :param padding:
    :return:
    '''
    x_coords, y_coords = zip(*points)
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)

    scale = min(
        (img_size - 2 * padding) / (x_max - x_min),
        (img_size - 2 * padding) / (y_max - y_min),
    )

    return [
        (int((x - x_min) * scale + padding),
         int(img_size - ((y - y_min) * scale + padding)))
        for x, y in points
    ]

def visualize_path(points, img_size=500):
    '''
    Function that visualizes the trajectory of robot
    :param points:
    :param img_size:
    :return:
    '''
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

def main():
    '''
    Main function
    :return:
    '''

    # Heights for used in task
    z_start = 0.4
    z_drawing = 0.186

    # Load and reduce trajectory points
    coordinates = read_coordinates(coordinates_file)
    reduced_coordinates = approximate_path(coordinates, num_points=25)

    #Reorder the reduced coordinates by connecting two closest points
    ordered_coordinates = reorder_points_by_distance(reduced_coordinates)

    # Visualize the reordered path
    visualize_path(ordered_coordinates)

    # Solve IK for the first point to fix initial configuration
    x_start, y_start = ordered_coordinates[0]

    ik_solutions = robot.ik_xyz(x=x_start, y=y_start, z=z_start)
    print(f"IK solutions: {ik_solutions}")
    print(f"IK solutions: {len(ik_solutions)}")
    if not ik_solutions:
        print(f"No IK solution for start point ({x_start}, {y_start}, {z_start})")
        robot.close()
        return

    # Choose the solution that is valid for entire path
    for x,y in ordered_coordinates:
        ik_solutions = robot.ik_xyz(x=x,y=y,z=z_start)
        if len(ik_solutions) == 1:
            reference_solution = ik_solutions[0]
            print(f"Using fixed joint configuration: {reference_solution}")
            break

    if not simulation:
        robot.move_to_q(reference_solution)
        robot.wait_for_motion_stop()

    # Step 5: Execute trajectory with fixed IK solution
    for x, y in ordered_coordinates:
        ik_solutions = robot.ik_xyz(x=x, y=y, z=z_drawing)

        print(f"IK solutions: {ik_solutions}")
        print(f"IK solutions: {len(ik_solutions)}")
        if not ik_solutions:
            print(f"No IK solution for point ({x}, {y}, {z_drawing})")
            continue

        # Select the solution closest to the reference solution
        closest_solution = min(
            ik_solutions, key=lambda q: np.linalg.norm(q - reference_solution)
        )

        print(f"Moving to ({x}, {y}, {z_drawing}) with closest IK solution: {closest_solution}")
        if not simulation:
            robot.move_to_q(reference_solution)
            robot.wait_for_motion_stop()

        # Update reference solution

        reference_solution = closest_solution

    # From last point go up
    x, y = ordered_coordinates[-1]
    ik_solutions = robot.ik_xyz(x=x, y=y, z=z_start)
    closest_solution = min(
        ik_solutions, key=lambda q: np.linalg.norm(q - reference_solution)
    )
    if not simulation:
        robot.move_to_q(closest_solution)
        robot.wait_for_motion_stop()

    # Step 6: Close the robot connection
    robot.soft_home()
    robot.close()
    print("Trajectory execution complete.")


if __name__ == "__main__":
    main()