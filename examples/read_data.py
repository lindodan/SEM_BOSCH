import cv2
import numpy as np
from pathlib import Path

# Step 1: Path setup and load the image
script_dir = Path(__file__).parent.parent / 'drawings'
print(f"Script directory: {script_dir}")

image_path = script_dir / "C-1.png"
img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)

# Step 2: Invert and Threshold the image
_, binary_img = cv2.threshold(img, 128, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
cv2.imshow("Binary Image", binary_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Step 3: Find all black pixels (non-zero pixels in binary image)
black_pixels = cv2.findNonZero(binary_img)  # Find all non-zero (white) pixels in binary image
black_pixels = black_pixels[:, 0]  # Reshape to Nx2 (x, y)

# Step 4: Center and convert to robot coordinates
robot_coordinates_mm = []
center_x = 500  # Center X pixel (robot center)
center_y = 500  # Center Y pixel (robot center)

for x, y in black_pixels:
    x_robot_mm = x - center_x  # Convert to robot's X coordinate in mm
    y_robot_mm = center_y - y  # Convert to robot's Y coordinate in mm
    robot_coordinates_mm.append((x_robot_mm, y_robot_mm))

# Step 5: Convert coordinates to meters
robot_coordinates_m = [(x / 1000, y / 1000) for x, y in robot_coordinates_mm]

# Step 6: Sort coordinates (y first, then x for trajectory)
coordinates = robot_coordinates_m

# Print coordinates
print("Sorted Robot Coordinates (meters):")
for coord in coordinates:
    print(f"{coord[0]:.3f}, {coord[1]:.3f}")

# Step 7: Save to file
output_file = script_dir / "robot_coordinates.txt"
with open(output_file, "w") as f:
    for x, y in coordinates:
        f.write(f"{x:.3f}, {y:.3f}\n")

print(f"Robot coordinates saved to: {output_file}")

# Step 8: Optional - Visualize the trajectory on the original image
output_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
for x_robot_mm, y_robot_mm in robot_coordinates_mm:
    x_pixel = int(x_robot_mm + center_x)
    y_pixel = int(center_y - y_robot_mm)
    cv2.circle(output_img, (x_pixel, y_pixel), 1, (0, 0, 255), -1)

cv2.imshow("Trajectory Visualization", output_img)
cv2.imwrite(str(script_dir / "trajectory_visualized.png"), output_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

print("Trajectory visualization saved as trajectory_visualized.png")
