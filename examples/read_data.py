import cv2
import numpy as np
from pathlib import Path

# Load image from directory
script_dir = Path(__file__).parent.parent / 'drawings'
print(f"Script directory: {script_dir}")
image_path = script_dir / "C-1.png"
img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)

# Invert the image to black background and white line (binary values)   0 black 255 white
blur_img = cv2.GaussianBlur(img, (5, 5), 0)
_, binary_img = cv2.threshold(blur_img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
cv2.imshow("Binary Image", binary_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Find all white pixels in image
white_pixels= cv2.findNonZero(binary_img)
white_pixels = white_pixels[:, 0]

# Zero out and convert to robot coordinates (500x500) is center
robot_coordinates_mm = []
center_x = 500  # Center X pixel
center_y = 500  # Center Y pixel

for x, y in white_pixels:
    x_robot_mm = x - center_x
    y_robot_mm = center_y - y
    robot_coordinates_mm.append((x_robot_mm, y_robot_mm))

# Convert coordinates to meters
robot_coordinates_m = [(x / 1000, y / 1000) for x, y in robot_coordinates_mm]

coordinates = robot_coordinates_m
print("Robot Coordinates (meters):")
for coord in coordinates:
    print(f"{coord[0]:.3f}, {coord[1]:.3f}") #

# Save to file
output_file = script_dir / "robot_coordinates.txt"
with open(output_file, "w") as f:
    for x, y in coordinates:
        f.write(f"{x:.3f}, {y:.3f}\n")

print(f"Robot coordinates saved to: {output_file}")

# Visualize the trajectory on the original image
output_img = cv2.cvtColor(binary_img, cv2.COLOR_GRAY2BGR)
for x_robot_mm, y_robot_mm in robot_coordinates_mm:
    x_pixel = int(x_robot_mm + center_x)
    y_pixel = int(center_y - y_robot_mm)
    cv2.circle(output_img, (x_pixel, y_pixel), 1, (0, 0, 255), -1) # red circles are for each point in trajectory

cv2.imshow("Trajectory Visualization", output_img)
cv2.waitKey(0)
cv2.destroyAllWindows()