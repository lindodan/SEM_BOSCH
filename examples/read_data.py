import cv2
import numpy as np
from pathlib import Path
import random

# Path to the drawings directory
script_dir = Path(__file__).parent.parent / 'drawings'
print(script_dir)

# Read data from file
img = cv2.imread(str(script_dir / "A-1.png"))

# Convert to grayscale
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Edge detection using the Canny detector
edges = cv2.Canny(img_gray, 50, 150)
cv2.imshow("edges",edges)
cv2.imshow("img",img)
cv2.waitKey(0)
cv2.destroyAllWindows()
# Find contours (detect lines)
contours, _ = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Initialize the list for robot coordinates in mm
robot_coordinates_mm = []
center_x = 500
center_y = 500

# Process each contour and convert to robot coordinates
for contour in contours:
    for point in contour:
        x, y = point[0]  # Pixel x, y coordinates

        # Convert to robot coordinates in mm
        x_robot_mm = x - center_x  # Robot's x coordinate in mm
        y_robot_mm = center_y - y  # Robot's y coordinate in mm

        # Save coordinates
        robot_coordinates_mm.append((x_robot_mm, y_robot_mm))

# Convert robot coordinates to meters
robot_coordinates_m = [(x / 1000, y / 1000) for x, y in robot_coordinates_mm]

# Save coordinates to a file
output_file = script_dir / "robot_coordinates.txt"
with open(output_file, "w") as f:
    for x, y in robot_coordinates_m:
        f.write(f"{x:.3f}, {y:.3f}\n")

print(f"Robot coordinates saved to {output_file}")

# If you want to visualize the coordinates back on the image:
img_robot = img.copy()
for (x_robot_mm, y_robot_mm) in robot_coordinates_mm:
    # Convert back to pixel coordinates for visualization
    x_pixel = int(x_robot_mm + center_x)
    y_pixel = int(center_y - y_robot_mm)
    cv2.circle(img_robot, (x_pixel, y_pixel), 1, (0, 255, 0), 1)

cv2.imshow("Robot Coordinates", img_robot)
cv2.waitKey(0)
cv2.destroyAllWindows()


"""
# Inicializace obrázku pro vykreslení kontur
img_contours = img.copy()

# Funkce pro generování náhodné barvy
def random_color():
    return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))


# Vykreslení každé kontury jinou barvou
for contour in contours:
    # Generování náhodné barvy
    color = random_color()

    # Kreslení kontury na obrázek
    cv2.drawContours(img_contours, [contour], -1, color, 2)  # -1 znamená vykreslení všech kontur, tloušťka 2

# Zobrazení výsledného obrázku
cv2.imshow("Kontury s náhodnými barvami", img_contours)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""