import cv2
import numpy as np
from pathlib import Path
import random

script_dir = Path(__file__).parent.parent /'drawings'
print(script_dir)
# Read data from file
img = cv2.imread(script_dir/"A-1.png")

# Převedení na šedou škálu
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Detekce hran pomocí Cannyho detektoru (pokud chcete detekovat okraje čáry)
edges = cv2.Canny(img_gray, 50, 150)

# Detekce kontur (kde se nachází čára)
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# Inicializace seznamu souřadnic v souřadném systému robota
robot_coordinates = []
center_x = 500
center_y = 500

# Procházení všech kontur a převod na souřadnice robota
for contour in contours:
    for point in contour:
        x, y = point[0]  # x, y koordináty pixelu

        # Převod na souřadnice robota
        x_robot = x - center_x  # x-ová souřadnice robota
        y_robot = center_y - y  # y-ová souřadnice robota

        # Uložení souřadnic
        robot_coordinates.append((x_robot, y_robot))

# Pokud chcete souřadnice vykreslit zpět na obrázek, můžete použít:
img_robot = img.copy()
for (x_robot, y_robot) in robot_coordinates:
    cv2.circle(img_robot, (x_robot + center_x, center_y - y_robot), 1, (0, 255, 0), 1)

cv2.imshow("Souřadnice v souřadném systému robota", img_robot)
cv2.waitKey(0)
cv2.destroyAllWindows()

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
