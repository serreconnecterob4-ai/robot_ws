import cv2
import numpy as np

angle = 76.681
px_original = 26.6833333333
py_original = 184.133333333
origin_width = 9966
mini_width = 1661
px_mini = px_original * mini_width / origin_width
py_mini = py_original * mini_width / origin_width

def rotate_image(input_path, output_path, angle):
    # Charger l'image
    img = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)

    h, w = img.shape[:2]
    center = (w/2, h/2)

    # matrice de rotation
    M = cv2.getRotationMatrix2D(center, angle, 1.0)

    # calcul nouvelle taille pour éviter de couper l'image
    cos = abs(M[0,0])
    sin = abs(M[0,1])

    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    # ajuster la translation
    M[0,2] += new_w/2 - center[0]
    M[1,2] += new_h/2 - center[1]

    rotated = cv2.warpAffine(img, M, (new_w, new_h))

    cv2.imwrite(output_path, rotated)

    return M

def transform_point(M, x, y):
    """Applique la matrice de rotation M à un point (x, y)."""
    point = np.array([x, y, 1.0])
    transformed = M @ point
    return transformed[0], transformed[1]

# rotation
M = rotate_image("map.pgm", "map_rotated.pgm", angle)

# coordonnées du point original
new_px, new_py = transform_point(M, px, py)
print(f"Point original     : x = {px:.6f} px, y = {py:.6f} px")
print(f"Point après rotation: x = {new_px:.6f} px, y = {new_py:.6f} px")