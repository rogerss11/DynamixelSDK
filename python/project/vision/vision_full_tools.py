# === Dependencies ===
import json
from pathlib import Path
import numpy as np
import cv2

# ----------------------------
# Robust hoop detector (contour + Hough fallback)
# ----------------------------
def hoop_centroid_px(img_bgr, *,
                     radius_range_px=None,
                     hough_dp=1.2,
                     hough_param1=120,
                     hough_param2=30):
    img = img_bgr
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Improve contrast & preserve edges
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    gray_eq = clahe.apply(gray)
    gray_blur = cv2.bilateralFilter(gray_eq, d=7, sigmaColor=50, sigmaSpace=7)

    # ---- Contour path ----
    g = cv2.GaussianBlur(gray_blur, (5,5), 0)
    v = np.median(g)
    low = int(max(0, 0.66*v)); high = int(min(255, 1.33*v))
    edges = cv2.Canny(g, low, high)
    edges = cv2.dilate(edges, np.ones((3,3), np.uint8), 1)
    edges = cv2.erode(edges,  np.ones((3,3), np.uint8), 1)

    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_score = None, -1.0
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 200:  # tune to your scale
            continue
        peri = cv2.arcLength(c, True)
        if peri == 0:
            continue
        circularity = 4*np.pi*area/(peri*peri)
        score = circularity * np.sqrt(max(area, 1.0))
        if score > best_score:
            best, best_score = c, score
    if best is not None:
        M = cv2.moments(best)
        if M["m00"] != 0:
            u = M["m10"]/M["m00"]; v = M["m01"]/M["m00"]
            return float(u), float(v)

    # ---- Hough fallback ----
    H, W = gray.shape
    minDist = max(20, min(H, W)//8)
    if radius_range_px is not None:
        minR, maxR = map(int, radius_range_px)
        maxR = max(maxR, minR+1)
    else:
        minR = int(0.03*min(H, W)); maxR = int(0.30*min(H, W))

    circles = cv2.HoughCircles(
        gray_blur, cv2.HOUGH_GRADIENT, dp=hough_dp, minDist=minDist,
        param1=hough_param1, param2=hough_param2,
        minRadius=minR, maxRadius=maxR
    )
    if circles is None: 
        return None
    circles = np.round(circles[0, :]).astype(int)

    # pick the circle with strongest edge support
    e2 = cv2.Canny(g, low, high)
    def edge_support(x, y, r):
        theta = np.linspace(0, 2*np.pi, 180, endpoint=False)
        xs = np.clip((x + r*np.cos(theta)).astype(int), 0, W-1)
        ys = np.clip((y + r*np.sin(theta)).astype(int), 0, H-1)
        return int(np.count_nonzero(e2[ys, xs]))

    best_c, best_sup = None, -1
    for (x, y, r) in circles:
        s = edge_support(x, y, r)
        if s > best_sup:
            best_sup, best_c = s, (x, y, r)
    if best_c is None:
        return None
    x, y, _ = best_c
    return float(x), float(y)

# ----------------------------
# Load K from JSON (your earlier format)
# ----------------------------
def load_intrinsics_from_json(json_path="camera_intrinsics.json"):
    jp = Path(json_path)
    if not jp.exists():
        raise FileNotFoundError(f"Intrinsics JSON not found: {jp}")
    with open(jp, "r") as f:
        cfg = json.load(f)

    cm = cfg.get("camera_matrix", {})
    K_list = cm.get("K")
    if K_list is not None:
        K = np.array(K_list, dtype=float).reshape(3,3)
    else:
        fx = float(cm["fx"]); fy = float(cm["fy"])
        cx = float(cm["cx"]); cy = float(cm["cy"])
        K = np.array([[fx, 0.0, cx],
                      [0.0, fy, cy],
                      [0.0, 0.0, 1.0]], float)

    # Distortion not used for ray-plane here, but load if you need undistortion
    dist = np.array(cfg.get("distortion", {}).get("coeffs", [0,0,0,0,0]), float).reshape(-1,1)
    W = int(cfg.get("image_width", 0)); H = int(cfg.get("image_height", 0))
    return K, dist, (W, H), cfg

# ----------------------------
# Pixel → plane (z=0) via ray–plane using K and T_wc
# ----------------------------
def pixel_to_plane_ray(u, v, K, T_wc):
    Kinv = np.linalg.inv(K)
    d_c = Kinv @ np.array([u, v, 1.0], float)
    d_c = d_c / np.linalg.norm(d_c)

    R_wc = T_wc[:3, :3]
    t_wc = T_wc[:3, 3]
    print("CAMERA POSITION WORLD:", t_wc)
    print("R_wc:", R_wc)
    C = t_wc                      # camera center in world
    d_w = R_wc @ d_c              # ray dir in world

    if abs(d_w[2]) < 1e-9:
        return None  # nearly parallel to plane
    s = -C[2] / d_w[2]            # intersect z=0
    P = C + s*d_w
    return float(P[0]), float(P[1])

# ----------------------------
# Main helper: from image + pose -> world (X,Y) of hoop on z=0
# ----------------------------
def locate_hoop_world_xy(image_path, T_wc, intrinsics_json="camera_intrinsics.json",
                         radius_hint_px=None):
    """
    Args:
      image_path: path to image file
      T_wc: 4x4 numpy array (world-from-camera pose)
      intrinsics_json: path to JSON storing K (and optional metadata)
      radius_hint_px: optional (minR, maxR) for Hough to stabilize detection

    Returns:
      dict with:
        'pixel_centroid': (u, v) or None
        'world_xy': (X, Y) or None
        'K': 3x3 numpy array
    """
    img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {image_path}")

    K, dist, (W, H), meta = load_intrinsics_from_json(intrinsics_json)

    img = cv2.undistort(img, K, dist)

    # If your stream is flipped/rotated by the driver, either disable it or
    # adjust K/image consistently. (Assuming raw frames here.)

    uv = hoop_centroid_px(img, radius_range_px=radius_hint_px)
    if uv is None:
        return {"pixel_centroid": None, "world_xy": None, "K": K}

    u, v = uv
    XY = pixel_to_plane_ray(u, v, K, T_wc)
    return {"pixel_centroid": (u, v), "world_xy": XY, "K": K}
