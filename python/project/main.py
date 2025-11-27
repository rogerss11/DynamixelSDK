#!/usr/bin/env python3
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

# === Import robot tools ===
from full_code_v2 import (
    init_motors,
    close_port,
    read_joint_pos,
    enc_to_q,
    q_to_enc,
    go_to_xyz_live,
    forward_camera_point,   # <--- USING THIS
    forward_points
)

# === Import vision tools ===
from vision.vision_full_tools import locate_hoop_world_xy

# --------------------------------------------------------------------
# Compute camera pose T_wc from *numeric* forward_camera_point
# --------------------------------------------------------------------
def debug_T(T):
    R = T[:3,:3]
    t = T[:3,3]

    print("\n=== T_wc Debug ===")
    print("Translation (camera position world):", t)

    # Print rotation rows
    print("Rotation matrix R_wc:")
    print(R)

    # Check orthonormality
    print("R * R^T (should be identity):")
    print(R @ R.T)

    # Determinant should be +1
    print("det(R) =", np.linalg.det(R))

    # Convert rotation to Euler angles (optional)
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    if sy > 1e-6:
        rx = np.arctan2(R[2,1], R[2,2])
        ry = np.arctan2(-R[2,0], sy)
        rz = np.arctan2(R[1,0], R[0,0])
    else:
        # gimbal lock
        rx = np.arctan2(-R[1,2], R[1,1])
        ry = np.arctan2(-R[2,0], sy)
        rz = 0

    print("Euler angles (rad):", (rx, ry, rz))
    print("Euler angles (deg):", np.degrees([rx, ry, rz]))
    print("====================\n")

def full_scene_visualization(
    pts_robot,       # array from forward_points(q)
    T_wc,            # 4x4 camera pose in world coords
    K,               # 3x3 intrinsics
    W, H,            # image width/height
    pixel_uv=None,   # (u, v) from detection
    world_xy=None    # (X, Y) world intersection
):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # -----------------------------
    # 1. Robot links
    # -----------------------------
    ax.plot(pts_robot[:,0], pts_robot[:,1], pts_robot[:,2],
            'o-', lw=3, color='blue', label='Robot')

    # -----------------------------
    # 2. Camera pose
    # -----------------------------
    C = T_wc[:3,3]
    R = T_wc[:3,:3]

    ax.scatter(*C, s=80, color='magenta', label="Camera Center")

    # Camera axes
    axis_len = 80
    x_axis = R @ np.array([1,0,0]) * axis_len
    y_axis = R @ np.array([0,1,0]) * axis_len
    z_axis = R @ np.array([0,0,1]) * axis_len   # optical

    def draw_axis(vec, color, label):
        ax.quiver(C[0],C[1],C[2], vec[0],vec[1],vec[2],
                  color=color, lw=3, arrow_length_ratio=0.1)
        P = C + vec
        ax.text(P[0], P[1], P[2], label, color=color, fontsize=12)

    draw_axis(x_axis, 'red', "Cam X")
    draw_axis(y_axis, 'green', "Cam Y")
    draw_axis(z_axis, 'blue', "Cam Z(optical)")

    # -----------------------------
    # 3. Frustum
    # -----------------------------
    near_pts, far_pts = compute_frustum_corners_world(T_wc, K, W, H,
                                                      s_near=80, s_far=350)

    def seg(P, Q, col='cyan'):
        ax.plot([P[0],Q[0]],[P[1],Q[1]],[P[2],Q[2]],
                color=col, lw=1)

    # near plane
    for i in range(4): seg(near_pts[i], near_pts[(i+1)%4])
    # far plane
    for i in range(4): seg(far_pts[i], far_pts[(i+1)%4])
    # edges
    for i in range(4): seg(near_pts[i], far_pts[i])

    # -----------------------------
    # 4. Image polygon in world
    # -----------------------------
    # image corners projected using near-plane points
    img_poly = np.vstack([near_pts, near_pts[0]])
    ax.plot(img_poly[:,0], img_poly[:,1], img_poly[:,2],
            color='yellow', lw=2, label="Image plane (near)")

    # -----------------------------
    # 5. Pixel ray + detected point
    # -----------------------------
    if pixel_uv is not None:
        u, v = pixel_uv
        # Compute ray in world
        Kinv = np.linalg.inv(K)
        d_c = Kinv @ np.array([u, v, 1.0])
        d_c /= np.linalg.norm(d_c)
        d_w = R @ d_c

        ray_len = 350
        ray_end = C + d_w * ray_len

        # Ray
        ax.plot([C[0], ray_end[0]],
                [C[1], ray_end[1]],
                [C[2], ray_end[2]],
                color='magenta', lw=2, label="Ray through pixel")

        # Plot intersection with table
        if world_xy is not None:
            Xw, Yw = world_xy
            Zw = 0
            ax.scatter(Xw, Yw, Zw, color='red', s=80,
                       label=f"Detected world point ({Xw:.1f}, {Yw:.1f}, 0)")

    # -----------------------------
    # axes setup
    # -----------------------------
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.set_title("Full Robot + Camera + Image + Ray + Detection Visualization")

    ax.set_xlim(C[0]-200, C[0]+200)
    ax.set_ylim(C[1]-200, C[1]+200)
    ax.set_zlim(-50, 300)

    ax.legend()
    plt.tight_layout()
    plt.show()


def apply_camera_pitch(R_cam, pitch_deg=-90):
    p = np.radians(pitch_deg)
    R_y = np.array([
        [ np.cos(p), 0, np.sin(p)],
        [ 0,         1,       0   ],
        [-np.sin(p), 0, np.cos(p)]
    ])
    return R_cam @ R_y

def compute_frustum_corners_world(T_wc, K, W, H, s_near=80, s_far=350):

    C = T_wc[:3,3]
    R = T_wc[:3,:3]
    Kinv = np.linalg.inv(K)

    # Pixels of image corners
    corners_px = [
        (0,   0),
        (W-1, 0),
        (W-1, H-1),
        (0,   H-1)
    ]

    near_pts = []
    far_pts  = []

    for u,v in corners_px:
        d_c = Kinv @ np.array([u,v,1.0], float)
        d_c /= np.linalg.norm(d_c)
        d_w = R @ d_c
        near_pts.append(C + s_near * d_w)
        far_pts.append(C + s_far  * d_w)

    return np.array(near_pts), np.array(far_pts)


# ============================================================
# Helper: draw frustum
# ============================================================
def draw_frustum(ax, near_pts, far_pts, color='cyan'):
    def seg(P, Q):
        ax.plot([P[0], Q[0]], [P[1], Q[1]], [P[2], Q[2]],
                color=color, linewidth=1)

    # near plane
    for i in range(4):
        seg(near_pts[i], near_pts[(i+1)%4])

    # far plane
    for i in range(4):
        seg(far_pts[i], far_pts[(i+1)%4])

    # near-far edges
    for i in range(4):
        seg(near_pts[i], far_pts[i])


# ============================================================
# THE METHOD YOU WANT:
# plot_camera_pose(T_wc, K, W, H)
# ============================================================
def plot_camera_pose(T_wc, K, W, H, axis_len=120):
    """
    Plots:
      - camera world position
      - camera orientation axes (X=red, Y=green, Z=blue)
      - camera frustum based on intrinsics
    """

    C = T_wc[:3,3]
    R = T_wc[:3,:3]

    # Camera orientation axes
    x_axis = R @ np.array([1,0,0])
    y_axis = R @ np.array([0,1,0])
    z_axis = R @ np.array([0,0,1])  # Optical axis

    # End points
    Xend = C + x_axis * axis_len
    Yend = C + y_axis * axis_len
    Zend = C + z_axis * axis_len

    # Build figure
    fig = plt.figure(figsize=(9,8))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(*C, s=80, color='magenta', label="Camera Position")

    # Orientation axes
    ax.quiver(C[0], C[1], C[2],
              x_axis[0], x_axis[1], x_axis[2],
              color='red', linewidth=3, arrow_length_ratio=0.1)
    ax.text(*Xend, "X", color='red')

    ax.quiver(C[0], C[1], C[2],
              y_axis[0], y_axis[1], y_axis[2],
              color='green', linewidth=3, arrow_length_ratio=0.1)
    ax.text(*Yend, "Y", color='green')

    ax.quiver(C[0], C[1], C[2],
              z_axis[0], z_axis[1], z_axis[2],
              color='blue', linewidth=3, arrow_length_ratio=0.1)
    ax.text(*Zend, "Z (optical)", color='blue')

    # Camera frustum
    near_pts, far_pts = compute_frustum_corners_world(T_wc, K, W, H)
    draw_frustum(ax, near_pts, far_pts, color='cyan')

    # Axes limits
    ax.set_xlim(C[0]-200, C[0]+200)
    ax.set_ylim(C[1]-200, C[1]+200)
    ax.set_zlim(C[2]-200, C[2]+200)

    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.set_title("Camera Pose in World Frame")

    ax.legend()
    plt.tight_layout()
    plt.show()


def get_camera_T_wc():
    enc = read_joint_pos()
    if np.any(np.isnan(enc)):
        raise RuntimeError("Could not read joint positions")

    q = enc_to_q(enc)

    # Camera world position
    T_wc = forward_camera_point(q)
    Rcorr = apply_camera_pitch(T_wc[:3,:3], 90)
    T_wc[:3,:3] = Rcorr

    # Load intrinsics
    import json
    intr = json.load(open("/home/silviu/Documents/Workspace/Robotics/Project/DynamixelSDK/python/project/vision/camera_intrinsics.json"))
    K = np.array(intr["camera_matrix"]["K"])
    W = intr["image_width"]
    H = intr["image_height"]

    plot_camera_pose(T_wc, K, W, H)

    # # Only translation is used for z=0 projection
    # T_wc = np.eye(4)
    # T_wc[:3, 3] = C

    return T_wc


# --------------------------------------------------------------------
# Capture a single webcam image
# --------------------------------------------------------------------
def capture_image(output_path="capture.png", cam_index=2):
    cap = cv2.VideoCapture(cam_index)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    time.sleep(0.5)
    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("Could not capture frame")

    # === Show captured image ===
    cv2.imshow("Captured Image", frame)
    cv2.waitKey(1)

    cv2.imwrite(output_path, frame)
    return output_path, frame


# --------------------------------------------------------------------
# MAIN PIPELINE
# --------------------------------------------------------------------
if __name__ == "__main__":
    try:
        print("\n=== Initializing motors ===")
        init_motors()

        # -------------------------------------------------------------
        # 1. Move robot to fixed viewing pose
        # -------------------------------------------------------------
        print("\n=== Moving robot to observation pose ===")


        observation_pose = np.array([511., 549., 303., 157.])

        from full_code_v2 import write_positions

        # Convert to int
        enc_int = observation_pose.astype(int)

        write_positions(enc_int)

        # Small wait to allow motors to reach the target
        time.sleep(2)

        # -------------------------------------------------------------
        # 2. Capture image
        # -------------------------------------------------------------
        print("\n=== Capturing webcam image ===")
        img_path, frame = capture_image("snapshot.png")

        # -------------------------------------------------------------
        # 3. Compute camera pose T_wc (translation only)
        # -------------------------------------------------------------
        print("\n=== Computing numeric camera pose ===")
        T_wc = get_camera_T_wc()
        debug_T(T_wc)

        # -------------------------------------------------------------
        # 4. Detect hoop + convert to world coordinates
        # -------------------------------------------------------------
        print("\n=== Detecting circle and projecting to world ===")
        result = locate_hoop_world_xy(
            image_path=img_path,
            T_wc=T_wc,
            intrinsics_json="/home/silviu/Documents/Workspace/Robotics/Project/DynamixelSDK/python/project/vision/camera_intrinsics.json"
        )

        uv = result["pixel_centroid"]
        XY = result["world_xy"]

        print("Pixel coordinates:", uv)
        print("World XY:", XY)

        # Get robot pose for plotting
        enc = read_joint_pos()
        q = enc_to_q(enc)
        pts_meas = forward_points(q)

        import json

        # Load intrinsics
        intr = json.load(open("/home/silviu/Documents/Workspace/Robotics/Project/DynamixelSDK/python/project/vision/camera_intrinsics.json"))
        K = np.array(intr["camera_matrix"]["K"])
        W = intr["image_width"]
        H = intr["image_height"]

        # Plot full scene
        full_scene_visualization(
            pts_robot=pts_meas,
            T_wc=T_wc,
            K=K,
            W=W,
            H=H,
            pixel_uv=uv,
            world_xy=XY
        )

        # -------------------------------------------------------------
        # 5. Draw detection result on the image
        # -------------------------------------------------------------
        if uv is not None:
            u, v = uv
            display = frame.copy()
            cv2.circle(display, (int(u), int(v)), 8, (0, 0, 255), -1)
            cv2.putText(display, f"({int(u)}, {int(v)})", (int(u)+10, int(v)+10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
            cv2.imshow("Detected Hoop", display)
            cv2.waitKey(0)  # wait so you can see the detection
        else:
            print("No hoop detected — aborting.")
            cv2.imshow("Detected Hoop", frame)
            cv2.waitKey(0)  # wait so you can see the detection
            raise SystemExit

        if XY is None:
            print("Projection failed — aborting.")
            raise SystemExit

        Xw, Yw = XY
        Zw = 0.0  # your plane

        # -------------------------------------------------------------
        # 6. Move robot to detected coordinates
        # -------------------------------------------------------------
        print("\n=== Moving robot to target XY ===")
        go_to_xyz_live(Xw, -Yw, Zw)
        time.sleep(5)

    except KeyboardInterrupt:
        pass

    finally:
        print("\n=== Closing port ===")
        close_port()
        cv2.destroyAllWindows()
