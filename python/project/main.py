#!/usr/bin/env python3
import time
import numpy as np
import cv2

# === Import robot tools ===
from full_code_v2 import (
    init_motors,
    close_port,
    read_joint_pos,
    enc_to_q,
    q_to_enc,
    go_to_xyz_live,
    forward_camera_point   # <--- USING THIS
)

# === Import vision tools ===
from vision.vision_full_tools import locate_hoop_world_xy

# --------------------------------------------------------------------
# Compute camera pose T_wc from *numeric* forward_camera_point
# --------------------------------------------------------------------
def get_camera_T_wc():
    enc = read_joint_pos()
    if np.any(np.isnan(enc)):
        raise RuntimeError("Could not read joint positions")

    q = enc_to_q(enc)

    # Camera world position
    C = forward_camera_point(q)

    # Only translation is used for z=0 projection
    T_wc = np.eye(4)
    T_wc[:3, 3] = C

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
        go_to_xyz_live(-114.23, 1.17, 112.25)
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
        go_to_xyz_live(Xw, Yw, Zw)

    except KeyboardInterrupt:
        pass

    finally:
        print("\n=== Closing port ===")
        close_port()
        cv2.destroyAllWindows()
