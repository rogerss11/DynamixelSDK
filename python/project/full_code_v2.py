#!/usr/bin/env python3
import numpy as np
import sympy as sp
import time
import matplotlib.pyplot as plt
from dynamixel_sdk import *
from constants import STANDING_POS

# ============================================================
# Robot geometry
# ============================================================
#A1, A2, A3, A4, D1 = 0.0, 93.0, 93.0, 50.0, 50.0
A1, A2, A3, A4, D1 = 0.0, 93.0, 93.0, 50.0, 65.0
BASE_Z = 15

# ============================================================
# Dynamixel setup
# ============================================================
ADDR_MX_TORQUE_ENABLE   = 24
ADDR_MX_GOAL_POSITION   = 30
ADDR_MX_PRESENT_POSITION= 36
ADDR_MX_MOVING_SPEED    = 32

PROTOCOL_VERSION = 1.0
BAUDRATE = 1_000_000
DEVICENAME = "/dev/ttyACM0"

DXL_IDS = [1, 2, 3, 4]
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

v_ref = np.array(STANDING_POS, float)
s     = np.array([-1, -1, -1, -1], float)
scale = np.deg2rad(300.0) / 1024.0

def enc_to_q(values):
    q = s * (np.array(values) - v_ref) * scale
    return (q + np.pi) % (2*np.pi) - np.pi

def q_to_enc(q_rad):
    return v_ref + (q_rad / scale) * s

# ============================================================
# FK symbolic
# ============================================================
def DH(a, alpha, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

q1,q2,q3,q4 = sp.symbols("q1 q2 q3 q4")

_DH = [
    (0,    sp.pi/2, 50, q1),
    (93,   0,       0,  q2+sp.pi/2),
    (93,   0,       0,  q3),
    (50,   0,       0,  q4),
]

def forward_transforms():
    Ts=[sp.eye(4)]
    T=sp.eye(4)
    for p in _DH:
        T = T @ DH(*p)
        Ts.append(sp.simplify(T))
    return Ts

Ts = forward_transforms()
T04 = Ts[4]
T03 = Ts[3]

# Camera fixed in link 3 coordinates
T35 = sp.eye(4)
T35[0, 3] = 35
T35[1, 3] = 45
T35[2, 3] = 0
Tcamera = T03 @ T35

o4_func  = sp.lambdify((q1,q2,q3,q4), T04[0:3,3], 'numpy')
T04_func = sp.lambdify((q1,q2,q3,q4), T04, 'numpy')
Tcamera_func = sp.lambdify((q1,q2,q3,q4), Tcamera, 'numpy')

# ============================================================
# FK numeric
# ============================================================
def DH_numeric(a, alpha, d, theta):
    ct,st = np.cos(theta), np.sin(theta)
    ca,sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,       d],
        [0,        0,        0,       1]
    ])

def forward_points(q_vec):
    q1v,q2v,q3v,q4v = q_vec
    DHs = [
        (0,  np.pi/2, 50, q1v),
        (93, 0,       0,  q2v + np.pi/2),
        (93, 0,       0,  q3v),
        (50, 0,       0,  q4v),
    ]
    T = np.eye(4)
    pts=[T[:3,3]]
    for a,alpha,d,th in DHs:
        T = T @ DH_numeric(a,alpha,d,th)
        pts.append(T[:3,3])
    return np.stack(pts)

def forward_camera_point(q_vec):
    q1v,q2v,q3v,q4v = q_vec
    T = np.eye(4)
    DHs = [
        (0,  np.pi/2, 50, q1v),
        (93, 0,       0,  q2v + np.pi/2),
        (93, 0,       0,  q3v),
        (50, 0,       0,  q4v),
    ]
    for a,alpha,d,th in DHs:
        T = T @ DH_numeric(a,alpha,d,th)

    # camera offset from EE
    T45 = np.eye(4)
    T45[0, 3] = -40
    T45[1, 3] = -45
    T45[2, 3] = 0

    Tcam = T @ T45
    return Tcam

# ============================================================
# IK
# ============================================================
def ik_solver(x, y, z, *, beta=None, c=None, elbow_up=True):
    q1v = np.arctan2(y, x) + np.pi

    r = np.hypot(x, y)
    z1 = z - D1

    if beta is None:
        beta = 0 if c is None else np.arcsin(np.clip(c,-1,1))

    rw = r  - A4*np.cos(beta)
    zw = z1 - A4*np.sin(beta)

    c3 = (rw**2 + zw**2 - A2**2 - A3**2) / (2*A2*A3)
    c3 = np.clip(c3,-1,1)
    s3 = np.sqrt(1-c3**2)
    if not elbow_up:
        s3 = -s3
    q3v = np.arctan2(s3,c3)

    theta2p = np.arctan2(zw,rw) - np.arctan2(A3*np.sin(q3v), A2 + A3*np.cos(q3v))
    q2v = theta2p - np.pi/2

    q4v = beta - (theta2p + q3v)

    q_vec = np.array([q1v, -q2v, -q3v, -q4v])
    return (q_vec + np.pi) % (2*np.pi) - np.pi

# ============================================================
# Motor control
# ============================================================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def init_motors():
    if not portHandler.openPort(): raise IOError("Could not open port")
    if not portHandler.setBaudRate(BAUDRATE): raise IOError("Bad baudrate")
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_MOVING_SPEED, 50)
    print("Motors initialized.")

def read_joint_pos():
    vals=[]
    for dxl_id in DXL_IDS:
        pos,res,err = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
        vals.append(np.nan if res!=COMM_SUCCESS or err!=0 else pos)
    return np.array(vals,float)

def write_positions(enc_targets):
    for i,dxl_id in enumerate(DXL_IDS):
        packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, int(enc_targets[i]))

def close_port():
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()

# ============================================================
# High-level move + live plotting (WITH CAMERA PLOT)
# ============================================================

# ============================================================
# Camera Frustum Helpers
# ============================================================


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


def compute_frustum_corners_world(T_wc, K, W, H, s_near=80, s_far=250):
    """
    Computes the 3D frustum corners in world coordinates given:
    T_wc : 4x4 camera-to-world transform
    K    : intrinsic matrix
    W,H  : image size
    s_near, s_far : distances along each ray for near/far planes
    """
    C = T_wc[:3, 3]
    R = T_wc[:3, :3]
    Kinv = np.linalg.inv(K)

    corners_px = [
        (0,   0),     # top-left
        (W-1, 0),     # top-right
        (W-1, H-1),   # bottom-right
        (0,   H-1)    # bottom-left
    ]

    near_pts = []
    far_pts  = []

    for u, v in corners_px:
        d_c = Kinv @ np.array([u, v, 1.0], float)
        d_c /= np.linalg.norm(d_c)
        d_w = R @ d_c
        near_pts.append(C + s_near * d_w)
        far_pts.append(C + s_far  * d_w)

    return np.array(near_pts), np.array(far_pts)

def draw_frustum(ax, near_pts, far_pts, color='cyan'):
    """
    Draws a wireframe camera frustum.
    """
    # Near plane
    for i in range(4):
        j = (i+1) % 4
        ax.plot([near_pts[i,0], near_pts[j,0]],
                [near_pts[i,1], near_pts[j,1]],
                [near_pts[i,2], near_pts[j,2]], color=color, linewidth=1)

    # Far plane
    for i in range(4):
        j = (i+1) % 4
        ax.plot([far_pts[i,0], far_pts[j,0]],
                [far_pts[i,1], far_pts[j,1]],
                [far_pts[i,2], far_pts[j,2]], color=color, linewidth=1)

    # Connect near → far
    for i in range(4):
        ax.plot([near_pts[i,0], far_pts[i,0]],
                [near_pts[i,1], far_pts[i,1]],
                [near_pts[i,2], far_pts[i,2]], color=color, linewidth=1)

def apply_camera_pitch(R_cam, pitch_deg=-90):
    p = np.radians(pitch_deg)
    R_y = np.array([
        [ np.cos(p), 0, np.sin(p)],
        [ 0,         1,       0   ],
        [-np.sin(p), 0, np.cos(p)]
    ])
    return R_cam @ R_y

def go_to_xyz_live(x, y, z):
    print("\n=== Moving to:", (x, y, z), "===\n")

    # -----------------------------------------
    # Load camera intrinsics
    # -----------------------------------------
    import json
    with open("/home/silviu/Documents/Workspace/Robotics/Project/DynamixelSDK/python/project/vision/camera_intrinsics.json", "r") as f:
        intr = json.load(f)

    K = np.array(intr["camera_matrix"]["K"], float)
    W = intr["image_width"]
    H = intr["image_height"]

    # Will contain all matplotlib Line3D objects of the frustum so we can delete/replace them
    frustum_lines = []

    # -----------------------------------------
    # Get IK target and send robot
    # -----------------------------------------
    q_goal = ik_solver(x, y, z, beta=-np.pi/2, elbow_up=False)
    enc_goal = q_to_enc(q_goal)

    plt.ion()
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.set_zlim(-50, 250)
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")

    line_meas, = ax.plot([], [], [], 'o-', lw=2, color='blue', label='Measured')
    line_goal, = ax.plot([], [], [], 'o-', lw=2, color='red', label='IK Target')

    camera_point, = ax.plot([], [], [], 'o', color='green', markersize=8, label='Camera')
    camera_ray_line, = ax.plot([], [], [], '-', color='magenta', linewidth=2, label='Camera Ray')

    ax.legend()

    write_positions(enc_goal)

    # -----------------------------------------
    # Live loop
    # -----------------------------------------
    for _ in range(50):
        enc = read_joint_pos()
        if np.any(np.isnan(enc)):
            continue

        q_meas = enc_to_q(enc)
        pts_meas = forward_points(q_meas)
        pts_goal = forward_points(q_goal)

        # ------------------------------------------------------
        # Camera transform (YOUR FK)
        # ------------------------------------------------------
        Tcam = forward_camera_point(q_meas)

        # Apply camera pitch correction (-90 degrees)
        Rcorr = apply_camera_pitch(Tcam[:3, :3], 90)
        Tcam[:3, :3] = Rcorr

        cam_pos = Tcam[:3, 3]

        # -------------------------
        # Camera marker
        # -------------------------
        camera_point.set_data([cam_pos[0]], [cam_pos[1]])
        camera_point.set_3d_properties([cam_pos[2]])

        # -------------------------
        # Camera optical ray
        # -------------------------
        zaxis = Rcorr[:, 2]               # camera optical axis
        Pend = cam_pos + zaxis * 200      # 200mm forward

        camera_ray_line.set_data([cam_pos[0], Pend[0]],
                                 [cam_pos[1], Pend[1]])
        camera_ray_line.set_3d_properties([cam_pos[2], Pend[2]])

        # ------------------------------------------------------
        # Remove previous frustum drawing
        # ------------------------------------------------------
        for L in frustum_lines:
            L.remove()
        frustum_lines = []

        # ------------------------------------------------------
        # Compute frustum corners (near plane ~80mm, far ~250mm)
        # ------------------------------------------------------
        near_pts, far_pts = compute_frustum_corners_world(
            Tcam, K, W, H, s_near=80, s_far=250
        )

        def draw_segment(P, Q, color='cyan'):
            line, = ax.plot([P[0], Q[0]],
                            [P[1], Q[1]],
                            [P[2], Q[2]],
                            color=color, linewidth=1)
            frustum_lines.append(line)

        # Near plane (square)
        for i in range(4):
            draw_segment(near_pts[i], near_pts[(i+1)%4])

        # Far plane (square)
        for i in range(4):
            draw_segment(far_pts[i], far_pts[(i+1)%4])

        # Connect near–far
        for i in range(4):
            draw_segment(near_pts[i], far_pts[i])

        # -------------------------
        # Robot links
        # -------------------------
        line_meas.set_data(pts_meas[:, 0], pts_meas[:, 1])
        line_meas.set_3d_properties(pts_meas[:, 2])

        line_goal.set_data(pts_goal[:, 0], pts_goal[:, 1])
        line_goal.set_3d_properties(pts_goal[:, 2])

        ee = pts_meas[-1]
        ax.set_title(f"EE: X={ee[0]:.1f}  Y={ee[1]:.1f}  Z={ee[2]:.1f}")

        plt.pause(0.05)

    


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    try:
        init_motors()
        #X=-103.62, Y=-6.37, Z=124.42
        go_to_xyz_live(-251, 62, 0)
        time.sleep(5)
        #go_to_xyz_live(-90, 0, 0)
        #time.sleep(5)

    except KeyboardInterrupt:
        pass
    finally:
        close_port()
        print("Port closed.")
