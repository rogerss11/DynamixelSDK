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
A1, A2, A3, A4, D1 = 0.0, 93.0, 93.0, 50.0, 50.0

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
    return Tcam[:3,3]

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
def go_to_xyz_live(x,y,z):
    print("\n=== Moving to:", (x,y,z), "===\n")

    q_goal = ik_solver(x,y,z, beta=-np.pi/2, elbow_up=False)
    enc_goal = q_to_enc(q_goal)

    plt.ion()
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111,projection='3d')
    ax.set_xlim(-200,200)
    ax.set_ylim(-200,200)
    ax.set_zlim(-50,250)
    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")

    line_meas, = ax.plot([],[],[],'o-',lw=2,color='blue',label='Measured')
    line_goal, = ax.plot([],[],[],'o-',lw=2,color='red',label='IK Target')

    # <<< ADDED: camera marker >>>
    camera_point, = ax.plot([], [], [], 'o', color='green', markersize=8, label='Camera')

    ax.legend()

    write_positions(enc_goal)

    for _ in range(50):
        enc = read_joint_pos()
        if np.any(np.isnan(enc)): continue
        q_meas = enc_to_q(enc)

        pts_meas = forward_points(q_meas)
        pts_goal = forward_points(q_goal)

        # <<< ADDED: update camera position >>>
        cam = forward_camera_point(q_meas)
        camera_point.set_data([cam[0]], [cam[1]])
        camera_point.set_3d_properties([cam[2]])

        line_meas.set_data(pts_meas[:,0], pts_meas[:,1])
        line_meas.set_3d_properties(pts_meas[:,2])

        line_goal.set_data(pts_goal[:,0], pts_goal[:,1])
        line_goal.set_3d_properties(pts_goal[:,2])

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
        go_to_xyz_live(-114.23, 1.17, 0)
        time.sleep(5)
        go_to_xyz_live(-114.23, 50.17, 112.25)
        time.sleep(5)

    except KeyboardInterrupt:
        pass
    finally:
        close_port()
        print("Port closed.")
