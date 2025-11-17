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
    #return q
    return (q + np.pi) % (2*np.pi) - np.pi

def q_to_enc(q_rad):
    return v_ref + (q_rad / scale) * s

# ============================================================
# FK (symbolic → lambdified)
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

o4_func  = sp.lambdify((q1,q2,q3,q4), T04[0:3,3], 'numpy')
T04_func = sp.lambdify((q1,q2,q3,q4), T04, 'numpy')

# ============================================================
# Numeric forward kinematics for plotting
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

# ============================================================
# Full IK (from your implementation)
# ============================================================
def ik_solver(x, y, z, *, beta=None, c=None, elbow_up=True):
    """
    Solve IK consistent with DH:
    (0, π/2, d1, q1), (a2,0,0,q2+π/2), (a3,0,0,q3), (a4,0,0,q4)

    Inputs:
    x,y,z : target of frame {4} in base {0} [mm]
    beta  : desired tool angle θ234 in radians (in the r–z plane). If None, use c.
    c     : optional shorthand, beta = asin(c)  (i.e., c = sin(beta))
    elbow_up : choose elbow-up (True) or elbow-down (False) branch

    Returns:
    np.array([q1, q2, q3, q4]) in radians
    """

    # 1) Base yaw
    q1v = np.arctan2(y, x) + np.pi

    # 2) Reduce to the planar problem (r,z') seen from the shoulder
    r  = np.hypot(x, y)       # radius in XY
    z1 = z - D1               # remove base lift

    # 3) Desired tool orientation in the plane
    if beta is None:
        if c is None:
            beta = 0.0
        else:
            beta = np.arcsin(np.clip(c, -1.0, 1.0))

    # 4) Wrist position (origin of joint 4) by subtracting the tool
    rw = r  - A4*np.cos(beta)
    zw = z1 - A4*np.sin(beta)

    # 5) 2-link IK to the wrist for (a2, a3)
    c3 = (rw**2 + zw**2 - A2**2 - A3**2) / (2*A2*A3)
    c3 = np.clip(c3, -1.0, 1.0)

    s3 = np.sqrt(1.0 - c3**2)
    if not elbow_up:
        s3 = -s3
    q3v = np.arctan2(s3, c3)  # elbow branch

    theta2p = np.arctan2(zw, rw) - np.arctan2(A3*np.sin(q3v), A2 + A3*np.cos(q3v))
    # theta2p = q2 + π/2   ⇒   q2 = theta2p - π/2
    q2v = theta2p - np.pi/2

    # 6) Close the chain for q4:  beta = (q2+π/2) + q3 + q4
    q4v = beta - (theta2p + q3v)

    q_vec = np.array([q1v, -q2v, -q3v, -q4v], dtype=float)
    # wrap to [-pi, pi] for neatness
    q_vec = (q_vec + np.pi) % (2*np.pi) - np.pi
    return q_vec

# ============================================================
# Dynamixel control
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
# High-level move command + live plot
# ============================================================
def go_to_xyz_live(x,y,z):
    print("\n=== Moving to:", (x,y,z), "===\n")

    # Desired IK pose (red)
    q_goal = ik_solver(x,y,z, beta=-np.pi/2, elbow_up=False)
    enc_goal = q_to_enc(q_goal)

    # Prepare plotting
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
    line_goal, = ax.plot([],[],[],'o-',lw=2,color='red', label='IK Target')
    ax.legend()

    # Send command once
    write_positions(enc_goal)

    # Loop while robot moves
    for _ in range(50):   # adjust duration as needed

        enc = read_joint_pos()
        if np.any(np.isnan(enc)): continue
        q_meas = enc_to_q(enc)

        pts_meas = forward_points(q_meas)
        pts_goal = forward_points(q_goal)

        print(f"encoder goal: {enc_goal}")
        print(f"q goal: {enc_goal}")

        # update plot lines
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

        # ---------------------------------------------------
        # 🔥 Move + live visualization:
        # ---------------------------------------------------
        go_to_xyz_live(-110, 100, 12)
        time.sleep(1)
        go_to_xyz_live(-110, 0, 12)
        time.sleep(1)
        go_to_xyz_live(-110, -100, 12)


    except KeyboardInterrupt:
        pass
    finally:
        close_port()
        print("Port closed.")
