import numpy as np
import sympy as sp
import time
from dynamixel_sdk import *           # Dynamixel SDK
from constants import STANDING_POS     # your 4 encoder counts in the standing pose

# ============================================================
# Robot geometry (mm)  — matches your current FK & IK
# ============================================================
A1, A2, A3, A4, D1 = 0.0, 93.0, 93.0, 50.0, 50.0  # A4 = last link length (planar)
# NOTE: This FK uses A4 as a *planar* link, consistent with your ik_solver.
# If you later switch to DH with d4=50 instead, update ik_solver accordingly.

# ---------------- Dynamixel & calibration -------------------
ADDR_MX_TORQUE_ENABLE   = 24
ADDR_MX_GOAL_POSITION   = 30
ADDR_MX_PRESENT_POSITION= 36
ADDR_MX_MOVING_SPEED = 32

PROTOCOL_VERSION = 1.0
BAUDRATE = 1_000_000
DEVICENAME = "/dev/ttyACM0"
DXL_IDS = [1, 2, 3, 4]
TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

# Standing pose (encoder counts) and mapping
v_ref = np.array(STANDING_POS, dtype=float)     # your chosen zero
s     = np.array([-1, -1, -1, -1], dtype=float) # your signs
scale = np.deg2rad(300.0) / 1024.0              # AX/10-bit @ 300° travel

def enc_to_q(values):
    """encoder -> radians (relative to standing pose)"""
    q = s * (np.array(values, float) - v_ref) * scale
    # optional wrap
    q = (q + np.pi) % (2*np.pi) - np.pi
    return q

def q_to_enc(q_rad):
    """radians -> encoder counts (relative to standing pose)"""
    return v_ref + (q_rad / scale) * s

# ---------------- Joint limits (in encoder counts) ----------
JOINT_LIMITS = {
    1: (40, 900),
    2: (200, 900),
    3: (45, 950),
    4: (155, 800),
}

def clamp_to_limits(enc):
    enc = np.array(enc, float)
    for i, dxl_id in enumerate(DXL_IDS):
        lo, hi = JOINT_LIMITS[dxl_id]
        enc[i] = np.clip(enc[i], lo, hi)
    return enc.astype(int)

# ============================================================
# FK with standard DH (A4 planar, as in your code)
# ============================================================
def DH(a, alpha, d, theta):
    return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta)*sp.cos(alpha),  sp.sin(theta)*sp.sin(alpha), a*sp.cos(theta)],
        [sp.sin(theta),  sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), a*sp.sin(theta)],
        [0,              sp.sin(alpha),                sp.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

q1, q2, q3, q4 = sp.symbols("q1 q2 q3 q4")
_DH = [
    (0, sp.pi/2, 50, q1),
    (93, 0, 0, q2+sp.pi/2),
    (93, 0, 0, q3),
    (50, 0, 0, q4)
]

def forward_transforms():
    Ts, T = [sp.eye(4)], sp.eye(4)
    for p in _DH:
        T = T @ DH(*p)
        Ts.append(sp.simplify(T))
    return Ts

Ts  = forward_transforms()
T04 = Ts[4]
# Fast evaluators
o4_func  = sp.lambdify((q1,q2,q3,q4), T04[0:3, 3], 'numpy')
J4       = sp.simplify( sp.Matrix.vstack(
             sp.Matrix.hstack(*[sp.Matrix.cross(Ts[i-1][0:3,2], T04[0:3,3]-Ts[i-1][0:3,3]) for i in range(1,5)]),
             sp.Matrix.hstack(*[Ts[i-1][0:3,2] for i in range(1,5)])
           ))
J_func   = sp.lambdify((q1,q2,q3,q4), J4, 'numpy')

# ============================================================
# IK (your planar 3R + tool)  — input c in [-1,1] with beta=arcsin(c)
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
    q1 = np.arctan2(y, x)

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
    q3 = np.arctan2(s3, c3)  # elbow branch

    theta2p = np.arctan2(zw, rw) - np.arctan2(A3*np.sin(q3), A2 + A3*np.cos(q3))
    # theta2p = q2 + π/2   ⇒   q2 = theta2p - π/2
    q2 = theta2p - np.pi/2

    # 6) Close the chain for q4:  beta = (q2+π/2) + q3 + q4
    q4 = beta - (theta2p + q3)

    return np.array([q1, q2, q3, q4], dtype=float)

# ============================================================
# Dynamixel I/O
# ============================================================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def init_motors():
    if not portHandler.openPort():
        raise IOError("Failed to open port")
    if not portHandler.setBaudRate(BAUDRATE):
        raise IOError("Failed to set baudrate")
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        # ⚠️ lower value = slower
        packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_MOVING_SPEED, 50)
    print("✅ Motors initialized, torque enabled, and speed set.")

def write_positions(enc_targets):
    print("targers: ")
    print(enc_targets)
    #enc_targets = clamp_to_limits(enc_targets)
    for i, dxl_id in enumerate(DXL_IDS):
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, dxl_id, ADDR_MX_GOAL_POSITION, int(enc_targets[i])
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] Comm Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"[ID:{dxl_id}] → {int(enc_targets[i])}")

def close_port():
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("🔌 Port closed.")

# ============================================================
# High-level: go to Cartesian (x,y,z) with pitch parameter c
# ============================================================
def go_to_xyz(x_mm, y_mm, z_mm, c=1.0):
    """
    c in [-1,1], where beta = asin(c) is the end-effector pitch used in your IK.
    Example: c=0 → beta=0 (tool aligned with the arm); c=1 → +90°.
    """
    # IK
    #q_rad = ik_solver(x_mm, y_mm, z_mm, c)
    target = (130.0, 50.0, 50.0)
    beta=-np.pi/2
    q_rad = ik_solver(*target, beta=beta, elbow_up=False)
    pos_fk = o4_func(*q_rad).astype(float).ravel()
    print("q [deg]:", np.round(np.degrees(q_rad),2))
    print("FK:", np.round(pos_fk,2), "mm")

    # Convert to encoders and enforce limits
    enc_targets = q_to_enc(q_rad)
    #enc_targets = clamp_to_limits(enc_targets)

    # Print FK check for sanity
    xyz_fk = o4_func(*q_rad).astype(float).ravel()
    print(f"IK q [deg]: {np.round(np.degrees(q_rad),2)}")
    print(f"FK(x,y,z):  {np.round(xyz_fk,2)} mm")

    # Move
    write_positions(enc_targets)

# ============================================================
# Example usage
# ============================================================
if __name__ == "__main__":
    try:
        init_motors()

        # Example: move to the upright pose tip (roughly center), beta=0
        # Expect something near (x≈0, y≈0, z≈286) *if* all qs≈0 in your calibration
        go_to_xyz(30, 50, 150.0, c=0.0)

        # Another point (change as you like)
        # go_to_xyz(120.0, 0.0, 180.0, c=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        #close_port()
        print("pual")
