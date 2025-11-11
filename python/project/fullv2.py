import numpy as np
import sympy as sp
import time
from dynamixel_sdk import *  # Dynamixel SDK
from constants import STANDING_POS

# ============================================================
# Robot geometry (mm)
# ============================================================
A1, A2, A3, A4, D1 = 0.0, 93.0, 93.0, 50.0, 50.0

# ---------------- Dynamixel setup -------------------
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

# ---------------------------------------------------
# Encoder calibration
v_ref = np.array(STANDING_POS, dtype=float)
s     = np.array([1, 1, 1, 1], dtype=float)
scale = np.deg2rad(300.0) / 1024.0

def enc_to_q(values):
    q = s * (np.array(values, float) - v_ref) * scale
    #q = (q + np.pi) % (2*np.pi) - np.pi
    return q

def q_to_enc(q_rad):
    return v_ref + (q_rad / scale) * s

# ============================================================
# FK (same DH)
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
    (93, 0, 0, q2 + sp.pi/2),
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
o4_func  = sp.lambdify((q1,q2,q3,q4), T04[0:3,3], 'numpy')
T04_func = sp.lambdify((q1,q2,q3,q4), T04, modules='numpy')

# ============================================================
# IK (planar 3R + tool)
# ============================================================
def ik_solver(x, y, z, *, beta=None, c=None, elbow_up=True):
    q1 = np.arctan2(y, x)
    r  = np.hypot(x, y)
    z1 = z - D1

    if beta is None:
        if c is None:
            beta = 0.0
        else:
            beta = np.arcsin(np.clip(c, -1.0, 1.0))

    rw = r  - A4*np.cos(beta)
    zw = z1 - A4*np.sin(beta)

    c3 = (rw**2 + zw**2 - A2**2 - A3**2) / (2*A2*A3)
    c3 = np.clip(c3, -1.0, 1.0)
    s3 = np.sqrt(1.0 - c3**2)
    if not elbow_up:
        s3 = -s3
    q3 = np.arctan2(s3, c3)

    theta2p = np.arctan2(zw, rw) - np.arctan2(A3*np.sin(q3), A2 + A3*np.cos(q3))
    q2 = theta2p - np.pi/2
    q4 = beta - (theta2p + q3)

    return np.array([q1, q2, q3, q4], dtype=float)

def ik_down(X, Y, Z, elbow="up"):
    """
    IK for the 4-DOF arm with the tool x-axis pointing straight DOWN (−z0).
    Returns q = [q1, q2, q3, q4] in radians.
    elbow: "down" -> s3 >= 0, "up" -> s3 <= 0
    """
    # 1) base yaw
    q1 = np.arctan2(Y, X)

    # 2) wrist center (joint 3)
    Xw, Yw, Zw = X, Y, Z + A4

    # 3) planar target in {1}'s vertical plane
    r = np.hypot(Xw, Yw)
    s = Zw - D1

    # reachability
    L = np.hypot(r, s)
    if L > (A2 + A3) + 1e-9 or L < abs(A2 - A3) - 1e-9:
        raise ValueError("Target not reachable with tool pointing down.")

    # 4) elbow angle q3
    c3 = (r**2 + s**2 - A2**2 - A3**2) / (2*A2*A3)
    c3 = np.clip(c3, -1.0, 1.0)
    s3_mag = np.sqrt(max(0.0, 1.0 - c3**2))
    s3 = s3_mag if elbow == "down" else -s3_mag
    q3 = np.arctan2(s3, c3)

    # 5) shoulder q2
    q2 = np.arctan2(s, r) - np.arctan2(A3*s3, A2 + A3*c3)

    # 6) wrist q4 for "down" orientation
    q4 = -(q2 + q3)

    return np.array([q1, q2, q3, q4])


# ============================================================
# Helper functions
# ============================================================
def angle_wrap(a): return (a + np.pi) % (2*np.pi) - np.pi
def beta_from_q(q): return (q[1] + np.pi/2) + q[2] + q[3]

def choose_closest_branch(cands, q_seed):
    best = None; best_cost = 1e18
    for q in cands:
        dq = angle_wrap(q - q_seed)
        cost = np.sum(np.abs(dq))
        if cost < best_cost:
            best_cost, best = cost, q
    return best

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
        packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_MOVING_SPEED, 40)
    print("✅ Motors initialized, torque enabled, slow speed set.")

def read_joint_pos():
    positions = []
    for dxl_id in DXL_IDS:
        pos, rc, err = packetHandler.read2ByteTxRx(portHandler, dxl_id, ADDR_MX_PRESENT_POSITION)
        positions.append(pos if rc==COMM_SUCCESS and err==0 else np.nan)
    return np.array(positions, dtype=float)

def write_positions(enc_targets):
    enc_targets = enc_targets.astype(int)
    for i, dxl_id in enumerate(DXL_IDS):
        packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_MX_GOAL_POSITION, enc_targets[i])
    print("🦾 Sent encoder goals:", enc_targets)

def wait_until_reached(enc_targets, tol=5, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        cur = read_joint_pos()
        if np.all(np.abs(cur - enc_targets) < tol):
            return True
        time.sleep(0.05)
    return False

# ============================================================
# Move to (x,y,z,beta)
# ============================================================
def move_to_xyz(x, y, z, beta, elbow_up=True):
    raw = read_joint_pos()
    q_meas = enc_to_q(raw)
    # q_up = ik_solver(x, y, z, beta=beta, elbow_up=True)
    # q_down = ik_solver(x, y, z, beta=beta, elbow_up=False)
    q_up = ik_down(x, y, z, elbow="up")
    q_down = ik_down(x, y, z, elbow="down")
    q_target = choose_closest_branch(np.vstack([q_up, q_down]), q_meas)

    pos_fk = o4_func(*q_target).astype(float).ravel()
    print(f"\n🎯 Target (x,y,z,beta): ({x:.1f}, {y:.1f}, {z:.1f}, {np.degrees(beta):.1f}°)")
    print("Computed q [deg]:", np.round(np.degrees(q_target),2))
    print("FK check [mm]:", np.round(pos_fk,2))

    enc_targets = q_to_enc(q_target)
    write_positions(enc_targets)
    ok = wait_until_reached(enc_targets)
    print("✅ Reached target" if ok else "⚠️ Timeout, maybe still moving")

# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    try:
        init_motors()
        print("Starting movement demo...\n")

        # # Example 1: upright center, tool down
        # move_to_xyz(0, 0, 250, beta=-np.pi/2)

        # Example 2: reach forward-right
        move_to_xyz(-100, 0, 100, beta=-np.pi/2)

        # # Example 3: lower left
        # move_to_xyz(-100, 50, 100, beta=-np.pi/2)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # for dxl_id in DXL_IDS:
        #     packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        # portHandler.closePort()
        print("🔌 Port closed.")
