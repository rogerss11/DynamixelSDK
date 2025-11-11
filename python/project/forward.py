import numpy as np
import sympy as sp
import time
from dynamixel_sdk import *  # Dynamixel SDK
from constants import STANDING_POS

# ============================================================
# CONSTANTS (mm, rad, s)
# ============================================================
A1, A2, A3, A4, D1 = 0.0, 93.0, 93.0, 50.0, 50.0

# ============================================================
# DH TABLE AND FORWARD KINEMATICS (Symbolic)
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
        T = T * DH(*p)
        Ts.append(sp.simplify(T))
    return Ts

Ts = forward_transforms()
T04 = Ts[4]
T04_func = sp.lambdify((q1, q2, q3, q4), T04, modules='numpy')

# ============================================================
# DYNAMIXEL CONFIGURATION
# ============================================================
ADDR_MX_PRESENT_POSITION = 36
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyACM0"
DXL_IDS = [1, 2, 3, 4]

# Calibration (your zero reference and directions)
v_ref = np.array(STANDING_POS, dtype=float)
s = np.array([-1, -1, -1, -1], dtype=float)
scale = np.deg2rad(300.0) / 1024   # 10-bit AX-12 range

def enc_to_q(values):
    q = s * (np.array(values, float) - v_ref) * scale
    q = (q + np.pi) % (2 * np.pi) - np.pi  # wrap [-pi, pi]
    return q

# ============================================================
# DYNAMIXEL READ FUNCTIONS
# ============================================================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def init_port():
    if not portHandler.openPort():
        raise IOError("Failed to open port")
    if not portHandler.setBaudRate(BAUDRATE):
        raise IOError("Failed to set baudrate")
    print("✅ Port opened and baudrate set.")

def read_joint_pos():
    positions = []
    for dxl_id in DXL_IDS:
        pos, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
            portHandler, dxl_id, ADDR_MX_PRESENT_POSITION
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            positions.append(np.nan)
        else:
            positions.append(pos)
    return np.array(positions, dtype=float)

# ============================================================
# MAIN LOOP
# ============================================================
if __name__ == "__main__":
    init_port()
    print("Streaming end-effector position... (Ctrl+C to stop)")
    try:
        while True:
            raw_positions = read_joint_pos()
            if np.any(np.isnan(raw_positions)):
                print("⚠️ Read error on some joints.")
                continue

            q_rad = enc_to_q(raw_positions)
            T = np.array(T04_func(*q_rad), dtype=float)
            pos = T[0:3, 3]
            print(f"Raw positions: {raw_positions}")
            print(f"Angles [deg]: {np.round(np.degrees(q_rad), 2)}")
            print(f"End-Effector Position [mm]: X={pos[0]:.2f}, Y={pos[1]:.2f}, Z={pos[2]:.2f}\n")
            time.sleep(0.3)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        portHandler.closePort()
        print("Port closed.")