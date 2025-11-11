from dynamixel_sdk import *  # Uses Dynamixel SDK library
import numpy as np
import time

# ------------------- Configuration -------------------
ADDR_MX_PRESENT_POSITION = 36     # Address for current position (Protocol 1.0)
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyACM0"       # e.g. "COM3" on Windows
DXL_IDS = [1, 2, 3, 4]

# ------------------- Calibration (your reference pose) -------------------
v_ref = np.array([573, 513, 508, 814], dtype=float)  # encoder values at zero pose
s = np.array([-1, -1, -1, -1], dtype=float)          # direction of each joint
scale = np.deg2rad(300.0) / 1024                     # AX-12A: 0–1023 → 300°

def enc_to_deg(values):
    """Convert raw encoder values to joint angles in degrees."""
    q_rad = s * (np.array(values, float) - v_ref) * scale
    q_rad = (q_rad + np.pi) % (2 * np.pi) - np.pi   # wrap to [-pi, pi]
    return np.degrees(q_rad)

# ------------------- Init -------------------
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
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] Comm Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
            positions.append(np.nan)
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
            positions.append(np.nan)
        else:
            positions.append(pos)
    return np.array(positions, dtype=float)

# ------------------- Run -------------------
if __name__ == "__main__":
    init_port()
    try:
        while True:
            raw_positions = read_joint_pos()
            joint_angles_deg = enc_to_deg(raw_positions)
            print(f"Raw: {raw_positions.astype(int)}  →  Angles [deg]: {np.round(joint_angles_deg, 2)}")
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        portHandler.closePort()
        print("Port closed.")