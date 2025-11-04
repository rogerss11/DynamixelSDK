from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ------------------- Configuration -------------------
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_POSITION = 30
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = "COM3"   # Change to your port (e.g., "COM3" on Windows)
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# IDs of your 4 Dynamixels
DXL_IDS = [1, 2, 3, 4]

# ------------------- Initialization -------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def init_motors():
    """Open port, set baudrate, and enable torque for all motors."""
    if not portHandler.openPort():
        raise IOError("Failed to open port")

    if not portHandler.setBaudRate(BAUDRATE):
        raise IOError("Failed to set baudrate")

    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    print("Motors initialized and torque enabled.")

# ------------------- Move Function -------------------
def move_robot(q):
    """
    Move 4 Dynamixel motors to desired joint positions.
    q : list or array [q1, q2, q3, q4] (goal positions in Dynamixel units)
    """
    if len(q) != len(DXL_IDS):
        raise ValueError(f"Expected {len(DXL_IDS)} joint positions, got {len(q)}")

    for i, dxl_id in enumerate(DXL_IDS):
        goal_pos = int(q[i])
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
            portHandler, dxl_id, ADDR_MX_GOAL_POSITION, goal_pos
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] Comm Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"[ID:{dxl_id}] Moved to position {goal_pos}")

# ------------------- Example Usage -------------------
if __name__ == "__main__":
    # Joint limits
    JOINT_LIMITS = {
        1: (40, 900),
        2: (200, 900),
        3: (45, 950),
        4: (155, 800),
    }

    # Standing position
    STANDING_POS = [867, 503, 499, 486]

    init_motors()

    # Example target configuration
    q_target = [867, 503, 499, 486]
    move_robot(q_target)