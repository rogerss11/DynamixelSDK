from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ------------------- Configuration -------------------
ADDR_MX_PRESENT_POSITION = 36     # Address for current position (Protocol 1.0)
PROTOCOL_VERSION = 1.0
BAUDRATE = 1000000
DEVICENAME = "COM3"       # Change if needed (e.g., "COM3" on Windows)
DXL_IDS = [1, 2, 3, 4]

# ------------------- Initialization -------------------
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def init_port():
    """Open port and set baudrate (safe to call once at the start)."""
    if not portHandler.openPort():
        raise IOError("Failed to open port")
    if not portHandler.setBaudRate(BAUDRATE):
        raise IOError("Failed to set baudrate")
    print("Port opened and baudrate set.")

# ------------------- Read Function -------------------
def read_joint_pos():
    """
    Reads and returns the current joint positions of all 4 Dynamixel motors.
    Returns a list [pos1, pos2, pos3, pos4] in Dynamixel units.
    """
    positions = []
    for dxl_id in DXL_IDS:
        pos, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
            portHandler, dxl_id, ADDR_MX_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(f"[ID:{dxl_id}] Comm Error: {packetHandler.getTxRxResult(dxl_comm_result)}")
            positions.append(None)
        elif dxl_error != 0:
            print(f"[ID:{dxl_id}] Packet Error: {packetHandler.getRxPacketError(dxl_error)}")
            positions.append(None)
        else:
            positions.append(pos)

    return positions

# ------------------- Example Usage -------------------
if __name__ == "__main__":
    # Make it read positions continuously for demonstration
    init_port()
    try:
        while True:
            joint_positions = read_joint_pos()
            print("Current Joint Positions:", joint_positions)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        portHandler.closePort()