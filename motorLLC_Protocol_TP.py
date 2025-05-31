from dynamixel_sdk import *                 #use dynamixel sdk library

#control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_GOAL_VELOCITY      = 32
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_PRESENT_SPEED = 38   # Present Speed (Velocity) 레지스터 주소

# Data Byte Length
LEN_MX_GOAL_POSITION       = 2
LEN_MX_GOAL_VEL            = 2
LEN_MX_PRESENT_POSITION    = 2
LEN_MX_PRESENT_SPEED       = 2

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

#Default setting
DXL1_ID = 1
DXL2_ID = 2
DXL3_ID = 3
DXL4_ID = 4
DXL5_ID = 5
DXL6_ID = 6
DXL7_ID = 7
BAUDRATE                    = 1000000            # Dynamixel default baudrate : 57600
DEVICENAME                  = "COM6"            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque


class motorLLC():
    def __init__(self):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION + LEN_MX_GOAL_VEL)

        # List of motor IDs for bulk operations
        self.ids = [
            DXL1_ID, DXL2_ID, DXL3_ID,
            DXL4_ID, DXL5_ID, DXL6_ID,
            DXL7_ID
        ]
        # Baudrate for opening port
        self.baudrate = BAUDRATE

    def open(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()

    def torque_enable(self):
        # Enable torque on all motors
        for dxl_id in self.ids:
            res, err = self.packetHandler.write1ByteTxRx(
                self.portHandler,
                dxl_id,
                ADDR_MX_TORQUE_ENABLE,
                TORQUE_ENABLE
            )
            if res != COMM_SUCCESS:
                print(f"[torque_enable][ID {dxl_id}] TxRxResult: {self.packetHandler.getTxRxResult(res)}")
            elif err:
                print(f"[torque_enable][ID {dxl_id}] RxPacketError: {self.packetHandler.getRxPacketError(err)}")
            else:
                print(f"[torque_enable][ID {dxl_id}] torque enabled")


    def moveTo(self, positions, velocities):
        # 인자 길이 확인
        if len(positions) != len(self.ids) or len(velocities) != len(self.ids):
            print("목표 위치/속도 리스트 길이가 모터 수와 다릅니다")
            return

        # SyncWrite 파라미터 추가 (pos LSB, pos MSB, vel LSB, vel MSB)
        for dxl_id, pos, vel in zip(self.ids, positions, velocities):
            goal_param = [
                DXL_LOBYTE(pos), DXL_HIBYTE(pos),
                DXL_LOBYTE(vel), DXL_HIBYTE(vel)
            ]
            ok = self.groupSyncWrite.addParam(dxl_id, goal_param)
            if not ok:
                print(f"[ID:{dxl_id:03d}] groupSyncWrite addParam failed")
                return

        # 패킷 전송
        res = self.groupSyncWrite.txPacket()
        if res != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(res))

        # 버퍼 클리어
        self.groupSyncWrite.clearParam()


    def readPos(self):
        # Read present position from all motors
        positions = []
        for dxl_id in self.ids:
            pos, res, err = self.packetHandler.read2ByteTxRx(
                self.portHandler,
                dxl_id,
                ADDR_MX_PRESENT_POSITION
            )
            if res != COMM_SUCCESS:
                print(f"[torque_enable][ID {dxl_id}] TxRxResult: {self.packetHandler.getTxRxResult(res)}")
            elif err:
                print(f"[torque_enable][ID {dxl_id}] RxPacketError: {self.packetHandler.getRxPacketError(err)}")
            else:
                print(f"[torque_enable][ID {dxl_id}] torque enabled")
        return positions

    def readVelocity(self):
        speeds = []
        for dxl_id in self.ids:
            speed, res, err = self.packetHandler.read2ByteTxRx(
                self.portHandler, dxl_id,
                ADDR_MX_PRESENT_SPEED
            )
            if res != COMM_SUCCESS:
                print(f"[torque_enable][ID {dxl_id}] TxRxResult: {self.packetHandler.getTxRxResult(res)}")
            elif err:
                print(f"[torque_enable][ID {dxl_id}] RxPacketError: {self.packetHandler.getRxPacketError(err)}")
            else:
                print(f"[torque_enable][ID {dxl_id}] torque enabled")
        return speeds


    def close(self):
        # Disable torque on all motors
        for dxl_id in self.ids:
            res, err = self.packetHandler.write1ByteTxRx(
                self.portHandler,
                dxl_id,
                ADDR_MX_TORQUE_ENABLE,
                TORQUE_DISABLE
            )
            if res != COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(res))
            elif err:
                print(self.packetHandler.getRxPacketError(err))

        # Close port
        self.portHandler.closePort()