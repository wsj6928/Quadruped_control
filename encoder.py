from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import rospy
from joint_pub.msg import joint
from sensor_msgs.msg import JointState

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84

# Protocol version
PROTOCOL_VERSION            = 2.0               #  See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#2 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID : 3
DXL4_ID                     = 4                 # Dynamixel#2 ID : 4
DXL5_ID                     = 5                 # Dynamixel#1 ID : 5
DXL6_ID                     = 6                 # Dynamixel#2 ID : 6
DXL7_ID                     = 7                 # Dynamixel#1 ID : 7
DXL8_ID                     = 8                 # Dynamixel#2 ID : 8

BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME1                 = '/dev/ttyUSB0'    # Check which port is being used on your controller
DEVICENAME2                 = '/dev/ttyUSB0'    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
DEVICENAME3                 = '/dev/ttyUSB1'
DEVICENAME4                 = '/dev/ttyUSB1'
DEVICENAME5                 = '/dev/ttyUSB2'
DEVICENAME6                 = '/dev/ttyUSB2'
DEVICENAME7                 = '/dev/ttyUSB3'
DEVICENAME8                 = '/dev/ttyUSB3'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler1 = PortHandler(DEVICENAME1)
portHandler2 = PortHandler(DEVICENAME2)
portHandler3 = PortHandler(DEVICENAME3)
portHandler4 = PortHandler(DEVICENAME4)
portHandler5 = PortHandler(DEVICENAME5)
portHandler6 = PortHandler(DEVICENAME6)
portHandler7 = PortHandler(DEVICENAME7)
portHandler8 = PortHandler(DEVICENAME8)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

######################### Open port #########################################
# Open port1
if portHandler1.openPort():
    print("Succeeded to open the port 1")
else:
    print("Failed to open the port 1")
    print("Press any key to terminate...")
    quit()

# Open port2
if portHandler2.openPort():
    print("Succeeded to open the port 2")
else:
    print("Failed to open the port 2")
    print("Press any key to terminate...")
    quit()

# Open port3
if portHandler3.openPort():
    print("Succeeded to open the port 3")
else:
    print("Failed to open the port 3")
    print("Press any key to terminate...")
    quit()

# Open port4
if portHandler4.openPort():
    print("Succeeded to open the port 4")
else:
    print("Failed to open the port 4")
    print("Press any key to terminate...")
    quit()

# Open port5
if portHandler5.openPort():
    print("Succeeded to open the port 5")
else:
    print("Failed to open the port 5")
    print("Press any key to terminate...")
    quit()

# Open port6
if portHandler6.openPort():
    print("Succeeded to open the port 6")
else:
    print("Failed to open the port 6")
    print("Press any key to terminate...")
    quit()

# Open port7
if portHandler7.openPort():
    print("Succeeded to open the port 7")
else:
    print("Failed to open the port 7")
    print("Press any key to terminate...")
    quit()

# Open port8
if portHandler8.openPort():
    print("Succeeded to open the port 8")
else:
    print("Failed to open the port 8")
    print("Press any key to terminate...")
    quit()

##################### set baudrate ##################################

# Set port1 baudrate
if portHandler1.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 1")
else:
    print("Failed to change the baudrate 1")
    print("Press any key to terminate...")
    quit()

# Set port2 baudrate
if portHandler2.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 2")
else:
    print("Failed to change the baudrate 2")
    print("Press any key to terminate...")
    quit()

# Set port3 baudrate
if portHandler3.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 3")
else:
    print("Failed to change the baudrate 3")
    print("Press any key to terminate...")
    quit()

# Set port4 baudrate
if portHandler4.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 4")
else:
    print("Failed to change the baudrate 4")
    print("Press any key to terminate...")
    quit()

# Set port5 baudrate
if portHandler5.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 5")
else:
    print("Failed to change the baudrate 5")
    print("Press any key to terminate...")
    quit()

# Set port6 baudrate
if portHandler6.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 6")
else:
    print("Failed to change the baudrate 6")
    print("Press any key to terminate...")
    quit()

# Set port7 baudrate
if portHandler7.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 7")
else:
    print("Failed to change the baudrate 7")
    print("Press any key to terminate...")
    quit()

# Set port8 baudrate
if portHandler8.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate 8")
else:
    print("Failed to change the baudrate 8")
    print("Press any key to terminate...")
    quit()

################################## enable torque ################################



######################### Set PID Gain Position Loop  ##############################


################################ control #########################################



############################## write goal position ####################################

rospy.init_node('joint_publisher',anonymous=True)
joint_pub = rospy.Publisher('joint_states',JointState,queue_size=1)
joint_instance = JointState()
joint_instance.name.append("front_left_joint1")
joint_instance.name.append("front_left_joint2")
joint_instance.name.append("front_right_joint1")
joint_instance.name.append("front_right_joint2")
joint_instance.name.append("back_left_joint1")
joint_instance.name.append("back_left_joint2")
joint_instance.name.append("back_right_joint1")
joint_instance.name.append("back_right_joint2")
joint_instance.position.append(0.0)
joint_instance.position.append(0.0)
joint_instance.position.append(0.0)
joint_instance.position.append(0.0)
joint_instance.position.append(0.0)
joint_instance.position.append(0.0)
joint_instance.position.append(0.0)
joint_instance.position.append(0.0)

while 1:
    ########################################## read position ###################################
    # Read Dynamixel#1 present position
    dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler1, DXL1_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#2 present position
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler2, DXL2_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#3 present position
    dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler3, DXL3_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#4 present position
    dxl4_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler4, DXL4_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#5 present position
    dxl5_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler5, DXL5_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#6 present position
    dxl6_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler6, DXL6_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#7 present position
    dxl7_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler7, DXL7_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Read Dynamixel#8 present position
    dxl8_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler8, DXL8_ID, ADDR_PRO_PRESENT_POSITION)
    # if dxl_comm_result != COMM_SUCCESS:
    #     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    # elif dxl_error != 0:
    #     print("%s" % packetHandler.getRxPacketError(dxl_error))

    #print("[1]%03d\t[2]%03d\t[3]%03d\t[4]%03d\t[5]%03d\t[6]%03d\t[7]%03d\t[8]%03d " % (dxl1_present_position,dxl2_present_position,dxl3_present_position,dxl4_present_position,dxl5_present_position,dxl6_present_position,dxl7_present_position,dxl8_present_position))

    #1 degree ~ 90
    offset1 = -410
    offset2 = -80
    offset3 = 167
    offset4 = -300
    offset5 = 0
    offset6 = -300
    offset7 = -168
    offset8 = -120

    theta1 = (20475.0 + offset1 -dxl1_present_position)*(15.0/120.0)*(360.0/4095.0)-90.0
    theta2 = (16380.0 + offset2 -dxl2_present_position)*(15.0/120.0)*(360.0/4095.0)
    theta3 = ((dxl3_present_position-offset3)*(15.0/120.0)*(360.0/4095.0))-90.0
    theta4 = ((dxl4_present_position-offset4)*(15.0/120.0)*(360.0/4095.0))-45.0
    theta5 = (20475.0 + offset5 -dxl5_present_position)*(15.0/120.0)*(360.0/4095.0)-90.0
    theta6 = (16380.0 + offset6 -dxl6_present_position)*(15.0/120.0)*(360.0/4095.0)
    theta7 = ((dxl7_present_position-offset7)*(15.0/120.0)*(360.0/4095.0))-90.0
    theta8 = ((dxl8_present_position-offset8)*(15.0/120.0)*(360.0/4095.0))-45.0

    print("[1]%.2f\t[2]%.2f\t[3]%.2f\t[4]%.2f\t[5]%.2f\t[6]%.2f\t[7]%.2f\t[8]%.2f " % (theta1,theta2,theta3,theta4,theta5,theta6,theta7,theta8))
    joint_instance.position[0] = theta1*3.1415/180.0
    joint_instance.position[1] = theta2*3.1415/180.0
    joint_instance.position[2] = theta3*3.1415/180.0
    joint_instance.position[3] = theta4*3.1415/180.0
    joint_instance.position[4] = theta5*3.1415/180.0
    joint_instance.position[5] = theta6*3.1415/180.0
    joint_instance.position[6] = theta7*3.1415/180.0
    joint_instance.position[7] = theta8*3.1415/180.0
    joint_instance.header.stamp = rospy.Time.now()
    joint_pub.publish(joint_instance)


############################################ disable torque ################################33

#################################### set PID ##############################

#####################################Close port ############################3

# Close port1
portHandler1.closePort()

# Close port2
portHandler2.closePort()

# Close port3
portHandler3.closePort()

# Close port4
portHandler4.closePort()

# Close port5
portHandler5.closePort()

# Close port6
portHandler6.closePort()

# Close port7
portHandler7.closePort()

# Close port8
portHandler8.closePort()