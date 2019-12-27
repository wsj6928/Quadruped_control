#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Sync Read and Sync Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import rospy
from joint_pub.msg import joint
from sensor_msgs.msg import JointState

# ros
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

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

ADDR_PRO_POSITION_D_GAIN    = 80
ADDR_PRO_POSITION_I_GAIN    = 82
ADDR_PRO_POSITION_P_GAIN    = 84

ADDR_PRO_PROFILE_ACCELERATION = 108
ADDR_PRO_PROFILE_VELOCITY = 112
ADDR_PRO_ACCELERATION_LIMIT = 40
ADDR_PRO_VELOCITY_LIMIT = 44

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

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

DEVICENAME0                 = '/dev/ttyUSB0'    # Check which port is being used on your controller
DEVICENAME1                 = '/dev/ttyUSB1'
DEVICENAME2                 = '/dev/ttyUSB2'
DEVICENAME3                 = '/dev/ttyUSB3'                  # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler0 = PortHandler(DEVICENAME0)
portHandler1 = PortHandler(DEVICENAME1)
portHandler2 = PortHandler(DEVICENAME2)
portHandler3 = PortHandler(DEVICENAME3)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite0 = GroupSyncWrite(portHandler0, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncWrite1 = GroupSyncWrite(portHandler1, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncWrite2 = GroupSyncWrite(portHandler2, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)
groupSyncWrite3 = GroupSyncWrite(portHandler3, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead0 = GroupSyncRead(portHandler0, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
groupSyncRead1 = GroupSyncRead(portHandler1, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
groupSyncRead2 = GroupSyncRead(portHandler2, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
groupSyncRead3 = GroupSyncRead(portHandler3, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

# Open port
if portHandler0.openPort():
    print("Succeeded to open the port 0")
else:
    print("Failed to open the port 0")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler1.openPort():
    print("Succeeded to open the port 1")
else:
    print("Failed to open the port 1")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler2.openPort():
    print("Succeeded to open the port 2")
else:
    print("Failed to open the port 2")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler3.openPort():
    print("Succeeded to open the port 3")
else:
    print("Failed to open the port 3")
    print("Press any key to terminate...")
    getch()
    quit()

##################### set baudrate ##################################

# Set port baudrate
if portHandler0.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
if portHandler1.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
if portHandler2.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()
if portHandler3.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

################################## enable torque ################################

# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)

# Enable Dynamixel#4 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL4_ID)

# Enable Dynamixel#5 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL5_ID)

# Enable Dynamixel#6 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL6_ID)

# Enable Dynamixel#7 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL7_ID)

# Enable Dynamixel#8 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL8_ID)


# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncRead0.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncRead0.addParam(DXL2_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
    quit()

# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncRead1.addParam(DXL3_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL3_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncRead1.addParam(DXL4_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL4_ID)
    quit()
# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncRead2.addParam(DXL5_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL5_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncRead2.addParam(DXL6_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL6_ID)
    quit()
# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = groupSyncRead3.addParam(DXL7_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL7_ID)
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = groupSyncRead3.addParam(DXL8_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL8_ID)
    quit()

######################### Set PID Gain Position Loop  ##############################
# set_P_Gain = 500   
# set_I_Gain = 100     
# set_D_Gain = 4700 
set_P_Gain = 2000
set_I_Gain = 10   
set_D_Gain = 10

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_POSITION_P_GAIN, set_P_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_POSITION_I_GAIN, set_I_Gain)
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_POSITION_D_GAIN, set_D_Gain)

######################### Set velocity profile  ##############################

set_A_l = 200
set_V_l = 100

set_A_PRFL = 40
set_V_PRFL = 60

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_VELOCITY_LIMIT, int(set_V_l))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_ACCELERATION_LIMIT, int(set_A_l))


dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_PROFILE_ACCELERATION, int(set_A_PRFL))
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_PROFILE_VELOCITY, int(set_V_PRFL))

################################ control #########################################

index = 0
goal1 = []
goal2 = []
goal3 = []
goal4 = []
goal5 = []
goal6 = []
goal7 = []
goal8 = []
#1 degree ~ 48


offset1 = -410
offset2 = -80
offset3 = 167
offset4 = -300
offset5 = 0
offset6 = -300
offset7 = -168
offset8 = -120

f= open("/home/seungjun/walking_simulation/mark8_12stairs/up12.txt",'r')
# f= open("/home/seungjun/walking_simulation/initial/03.txt",'r')
lines = f.readlines()
for line in lines:
    b = line.split(',')
    goal1.append(int(20475 - round((float(b[0])+90.0) * (120.0/15.0) * (4095.0/360.0))+offset1))
    goal2.append(int(16380 - round(float(b[1]) * (120.0/15.0) * (4095.0/360.0))+offset2))
    goal3.append(int(round((float(b[2])+90.0) * (120.0/15.0) * (4095.0/360.0))+ offset3))
    goal4.append(int(round((float(b[3])+45.0) * (120.0/15.0) * (4095.0/360.0))+offset4))
    goal5.append(int(20475 - round((float(b[4])+90.0) * (120.0/15.0) * (4095.0/360.0))+offset5))
    goal6.append(int(16380 - round(float(b[5]) * (120.0/15.0) * (4095.0/360.0))+offset6))
    goal7.append(int(round((float(b[6])+90.0) * (120.0/15.0) * (4095.0/360.0))+ offset7))
    goal8.append(int(round((float(b[7])+45.0) * (120.0/15.0) * (4095.0/360.0))+offset8))

while 1:
    # print("Press any key to continue! (or press ESC to quit!)")
    # if getch() == chr(0x1b):
    #     break

    # Allocate goal position value into byte array
    param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(goal1[index])), DXL_HIBYTE(DXL_LOWORD(goal1[index])), DXL_LOBYTE(DXL_HIWORD(goal1[index])), DXL_HIBYTE(DXL_HIWORD(goal1[index]))]
    param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(goal2[index])), DXL_HIBYTE(DXL_LOWORD(goal2[index])), DXL_LOBYTE(DXL_HIWORD(goal2[index])), DXL_HIBYTE(DXL_HIWORD(goal2[index]))]
    param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(goal3[index])), DXL_HIBYTE(DXL_LOWORD(goal3[index])), DXL_LOBYTE(DXL_HIWORD(goal3[index])), DXL_HIBYTE(DXL_HIWORD(goal3[index]))]
    param_goal_position4 = [DXL_LOBYTE(DXL_LOWORD(goal4[index])), DXL_HIBYTE(DXL_LOWORD(goal4[index])), DXL_LOBYTE(DXL_HIWORD(goal4[index])), DXL_HIBYTE(DXL_HIWORD(goal4[index]))]
    param_goal_position5 = [DXL_LOBYTE(DXL_LOWORD(goal5[index])), DXL_HIBYTE(DXL_LOWORD(goal5[index])), DXL_LOBYTE(DXL_HIWORD(goal5[index])), DXL_HIBYTE(DXL_HIWORD(goal5[index]))]
    param_goal_position6 = [DXL_LOBYTE(DXL_LOWORD(goal6[index])), DXL_HIBYTE(DXL_LOWORD(goal6[index])), DXL_LOBYTE(DXL_HIWORD(goal6[index])), DXL_HIBYTE(DXL_HIWORD(goal6[index]))]
    param_goal_position7 = [DXL_LOBYTE(DXL_LOWORD(goal7[index])), DXL_HIBYTE(DXL_LOWORD(goal7[index])), DXL_LOBYTE(DXL_HIWORD(goal7[index])), DXL_HIBYTE(DXL_HIWORD(goal7[index]))]
    param_goal_position8 = [DXL_LOBYTE(DXL_LOWORD(goal8[index])), DXL_HIBYTE(DXL_LOWORD(goal8[index])), DXL_LOBYTE(DXL_HIWORD(goal8[index])), DXL_HIBYTE(DXL_HIWORD(goal8[index]))]

    # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite0.addParam(DXL1_ID, param_goal_position1)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite0.addParam(DXL2_ID, param_goal_position2)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
        quit()
    
    # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite1.addParam(DXL3_ID, param_goal_position3)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
        quit()

    # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite1.addParam(DXL4_ID, param_goal_position4)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
        quit()
    
    # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite2.addParam(DXL5_ID, param_goal_position5)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
        quit()

    # Add Dynamixel#6 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite2.addParam(DXL6_ID, param_goal_position6)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL6_ID)
        quit()
    
    # Add Dynamixel#7 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite3.addParam(DXL7_ID, param_goal_position7)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL7_ID)
        quit()

    # Add Dynamixel#8 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite3.addParam(DXL8_ID, param_goal_position8)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % DXL8_ID)
        quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite0.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite1.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite2.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite3.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite0.clearParam()
    groupSyncWrite1.clearParam()
    groupSyncWrite2.clearParam()
    groupSyncWrite3.clearParam()

    while 1:
        # Syncread present position
        dxl_comm_result = groupSyncRead0.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
        # Syncread present position
        dxl_comm_result = groupSyncRead1.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Syncread present position
        dxl_comm_result = groupSyncRead2.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        
        # Syncread present position
        dxl_comm_result = groupSyncRead3.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead0.isAvailable(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead0.isAvailable(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
            quit()
        
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead1.isAvailable(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL3_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead1.isAvailable(DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL4_ID)
            quit()
        
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead2.isAvailable(DXL5_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL5_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead2.isAvailable(DXL6_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL6_ID)
            quit()
        
        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead3.isAvailable(DXL7_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL7_ID)
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = groupSyncRead3.isAvailable(DXL8_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % DXL8_ID)
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = groupSyncRead0.getData(DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl2_present_position = groupSyncRead0.getData(DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#1 present position value
        dxl3_present_position = groupSyncRead1.getData(DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl4_present_position = groupSyncRead1.getData(DXL4_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#1 present position value
        dxl5_present_position = groupSyncRead2.getData(DXL5_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl6_present_position = groupSyncRead2.getData(DXL6_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#1 present position value
        dxl7_present_position = groupSyncRead3.getData(DXL7_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl8_present_position = groupSyncRead3.getData(DXL8_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, goal1[index], dxl1_present_position, DXL2_ID, goal2[index], dxl2_present_position))
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

        if not ((abs(goal1[index] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
        (abs(goal2[index] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
        (abs(goal3[index] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
        (abs(goal4[index] - dxl4_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
        (abs(goal5[index] - dxl5_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
        (abs(goal6[index] - dxl6_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
        (abs(goal7[index] - dxl7_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
        (abs(goal8[index] - dxl8_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
            break

    # Change goal position
    if index == len(lines)-1:
        break
    else:
        index +=1

# Clear syncread parameter storage
groupSyncRead0.clearParam()
groupSyncRead1.clearParam()
groupSyncRead2.clearParam()
groupSyncRead3.clearParam()

# # Disable Dynamixel#1 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#2 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler0, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#3 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#4 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler1, DXL4_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#5 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler2, DXL5_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#6 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler2, DXL6_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#7 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler3, DXL7_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Disable Dynamixel#8 Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler3, DXL8_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler0.closePort()

# Close port1
portHandler1.closePort()

# Close port2
portHandler2.closePort()

# Close port3
portHandler3.closePort()