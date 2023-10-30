#!/usr/bin/python
#\file    dxlg_reboot.py
#\brief   Reboot a dynamixel-based gripper.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.04, 2022
import roslib;
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_util_msgs')
import rospy
from ay_py.misc.dxl_util import TDynamixel1,DxlPortHandler
from dxlg_driver import ExpandGripperType
import sys

def OperateDxlGripper(dev='/dev/ttyUSB0', gripper_type='DxlGripper', finger_type=None, command='Reboot'):
  dev= dev
  gripper_type= gripper_type
  if gripper_type=='DxlGripper':
    dxl= [TDynamixel1('XM430-W350')]
    dxl[0].Id= 1
    dxl[0].Baudrate= 2e6
    dxl[0].Setup()
  elif gripper_type=='RHP12RNGripper':
    dxl= [TDynamixel1('RH-P12-RN')]
    dxl[0].Id= 1
    dxl[0].Baudrate= 2e6
    dxl[0].Setup()
  elif gripper_type=='RHP12RNAGripper':
    dxl= [TDynamixel1('RH-P12-RN(A)')]
    dxl[0].Id= 1
    dxl[0].Baudrate= 2e6
    dxl[0].Setup()
  elif gripper_type=='EZGripper':
    pass
  elif gripper_type=='DxlpO2Gripper':
    pass
  elif gripper_type=='DxlpY1Gripper':
    dxl= [TDynamixel1('XD540-T270')]
    dxl[0].Id= 1
    dxl[0].Baudrate= 2e6
    dxl[0].Setup()
  elif gripper_type=='DxlO3Gripper':
    pass
  else:
    raise Exception('Invalid gripper type: {gripper_type}'.format(gripper_type=gripper_type))

  #Set callback to exit when Ctrl+C is pressed.
  DxlPortHandler.ReopenCallback= lambda: not rospy.is_shutdown()

  print 'Executing the command:',command
  if command=='EnableTorque':  #Enable joint_names (joint_names is [], all joints are enabled).
    for d in dxl:
      d.EnableTorque()
  elif command=='DisableTorque':  #Disable joint_names (joint_names is [], all joints are disabled).
    for d in dxl:
      d.DisableTorque()
  elif command=='Reboot':  #Reboot joint_names (joint_names is [], all joints are rebooted).
    for d in dxl:
      d.Reboot()
  elif command=='FactoryReset':  #FactoryReset joint_names (joint_names is [], all joints are rebooted).
    for d in dxl:
      d.FactoryReset()

if __name__=='__main__':
  rospy.init_node('gripper_reboot')
  dxldev= rospy.get_param('~dxldev', '/dev/ttyUSB0')
  gripper_type= ExpandGripperType(rospy.get_param('~gripper_type', 'DxlGripper'))
  finger_type= rospy.get_param('~finger_type', '')
  command= rospy.get_param('~command', 'Reboot')
  print '''Parameters:
    dxldev: {dxldev}
    gripper_type: {gripper_type}
    finger_type: {finger_type}
    command: {command}
  '''.format(dxldev=dxldev, gripper_type=gripper_type, finger_type=finger_type, command=command)

  OperateDxlGripper(dxldev, gripper_type, finger_type, command)
