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
from ay_py.misc.dxl_util import DxlPortHandler
import sys

class TDxlGripperDriver(object):
  def __init__(self, dev='/dev/ttyUSB0', gripper_type='DxlGripper', finger_type=None):
    self.dev= dev
    self.gripper_type= gripper_type
    if self.gripper_type=='DxlGripper':
      mod= __import__('ay_py.misc.dxl_gripper',globals(),None,('TDynamixelGripper',))
      self.gripper= mod.TDynamixelGripper(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='RHP12RNGripper':
      mod= __import__('ay_py.misc.dxl_rhp12rn',globals(),None,('TRHP12RN',))
      self.gripper= mod.TRHP12RN(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='EZGripper':
      mod= __import__('ay_py.misc.dxl_ezg',globals(),None,('TEZG',))
      self.gripper= mod.TEZG(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='DxlpO2Gripper':
      mod= __import__('ay_py.misc.dxl_dxlpo2',globals(),None,('TDxlpO2',))
      self.gripper= mod.TDxlpO2(dev=self.dev, finger_type=finger_type)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='DxlpY1Gripper':
      mod= __import__('ay_py.misc.dxl_dxlpy1',globals(),None,('TDxlpY1',))
      self.gripper= mod.TDxlpY1(dev=self.dev)
      self.joint_names= ['joint0']
      self.dxl= {'joint0':self.gripper.dxl}
    elif self.gripper_type=='DxlO3Gripper':
      mod= __import__('ay_py.misc.dxl_dxlo3',globals(),None,('TDxlO3',))
      self.gripper= mod.TDxlO3(dev=self.dev)
      self.joint_names= ['joint0','joint1']
      self.dxl= {'joint0':self.gripper.dxl[0], 'joint1':self.gripper.dxl[1]}
    else:
      raise Exception('Invalid gripper type: {gripper_type}'.format(gripper_type=gripper_type))

    #Set callback to exit when Ctrl+C is pressed.
    DxlPortHandler.ReopenCallback= lambda: not rospy.is_shutdown()

    print 'Initializing and activating {gripper_type}({finger_type}) gripper...'.format(gripper_type=self.gripper_type,finger_type=finger_type)
    if not self.gripper.Init():
      raise Exception('Failed to setup {gripper_type}({finger_type}) gripper.'.format(gripper_type=self.gripper_type,finger_type=finger_type))

  def OperateDxl(self, command):
    if command=='EnableTorque':  #Enable joint_names (joint_names is [], all joints are enabled).
      with self.gripper.port_locker:
        for j in self.joint_names:  self.dxl[j].EnableTorque()
    elif command=='DisableTorque':  #Disable joint_names (joint_names is [], all joints are disabled).
      with self.gripper.port_locker:
        for j in self.joint_names:  self.dxl[j].DisableTorque()
    elif command=='Reboot':  #Reboot joint_names (joint_names is [], all joints are rebooted).
      with self.gripper.port_locker:
        for j in self.joint_names:  self.dxl[j].Reboot()


if __name__=='__main__':
  dev= sys.argv[1] if len(sys.argv)>1 else '/dev/ttyUSB0'
  gripper_type= sys.argv[2] if len(sys.argv)>2 else 'DxlGripper'
  finger_type= sys.argv[3] if len(sys.argv)>3 else None
  command= 'Reboot'
  print 'args=',sys.argv
  TDxlGripperDriver(dev, gripper_type, finger_type).OperateDxl(command)

