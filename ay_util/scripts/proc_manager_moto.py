#!/usr/bin/python
#\file    proc_manager_moto.py
#\brief   Provides TProcessManagerMotoman.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.21, 2024
import roslib
roslib.load_manifest('ay_trick_msgs')
import copy
import rospy
import std_msgs.msg
import std_srvs.srv
import ay_trick_msgs.msg
import ay_trick_msgs.srv
import ay_util_msgs.srv
import industrial_msgs.msg
from ay_py.core import InsertDict
from ay_py.ros.base import SetupServiceProxy
try:
  roslib.load_manifest('ur_msgs')
  import ur_msgs.msg
except Exception as e:
  print e
from proc_manager_gui import TProcessManagerGUIBase

class TProcessManagerMotoman(TProcessManagerGUIBase):

  #Classify the current status.
  #Overwritten for Motoman.
  def GetStatus(self):
    #FIXME: In which case, we should set EMERGENCY?
    status= self.UNDEFINED
    if not self.robot_ros_running or self.moto_robot_status is None:
      status= self.NO_CORE_PROGRAM
    else:
      #self.moto_robot_status.mode.val==industrial_msgs.msg.RobotMode.AUTO
      if self.moto_robot_status.in_error.val==industrial_msgs.msg.TriState.TRUE:
        status= self.FAULT
      elif self.moto_robot_status.e_stopped.val==industrial_msgs.msg.TriState.TRUE:
        status= self.ROBOT_EMERGENCY_STOP
      elif self.moto_robot_status.drives_powered.val==industrial_msgs.msg.TriState.FALSE:
        status= self.POWER_OFF
      else:
        status= self.TORQUE_ENABLED
        if self.moto_robot_status.motion_possible.val==industrial_msgs.msg.TriState.TRUE:
          status= self.ROBOT_READY
          if not self.script_node_running:
            status= self.ROBOT_READY
          elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.READY:
            status= self.WAIT_REQUEST
          elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING:
            status= self.PROGRAM_RUNNING
    return status

  def __init__(self, node_name='motoman_dashboard', topics_to_monitor=None, is_sim=False):
    TProcessManagerGUIBase.__init__(self, node_name=node_name, topics_to_monitor=topics_to_monitor, is_sim=is_sim)
    self.moto_robot_status= None
    self.io_states= None

  def ConnectToRobot(self, timeout=6.0):
    self.srvp_set_pui= SetupServiceProxy('/ur_pui_server/set_pui', ay_util_msgs.srv.SetPUI, persistent=False, time_out=timeout)
    if not self.is_sim:
      self.sub_robot_status= rospy.Subscriber('/robot_status', industrial_msgs.msg.RobotStatus, self.RobotStatusCallback)
      self.srvp_robot_enable= SetupServiceProxy('/robot_enable', std_srvs.srv.Trigger, persistent=False, time_out=timeout)
      self.srvp_robot_disable= SetupServiceProxy('/robot_disable', std_srvs.srv.Trigger, persistent=False, time_out=timeout)

    #Connect to io_states to publish a fake io_states.
    self.pub_io_states= rospy.Publisher('/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, queue_size=10)
    self.sub_io_states= rospy.Subscriber('/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, self.IOStatesCallback)

  def DisconnectFromRobot(self):
    if self.is_sim:  return
    TProcessManagerGUIBase.Cleanup(self)
    self.srvp_set_pui= None
    self.srvp_robot_enable= None
    self.srvp_robot_disable= None

    if self.pub_io_states is not None:
      self.pub_io_states.unregister()
    self.pub_io_states= None

  def RobotStatusCallback(self, msg):
    self.moto_robot_status= msg

  def EnableTorque(self):
    if self.srvp_robot_enable is not None:
      self.srvp_robot_enable()

  def DisableTorque(self):
    if self.srvp_robot_disable is not None:
      self.srvp_robot_disable()

  def IOStatesCallback(self, msg):
    self.io_states= msg

  def SendFakeDigitalInSignal(self, signal_idx, signal_trg):
    if self.pub_io_states is None:  return
    if self.io_states is not None:
      msg= copy.deepcopy(self.io_states)
      print 'debug,1,',type(msg)
      print 'debug,1,',dir(msg)
      print 'debug,1,',msg
    else:
      msg= ur_msgs.msg.IOStates()
      msg.digital_in_states= [ur_msgs.msg.Digital(pin,False) for pin in range(18)]
      msg.digital_out_states= [ur_msgs.msg.Digital(pin,False) for pin in range(18)]
      msg.flag_states= [ur_msgs.msg.Digital(pin,False) for pin in range(2)]
      msg.analog_in_states= [ur_msgs.msg.Analog(pin,0,0) for pin in range(2)]
      msg.analog_out_states= [ur_msgs.msg.Analog(pin,0,0) for pin in range(2)]
      print 'debug,2,',type(msg)
      print 'debug,2,',dir(msg)
    msg.digital_in_states[signal_idx]= ur_msgs.msg.Digital(signal_idx,signal_trg)
    self.pub_io_states.publish(msg)

