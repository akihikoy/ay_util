#!/usr/bin/python
#\file    proc_manager_gen3.py
#\brief   Provides TProcessManagerGen3.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.21, 2024
#         Separated from ur_panel.py and refactored.
import roslib
roslib.load_manifest('ay_trick_msgs')
import rospy
import ay_trick_msgs.msg
import ay_trick_msgs.srv
from ay_py.core import InsertDict
from proc_manager_gui import TProcessManagerGUIBase

class TProcessManagerGen3(TProcessManagerGUIBase):

  #Classify the current status.
  #Overwritten for Gen3.
  def GetStatus(self):
    status= self.UNDEFINED
    if not self.robot_ros_running:
      status= self.NO_CORE_PROGRAM
    else:
      status= self.TORQUE_ENABLED
      if not self.script_node_running:
        status= self.ROBOT_READY
      elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.READY:
        status= self.WAIT_REQUEST
      elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING:
        status= self.PROGRAM_RUNNING
    return status

  def __init__(self, node_name='gen3_dashboard', topics_to_monitor=None, is_sim=False):
    topics_to_monitor_base= {
      'Robot': '/gen3a/joint_states',
      }
    if topics_to_monitor is not None:  InsertDict(topics_to_monitor_base, topics_to_monitor)
    TProcessManagerGUIBase.__init__(self, node_name=node_name, topics_to_monitor=topics_to_monitor_base, is_sim=is_sim)

  def Disconnect(self):
    if self.is_sim:  return
    TProcessManagerGUIBase.Cleanup(self)

