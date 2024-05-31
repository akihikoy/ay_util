#!/usr/bin/python
#\file    proc_manager_gui.py
#\brief   Define a base class of a process manager with GUI (currently Qt based).
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.21, 2024
#         Separated from ur_dashboard_gui.py
import roslib
roslib.load_manifest('ay_py')
from ay_py.core import CPrint, InsertDict
from ay_py.tool.py_panel import QtCore
import threading
import copy
import rospy
import ay_util_msgs.srv
from proc_manager import TSubProcManager
from topic_monitor import TTopicMonitor
from script_node_client import TScriptNodeClient

'''
Base class of a process manager with GUI (currently Qt based).
'''
class TProcessManagerGUIBase(QtCore.QObject, TSubProcManager, TScriptNodeClient, TTopicMonitor):

  #Definition of states (commonly defined for all kinds of robots):
  UNDEFINED= -10
  FAULT= -4
  EMERGENCY= -3
  PROTECTIVE_STOP= -2
  ROBOT_EMERGENCY_STOP= -1
  NO_CORE_PROGRAM= 0
  POWER_OFF= 1
  BOOTING= 2
  IDLE= 3
  TORQUE_ENABLED= 4
  ROBOT_READY= 5
  WAIT_REQUEST= 6
  PROGRAM_RUNNING= 7

  onstatuschanged= QtCore.pyqtSignal(int)
  ontopicshzupdated= QtCore.pyqtSignal()

  #Classify the current status.
  #Should be overwritten by a sub class for each robot.
  def GetStatus(self):
    return self.UNDEFINED

  def UpdateStatus(self):
    t_now= rospy.Time.now()
    self.robot_ros_running= self.IsActive('Robot')

    status= self.GetStatus()

    #NOTE: Do not use yellow as it is used by other modules.
    if status in (self.UNDEFINED, self.FAULT, self.EMERGENCY, self.ROBOT_EMERGENCY_STOP, self.PROTECTIVE_STOP):
      self.status_color= ['red']
    elif status in (self.WAIT_REQUEST,):
      self.status_color= ['green']
    elif status in (self.PROGRAM_RUNNING,):
      self.status_color= ['green']
    else:
      self.status_color= []

    if self.status != status:
      self.status= status
      self.OnStatusChanged()
    #self.onstatuschanged.emit(self.status)

  def OnStatusChanged(self):
    #if self.panel is None:  return
    #for w_name, (w_type, w_param) in self.panel.widgets_in.iteritems():
      #if 'onstatuschanged' in w_param and w_param['onstatuschanged'] is not None:
        #w_param['onstatuschanged'](self.panel, self.status)
    self.onstatuschanged.emit(self.status)

    self.TurnOffLEDAll()
    for color in self.status_color:  self.SetLEDLight(color, True)

    if self.status in (self.PROGRAM_RUNNING,):
      self.SetStartStopLEDs(True, True)

  def UpdateStatusThread(self):
    rate= rospy.Rate(20)
    while self.thread_status_update_running and not rospy.is_shutdown():
      self.UpdateStatus()
      rate.sleep()

  def StartUpdateStatusThread(self):
    self.StartTopicMonitorThread()
    self.thread_status_update_running= True
    self.thread_status_update= threading.Thread(name='status_update', target=self.UpdateStatusThread)
    self.thread_status_update.start()

  def StopUpdateStatusThread(self):
    self.thread_status_update_running= False
    if self.thread_status_update is not None:
      self.thread_status_update.join()
    self.StopTopicMonitorThread()

  def __init__(self, node_name='robot_operation', topics_to_monitor=None, is_sim=False):
    topics_to_monitor_base= {
      'Robot': '/joint_states',
      'Gripper': '/gripper_driver/joint_states',
      }
    if topics_to_monitor is not None:  InsertDict(topics_to_monitor_base, topics_to_monitor)
    TTopicMonitor.__init__(self, topics_to_monitor_base)
    self.thread_topics_hz_callback= lambda: self.ontopicshzupdated.emit()

    self.robot_ros_running= None  #==IsActive('Robot')   ; NOTE: Renamed from ur_ros_running

    QtCore.QObject.__init__(self)
    TSubProcManager.__init__(self)
    TScriptNodeClient.__init__(self)
    self.node_name= node_name
    self.is_sim= is_sim

    self.status_names= {
      self.UNDEFINED:           'UNDEFINED'           ,
      self.FAULT:               'FAULT'               ,
      self.EMERGENCY:           'EMERGENCY'           ,
      self.PROTECTIVE_STOP:     'PROTECTIVE_STOP'     ,
      self.ROBOT_EMERGENCY_STOP:'ROBOT_EMERGENCY_STOP',
      self.NO_CORE_PROGRAM:     'NO_CORE_PROGRAM'     ,
      self.POWER_OFF:           'POWER_OFF'           ,
      self.BOOTING:             'BOOTING'             ,
      self.IDLE:                'IDLE'                ,
      self.TORQUE_ENABLED:      'TORQUE_ENABLED'      ,
      self.ROBOT_READY:         'ROBOT_READY'         ,
      self.WAIT_REQUEST:        'WAIT_REQUEST'        ,
      self.PROGRAM_RUNNING:     'PROGRAM_RUNNING'     ,
      }

    #self.panel= None
    self.status= self.UNDEFINED
    self.status_color= []  #List of 'red','green' (yellow is used by the other modules).

    self.thread_status_update= None
    self.thread_status_update_running= False

    self.srvp_set_pui= None  #TODO: Connect to a PUI server set_pui service (ay_util_msgs.srv.SetPUI) in a robot subclass.

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def TurnOffLEDAll(self):
    self.SetLEDLight('red', False)
    self.SetLEDLight('green', False)
    #self.SetLEDLight('yellow', False)
    #self.SetBeep(False)
    self.SetStartStopLEDs(False, False)

  #color: 'red','yellow','green'
  def SetLEDLight(self, color, is_on):
    if self.is_sim:  return
    if self.srvp_set_pui is None:  return
    config_name= {'red':'STATE_LED_RED',
                  'yellow':'STATE_LED_YELLOW',
                  'green':'STATE_LED_GREEN'}[color]
    req= ay_util_msgs.srv.SetPUIRequest()
    req.name= config_name
    req.action= req.ON if is_on else req.OFF
    return self.srvp_set_pui(req)

  def SetStartStopLEDs(self, is_start_on, is_stop_on):
    if self.is_sim:  return
    if self.srvp_set_pui is None:  return
    req= ay_util_msgs.srv.SetPUIRequest()
    req.name= 'START_BTN_LED'
    req.action= req.ON if is_start_on else req.OFF
    self.srvp_set_pui(req)
    req= ay_util_msgs.srv.SetPUIRequest()
    req.name= 'STOP_BTN_LED'
    req.action= req.ON if is_stop_on else req.OFF
    self.srvp_set_pui(req)

  def Cleanup(self):
    if self.is_sim:  return
    self.TurnOffLEDAll()

  #robot_ros_running: True or False
  #NOTE: Renamed from WaitForURROSRunning
  def WaitForRobotROSRunning(self, robot_ros_running, timeout=20):
    if self.is_sim:  return True
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.robot_ros_running != robot_ros_running:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForRobotROSRunning timeout.'
        return False
    return True


