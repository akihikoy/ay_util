#!/usr/bin/python
#\file    proc_manager_ur.py
#\brief   Provides TURManager, TProcessManagerUR.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Feb.21, 2024
#         Separated from ur_dashboard_gui.py
import roslib
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_trick_msgs')
from ay_py.core import CPrint, InsertDict
from ay_py.ros.base import SetupServiceProxy
import threading
import copy
import rospy
import std_msgs.msg
import std_srvs.srv
import ay_trick_msgs.msg
import ay_trick_msgs.srv
import ay_util_msgs.srv
try:
  roslib.load_manifest('ur_dashboard_msgs')
  import ur_dashboard_msgs.msg
  roslib.load_manifest('ur_msgs')
  import ur_msgs.msg
  import ur_msgs.srv
except Exception as e:
  print e
from proc_manager_gui import TProcessManagerGUIBase

#Simple module to operate UR.
class TURManager(object):
  def __init__(self, is_sim=False):
    self.ur_robot_mode_names= {
      ur_dashboard_msgs.msg.RobotMode.POWER_OFF: 'POWER_OFF',
      ur_dashboard_msgs.msg.RobotMode.BOOTING: 'BOOTING',
      ur_dashboard_msgs.msg.RobotMode.IDLE: 'IDLE',
      ur_dashboard_msgs.msg.RobotMode.RUNNING: 'RUNNING' }
    self.ur_safety_mode_names= {
      ur_dashboard_msgs.msg.SafetyMode.NORMAL: 'NORMAL',
      ur_dashboard_msgs.msg.SafetyMode.PROTECTIVE_STOP: 'PROTECTIVE_STOP',
      ur_dashboard_msgs.msg.SafetyMode.ROBOT_EMERGENCY_STOP: 'ROBOT_EMERGENCY_STOP',
      ur_dashboard_msgs.msg.SafetyMode.FAULT: 'FAULT' }

    self.is_sim= is_sim

    #self.panel= None
    self.io_states= None
    self.srvp_ur_dashboard= {}
    #self.srvp_ur_set_io= None
    self.srvp_ur_set_pui= None
    self.ur_robot_mode= None
    self.ur_safety_mode= None
    self.ur_program_running= None

  def ConnectToURDashboard(self, timeout=6.0, with_thread=True):
    if not self.is_sim:
      services= ['power_on', 'power_off', 'brake_release', 'play', 'stop', 'shutdown', 'unlock_protective_stop', 'restart_safety', 'close_safety_popup']
      if with_thread:
        threads= {}
        for service in services:
          threads[service]= threading.Thread(name=service, target=lambda service=service:(self.srvp_ur_dashboard.__setitem__(service,SetupServiceProxy('/ur_hardware_interface/dashboard/{0}'.format(service), std_srvs.srv.Trigger, persistent=False, time_out=timeout))))
          threads[service].start()
        #threads['srvp_ur_set_io']= threading.Thread(name='srvp_ur_set_io', target=lambda:(setattr(self,'srvp_ur_set_io',SetupServiceProxy('/ur_hardware_interface/set_io', ur_msgs.srv.SetIO, persistent=False, time_out=timeout))))
        #threads['srvp_ur_set_io'].start()
        threads['srvp_ur_set_pui']= threading.Thread(name='srvp_ur_set_pui', target=lambda:(setattr(self,'srvp_ur_set_pui',SetupServiceProxy('/ur_pui_server/set_pui', ay_util_msgs.srv.SetPUI, persistent=False, time_out=timeout))))
        threads['srvp_ur_set_pui'].start()
        for name,th in threads.iteritems():  th.join()
      else:
        for service in services:
          self.srvp_ur_dashboard[service]= SetupServiceProxy('/ur_hardware_interface/dashboard/{0}'.format(service), std_srvs.srv.Trigger, persistent=False, time_out=timeout)
        #self.srvp_ur_set_io= SetupServiceProxy('/ur_hardware_interface/set_io', ur_msgs.srv.SetIO, persistent=False, time_out=timeout)
        self.srvp_ur_set_pui= SetupServiceProxy('/ur_pui_server/set_pui', ay_util_msgs.srv.SetPUI, persistent=False, time_out=timeout)

    #Connect to io_states to publish a fake io_states.
    self.pub_io_states= rospy.Publisher('/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, queue_size=10)

    self.sub_io_states= rospy.Subscriber('/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, self.IOStatesCallback)

    if not self.is_sim:
      self.sub_robot_mode= rospy.Subscriber('/ur_hardware_interface/robot_mode', ur_dashboard_msgs.msg.RobotMode, self.RobotModeCallback)
      self.sub_safety_mode= rospy.Subscriber('/ur_hardware_interface/safety_mode', ur_dashboard_msgs.msg.SafetyMode, self.SafetyModeCallback)
      self.sub_program_running= rospy.Subscriber('/ur_hardware_interface/robot_program_running', std_msgs.msg.Bool, self.ProgramRunningCallback)

  def DisconnectUR(self):
    if self.is_sim:  return
    #self.srvp_ur_set_io= None
    self.srvp_ur_set_pui= None  #TODO: srvp_ur_set_pui may be used from other thread, so the LED may be turned on before this.
    self.srvp_ur_dashboard= {}

    if not self.is_sim:
      self.sub_robot_mode.unregister()
      self.sub_safety_mode.unregister()
      self.sub_program_running.unregister()
      self.sub_robot_mode= None
      self.sub_safety_mode= None
      self.sub_program_running= None

    self.pub_io_states.unregister()
    self.pub_io_states= None

  def RunURDashboard(self, service):
    if service not in self.srvp_ur_dashboard:
      return False
    res= self.srvp_ur_dashboard[service]()
    return res.success

  ##int8 fun, int8 pin, float32 state
  ##fun: ur_msgs.srv.SetIORequest.{FUN_SET_DIGITAL_OUT,FUN_SET_FLAG,FUN_SET_ANALOG_OUT,FUN_SET_TOOL_VOLTAGE}
  ##state: ur_msgs.srv.SetIORequest.{STATE_OFF,STATE_ON}
  #def SetURIO(self, fun, pin, state):
    #if self.srvp_ur_set_io is None:  return
    #return self.srvp_ur_set_io(ur_msgs.srv.SetIORequest(fun, pin, state)).success

  def IOStatesCallback(self, msg):
    self.io_states= msg

  def RobotModeCallback(self, msg):
    self.ur_robot_mode= msg.mode

  def SafetyModeCallback(self, msg):
    self.ur_safety_mode= msg.mode
    if self.ur_safety_mode!=ur_dashboard_msgs.msg.SafetyMode.NORMAL and self.ur_program_running:
      self.RunURDashboard('stop')
      self.WaitForProgramRunning(False, timeout=1.0)

  def ProgramRunningCallback(self, msg):
    self.ur_program_running= msg.data

  #mode: ur_dashboard_msgs.msg.RobotMode.{POWER_OFF,BOOTING,IDLE,RUNNING}
  def WaitForRobotMode(self, mode, timeout=20):
    if self.is_sim:  return True
    if mode==ur_dashboard_msgs.msg.RobotMode.POWER_OFF and\
      self.ur_robot_mode in (ur_dashboard_msgs.msg.RobotMode.BOOTING,
                             ur_dashboard_msgs.msg.RobotMode.IDLE,
                             ur_dashboard_msgs.msg.RobotMode.RUNNING):  return False
    if mode==ur_dashboard_msgs.msg.RobotMode.IDLE and\
      self.ur_robot_mode in (ur_dashboard_msgs.msg.RobotMode.RUNNING,):  return False
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    #print 'WaitForRobotMode',mode,self.ur_robot_mode
    while self.ur_robot_mode != mode:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForRobotMode timeout.'
        return False
    #print '  done.',mode,self.ur_robot_mode
    return True

  #mode: ur_dashboard_msgs.msg.SafetyMode.{NORMAL,PROTECTIVE_STOP,ROBOT_EMERGENCY_STOP,FAULT}
  def WaitForSafetyMode(self, mode, timeout=20):
    if self.is_sim:  return True
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.ur_safety_mode != mode:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForSafetyMode timeout.'
        return False
    return True

  #program_running: True or False
  def WaitForProgramRunning(self, program_running, timeout=20):
    if self.is_sim:  return True
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.ur_program_running != program_running:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForProgramRunning timeout.'
        return False
    return True

  def SendFakeDigitalInSignal(self, signal_idx, signal_trg):
    if self.ur_robot_mode is not None and self.io_states is not None:
      msg= copy.deepcopy(self.io_states)
    else:
      msg= ur_msgs.msg.IOStates()
      msg.digital_in_states= [ur_msgs.msg.Digital(pin,False) for pin in range(18)]
      msg.digital_out_states= [ur_msgs.msg.Digital(pin,False) for pin in range(18)]
      msg.flag_states= [ur_msgs.msg.Digital(pin,False) for pin in range(2)]
      msg.analog_in_states= [ur_msgs.msg.Analog(pin,0,0) for pin in range(2)]
      msg.analog_out_states= [ur_msgs.msg.Analog(pin,0,0) for pin in range(2)]
    msg.digital_in_states[signal_idx]= ur_msgs.msg.Digital(signal_idx,signal_trg)
    self.pub_io_states.publish(msg)


class TProcessManagerUR(TProcessManagerGUIBase, TURManager):

  #Classify the current status.
  #Overwritten for UR.
  def GetStatus(self):
    #FIXME: In which case, we should set EMERGENCY?
    status= self.UNDEFINED
    if not self.robot_ros_running:
      status= self.NO_CORE_PROGRAM
    else:
      if self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.FAULT:
        status= self.FAULT
      elif self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.ROBOT_EMERGENCY_STOP:
        status= self.ROBOT_EMERGENCY_STOP
      elif self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.PROTECTIVE_STOP:
        status= self.PROTECTIVE_STOP
      elif self.ur_safety_mode == ur_dashboard_msgs.msg.SafetyMode.NORMAL:
        if self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.POWER_OFF:
          status= self.POWER_OFF
        elif self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.BOOTING:
          status= self.BOOTING
        elif self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.IDLE:
          status= self.IDLE
        elif self.ur_robot_mode == ur_dashboard_msgs.msg.RobotMode.RUNNING:
          if not self.ur_program_running:
            status= self.TORQUE_ENABLED
          else:
            if not self.script_node_running:
              status= self.ROBOT_READY
            elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.READY:
              status= self.WAIT_REQUEST
            elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING:
              status= self.PROGRAM_RUNNING
    return status

  def __init__(self, node_name='ur_dashboard', topics_to_monitor=None, is_sim=False):
    TProcessManagerGUIBase.__init__(self, node_name=node_name, topics_to_monitor=topics_to_monitor, is_sim=is_sim)
    TURManager.__init__(self, is_sim)

  def ConnectToURDashboard(self, timeout=6.0, with_thread=True):
    TURManager.ConnectToURDashboard(self, timeout=timeout, with_thread=with_thread)
    self.srvp_set_pui= self.srvp_ur_set_pui

  def DisconnectUR(self):
    if self.is_sim:  return
    TProcessManagerGUIBase.Cleanup(self)
    TURManager.DisconnectUR(self)
    self.srvp_set_pui= self.srvp_ur_set_pui

