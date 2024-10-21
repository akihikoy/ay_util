#!/usr/bin/python
#\file    ur_io_watchdog.py
#\brief   Watch digital inputs and send digital outputs conditionally.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Oct.21, 2024
import roslib
roslib.load_manifest('ay_py')
import std_msgs.msg
from ay_py.core import InsertDict, LoadYAML
from ay_py.ros.base import TROSUtil
import rospy
import sys
import threading
try:
  roslib.load_manifest('ur_dashboard_msgs')
  import ur_dashboard_msgs.msg
  roslib.load_manifest('ur_msgs')
  import ur_msgs.msg
  import ur_msgs.srv
except Exception as e:
  print e

'''
Example of conditions:

UR_IO_WATCHDOG:
  'ConditionalEStop':
    PHASE_START: 'move_to_grasp'
    PHASE_END_LIST: ['grasped', 'control_error']
    INPUT_IDX: 6
    INPUT_VALID_VALUE: true
    VALID_OUT_IDX_LIST: [6]
    VALID_OUT_VALUE_LIST: [true]
    INVALID_OUT_IDX_LIST: [6]
    INVALID_OUT_VALUE_LIST: [false]
'''


class TURIOWatchDog(TROSUtil):

  CONDITION_NAME_INIT= '*INIT'
  CONDITION_NAME_EXIT= '*EXIT'
  PHASE_ANY= '*ANY'

  def __init__(self, node_name='ur_io_watchdog', conditions={}, is_sim=False):
    super(TURIOWatchDog,self).__init__()

    self.conditions= conditions

    self.node_name= node_name
    self.is_sim= is_sim

    self.active_conditions= []

    self.set_io_locker= threading.RLock()

  def __del__(self):
    self.Cleanup()
    if TURIOWatchDog is not None:  super(TURIOWatchDog,self).__del__()
    print 'TURIOWatchDog: done',self

  def Cleanup(self):
    if TURIOWatchDog is not None:  super(TURIOWatchDog,self).Cleanup()

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def Connect(self, timeout=6.0):
    if self.is_sim:  return
    self.AddSrvP('set_io', '/ur_hardware_interface/set_io', ur_msgs.srv.SetIO, time_out=timeout)
    self.AddSub('motion_phase','/motion_phase',
                std_msgs.msg.String, self.CallbackMotionPhase)
    self.AddSub('io_states','/ur_hardware_interface/io_states',
                ur_msgs.msg.IOStates, self.CallbackIOStates)

    self.SetAllInvalidOutputs()
    if self.CONDITION_NAME_INIT in self.conditions:
      self.SetOutputs(self.condition[self.CONDITION_NAME_INIT], is_valid=True)

    #Start PHASE_ANY conditions:
    self.active_conditions= [name for name, cond in self.conditions.iteritems()
                             if cond['PHASE_START']==self.PHASE_ANY]

  def Disconnect(self):
    if self.is_sim:  return
    self.SetAllInvalidOutputs()
    if self.CONDITION_NAME_EXIT in self.conditions:
      self.SetOutputs(self.condition[self.CONDITION_NAME_EXIT], is_valid=True)
    self.Cleanup()

  #int8 fun, int8 pin, float32 state
  #fun: ur_msgs.srv.SetIORequest.{FUN_SET_DIGITAL_OUT,FUN_SET_FLAG,FUN_SET_ANALOG_OUT,FUN_SET_TOOL_VOLTAGE}
  #state: ur_msgs.srv.SetIORequest.{STATE_OFF,STATE_ON}
  def SetURIO(self, fun, pin, state):
    return self.srvp.set_io(ur_msgs.srv.SetIORequest(fun, pin, state)).success

  def SetByPin(self, pin, is_on):
    state= ur_msgs.srv.SetIORequest.STATE_ON if is_on else ur_msgs.srv.SetIORequest.STATE_OFF
    return self.SetURIO(ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT, pin, state)

  #Set valid or invalid outputs of a condition.
  def SetOutputs(self, cond, is_valid):
    if is_valid:
      for pin, value in zip(cond['VALID_OUT_IDX_LIST'], cond['VALID_OUT_VALUE_LIST']):
        self.SetByPin(pin, value)
    else:
      for pin, value in zip(cond['INVALID_OUT_IDX_LIST'], cond['INVALID_OUT_VALUE_LIST']):
        self.SetByPin(pin, value)

  #Set invalid outputs of all conditions.
  def SetAllInvalidOutputs(self):
    for name, cond in self.conditions.iteritems():
      self.SetOutputs(cond, is_valid=False)

  def CallbackMotionPhase(self, msg):
    phase= msg.data
    active_conditions= self.active_conditions

    #Check conditions that are activated at this phase.
    for name, cond in self.conditions.iteritems():
      if cond['PHASE_START']==phase or cond['PHASE_START']==self.PHASE_ANY:
        if name not in active_conditions:
          active_conditions.append(name)

    #print 'phase: {}'.format(phase)
    #print 'PHASE_END_LIST: {}'.format({name:self.conditions[name]['PHASE_END_LIST'] for name in active_conditions})

    #Check active_conditions that are deactivated at this phase.
    remove_conditions= [name for name in active_conditions
                        if phase in self.conditions[name]['PHASE_END_LIST']
                        or (cond['PHASE_START']!=phase and cond['PHASE_END_LIST']==self.PHASE_ANY)]
    active_conditions= [name for name in active_conditions if name not in remove_conditions]

    self.active_conditions= active_conditions

    if remove_conditions is not None:
      #Set invalid values for remove_conditions
      with self.set_io_locker:
        for name in remove_conditions:
          self.SetOutputs(self.conditions[name], is_valid=False)

  def CallbackIOStates(self, msg):
    in_states= [bool(in_st.state) for in_st in msg.digital_in_states]
    active_conditions= self.active_conditions
    print 'active_conditions= {}'.format(active_conditions)

    with self.set_io_locker:
      for name in active_conditions:
        cond= self.conditions[name]
        is_valid= in_states[cond['INPUT_IDX']]==cond['INPUT_VALID_VALUE']
        print '{}: {}'.format(name, 'valid' if is_valid else 'invalid')
        self.SetOutputs(cond, is_valid)


if __name__=='__main__':
  try:
    is_sim_default= rospy.get_param('robot_code').endswith('_SIM')
  except KeyError:
    is_sim_default= False
  is_sim= True if '-sim' in sys.argv or '--sim' in sys.argv else is_sim_default
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  node_name= get_arg('-node_name=',get_arg('--node_name=','ur_io_watchdog'))
  config_yaml= get_arg('-config_yaml=',get_arg('--config_yaml=',None))
  config_yaml_section= get_arg('-config_section=',get_arg('--config_section=','UR_IO_WATCHDOG'))
  config= None
  if config_yaml is not None and config_yaml!='':
    try:
      config= LoadYAML(config_yaml)[config_yaml_section]
      print 'Loaded config from YAML={}, section={}'.format(config_yaml,config_yaml_section)
      print 'config=',config
    except Exception:
      print 'Failed to load config from YAML={}, section={}'.format(config_yaml,config_yaml_section)
      print 'The program is terminated.'
      sys.exit(1)

  watchdog= TURIOWatchDog(node_name=node_name, conditions=config, is_sim=is_sim)
  watchdog.InitNode()
  watchdog.Connect()
  rospy.spin()

