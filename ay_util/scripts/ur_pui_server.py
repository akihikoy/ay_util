#!/usr/bin/python
#\file    ur_pui_server.py
#\brief   Physical UI (LED light, beep, button) server for UR robots.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    May.13, 2023
import roslib
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_util_msgs')
#import rostopic
from ay_py.core import InsertDict, LoadYAML
from ay_py.ros.base import SetupServiceProxy
import sys
import threading
import numpy as np
import rospy
#import std_msgs.msg
#import std_srvs.srv
#import sensor_msgs.msg
import ay_util_msgs.srv
try:
  roslib.load_manifest('ur_dashboard_msgs')
  import ur_dashboard_msgs.msg
  roslib.load_manifest('ur_msgs')
  import ur_msgs.msg
  import ur_msgs.srv
except Exception as e:
  print e


class TURPhysicalUIServer(object):

  def __init__(self, node_name='ur_pui_server', config=None, hz=50, is_sim=False):
    config_base= {
        'STATE_LED_RED': 0,
        'STATE_LED_YELLOW': 1,
        'STATE_LED_GREEN': 2,
        'STATE_BEEP': 3,
        'START_BTN_LED': 4,
        'STOP_BTN_LED': 5,
      }
    #if config is not None:  InsertDict(config_base, config)
    if config is not None:  config_base= config
    self.config= config_base

    self.node_name= node_name
    self.is_sim= is_sim

    self.srvp_ur_set_io= None

    self.state= {name:False for name in self.config.iterkeys()}
    self.pattern_threads= {name:dict(thread=None,running=False) for name in self.config.iterkeys()}
    self.hz= hz

    #self.thread_topics_hz= None
    #self.thread_topics_hz_running= False

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def Connect(self, timeout=6.0, with_thread=True):
    if self.is_sim:  return
    self.srvp_ur_set_io= SetupServiceProxy('/ur_hardware_interface/set_io', ur_msgs.srv.SetIO, persistent=False, time_out=timeout)
    rospy.Service('~set_pui', ay_util_msgs.srv.SetPUI, self.SetPUI)

  def Disconnect(self):
    if self.is_sim:  return
    self.StopAllPatternThreads()
    self.TurnOffAll()
    self.srvp_ur_set_io= None  #TODO: srvp_ur_set_io may be used from other thread, so the LED may be turned on before this.

  #int8 fun, int8 pin, float32 state
  #fun: ur_msgs.srv.SetIORequest.{FUN_SET_DIGITAL_OUT,FUN_SET_FLAG,FUN_SET_ANALOG_OUT,FUN_SET_TOOL_VOLTAGE}
  #state: ur_msgs.srv.SetIORequest.{STATE_OFF,STATE_ON}
  def SetURIO(self, fun, pin, state):
    if self.srvp_ur_set_io is None:  return
    return self.srvp_ur_set_io(ur_msgs.srv.SetIORequest(fun, pin, state)).success

  def SetByPin(self, pin, is_on):
    state= ur_msgs.srv.SetIORequest.STATE_ON if is_on else ur_msgs.srv.SetIORequest.STATE_OFF
    return self.SetURIO(ur_msgs.srv.SetIORequest.FUN_SET_DIGITAL_OUT, pin, state)

  def SetByName(self, name, is_on, update_state=True):
    pin= self.config[name]
    res= self.SetByPin(pin, is_on)
    if update_state:  self.state[name]= is_on
    return res

  def TurnOffAll(self, update_state=True):
    self.StopAllPatternThreads()
    for name in self.config.iterkeys():
      self.SetByName(name, False, update_state=update_state)

  def StopPatternThread(self, name):
    self.pattern_threads[name]['running']= False
    if self.pattern_threads[name]['thread'] is not None:
      self.pattern_threads[name]['thread'].join()
      self.pattern_threads[name]['thread']= None

  def StopAllPatternThreads(self):
    for name in self.config.iterkeys():
      self.StopPatternThread(name)

  def PatternLoop(self, th_info, name, t_start, on_off_traj, dt_traj, n_repeat):
    f_set_pin= lambda is_on: self.SetByPin(self.config[name],is_on)
    subt_traj= [0.0]+np.cumsum(dt_traj).tolist()
    t_traj= [k*subt_traj[-1]+t for k in range(n_repeat) for t in subt_traj[:-1]]+[n_repeat*subt_traj[-1]]
    on_off_traj= list(on_off_traj)*n_repeat+[on_off_traj[-1]]
    assert(len(on_off_traj)==len(t_traj))
    rate_adjuster= rospy.Rate(self.hz)
    while not rospy.is_shutdown() and th_info['running']:
      t_now= (rospy.Time.now()-t_start).to_sec()
      if t_now>=t_traj[-1]:
        f_set_pin(on_off_traj[-1])
        th_info['running']= False
        break
      try:
        idx= max(0, next(i for i,t in enumerate(t_traj) if t>t_now)-1)
      except StopIteration:
        idx= len(t_traj)-1
      f_set_pin(on_off_traj[idx])
      rate_adjuster.sleep()
    f_set_pin(self.state[name])
    th_info['thread']= None

  #req: ay_util_msgs.srv.SetPUIRequest
  def SetPUI(self, req):
    print 'set_pui: received req=',req
    if req.action==req.OFF_ALL:  self.TurnOffAll()
    else:
      if req.name not in self.config:
        print 'set_pui: Warning: req.name {} not in config'.format(req.name)
        return ay_util_msgs.srv.SetPUIResponse(False)
      self.StopPatternThread(req.name)
      if   req.action==req.ON :  self.SetByName(req.name, is_on=True)
      elif req.action==req.OFF:  self.SetByName(req.name, is_on=False)
      elif req.action==req.PATTERN:
        assert(len(req.on_off_traj)==len(req.dt_traj))
        th_info= self.pattern_threads[req.name]
        thread= threading.Thread(name=req.name,
                                target=lambda th_info=th_info,name=req.name,t_start=req.start,on_off_traj=req.on_off_traj,dt_traj=req.dt_traj,n_repeat=req.n_repeat:self.PatternLoop(th_info, name, t_start, on_off_traj, dt_traj, n_repeat))
        th_info['running']= True
        th_info['thread']= thread
        th_info['thread'].start()
    return ay_util_msgs.srv.SetPUIResponse(True)

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
  node_name= get_arg('-node_name=',get_arg('--node_name=','ur_pui_server'))
  config_yaml= get_arg('-config_yaml=',get_arg('--config_yaml=',None))
  config_yaml_section= get_arg('-config_section=',get_arg('--config_section=','UR_STATUS_PINS'))
  config= None
  if config_yaml is not None and config_yaml!='':
    try:
      config= LoadYAML(config_yaml)[config_yaml_section]
      print 'Loaded config from YAML={}, section={}'.format(config_yaml,config_yaml_section)
      print 'config=',config
    except Exception:
      print 'Failed to load config from YAML={}, section={}'.format(config_yaml,config_yaml_section)
      print 'Default config is used.'
  hz= get_arg('-hz=',get_arg('--hz=',50))

  server= TURPhysicalUIServer(node_name=node_name, config=config, hz=hz, is_sim=is_sim)
  server.InitNode()
  server.Connect()
  rospy.spin()
