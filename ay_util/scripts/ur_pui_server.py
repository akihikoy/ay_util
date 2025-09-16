#!/usr/bin/python3
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
import copy
import rospy
import std_msgs.msg
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
  print(e)


class TURPhysicalUIServer(object):

  def __init__(self, node_name='ur_pui_server', config=None, is_sim=False):
    config_base= {
      'OUTPUTS': {
        'PINS': {
          'STATE_LED_RED': 0,
          'STATE_LED_YELLOW': 1,
          'STATE_LED_GREEN': 2,
          'STATE_BEEP': 3,
          'START_BTN_LED': 4,
          'STOP_BTN_LED': 5,
        },
        'SIGNAL_ON': True,
        'OUT_HZ': 50,  #Output rate (Hz)
        },
      'INPUTS': {
        'PINS': {
          'E_STOP': 0,
          'FENCE': 1,
          'START_BTN': 4,
          'STOP_BTN': 5,
          'SRC_CNT_LS': 6,  #SRC_CONTAINER Limit Switch (when connected).
        },
        'SIGNAL_ON': True,
        'DT_FILTER': 0.05,  #Filter for the input signal.
        },
      }
    #if config is not None:  InsertDict(config_base, config)
    # NOTE: We do not merge config by InsertDict as it does not provide a way of zero base config.
    if config is not None:  config_base= config
    self.config= config_base
    self.in_pins= self.config['INPUTS']['PINS']
    self.out_pins= self.config['OUTPUTS']['PINS']
    self.dt_filter= self.config['INPUTS']['DT_FILTER']

    self.node_name= node_name
    self.is_sim= is_sim

    self.pub_in= {}
    self.srvp_ur_set_io= None
    self.sub_io_states= None

    self.in_state= {name:False for name in self.in_pins.keys()}
    self.in_state_change_time= None
    self.prev_din= None
    self.out_state= {name:False for name in self.out_pins.keys()}
    self.out_pattern_threads= {name:dict(thread=None,running=False) for name in self.out_pins.keys()}
    self.out_hz= self.config['OUTPUTS']['OUT_HZ']

    self.io_states= None
    self.pub_io_states= None

    #self.thread_topics_hz= None
    #self.thread_topics_hz_running= False

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def Connect(self, timeout=6.0, with_thread=True):
    if self.is_sim:  return

    for name in self.in_pins.keys():
      self.pub_in[name]= rospy.Publisher(f'~inputs/{name}', std_msgs.msg.Bool, queue_size=10)

    self.srvp_ur_set_io= SetupServiceProxy('/ur_hardware_interface/set_io', ur_msgs.srv.SetIO, persistent=False, time_out=timeout)
    rospy.Service('~set_pui', ay_util_msgs.srv.SetPUI, self.SetPUI)
    self.sub_io_states= rospy.Subscriber('/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, self.IOStatesCallback)
    #Publisher of io_states for sending a fake digital input.
    self.pub_io_states= rospy.Publisher('/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, queue_size=10)
    rospy.Service('~send_fake_din', ay_util_msgs.srv.SetFlag, self.SendFakeDigitalInSignal)

  def Disconnect(self):
    if self.is_sim:  return
    self.StopAllPatternThreads()
    self.TurnOffAll()
    self.srvp_ur_set_io= None  #TODO: srvp_ur_set_io may be used from other thread, so the LED may be turned on before this.
    if self.sub_io_states is not None:
      self.sub_io_states.unregister()
      self.sub_io_states= None
    for pub in self.pub_in:
      pub.unregister()
    self.pub_in= []

    if self.pub_io_states is not None:
      self.pub_io_states.unregister()
    self.io_states= None
    self.pub_io_states= None

  def IOStatesCallback(self, msg):
    self.io_states= msg
    din= {name: self.io_states.digital_in_states[pin].state==self.config['INPUTS']['SIGNAL_ON']
          for name,pin in self.in_pins.items()}

    current_time= rospy.Time.now().to_sec()
    if self.in_state_change_time is None or self.prev_din is None:
      self.in_state_change_time= {name: current_time for name in self.in_pins.keys()}
    else:
      #Update the time stamp when din and prev_din are different.
      self.in_state_change_time= {name: current_time if din[name]!=self.prev_din[name]
                                    else ch_time
                                  for name, ch_time in self.in_state_change_time.items()}
    self.in_state= {name: din[name] if current_time-ch_time>self.dt_filter
                      else self.in_state[name]
                    for name, ch_time in self.in_state_change_time.items()}
    self.prev_din= din

    #print(self.in_state)
    for name, state in self.in_state.items():
      self.pub_in[name].publish(std_msgs.msg.Bool(state))

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
    pin= self.out_pins[name]
    res= self.SetByPin(pin, is_on)
    if update_state:  self.out_state[name]= is_on
    return res

  def TurnOffAll(self, update_state=True):
    self.StopAllPatternThreads()
    for name in self.out_pins.keys():
      self.SetByName(name, False, update_state=update_state)

  def StopPatternThread(self, name):
    self.out_pattern_threads[name]['running']= False
    if self.out_pattern_threads[name]['thread'] is not None:
      self.out_pattern_threads[name]['thread'].join()
      self.out_pattern_threads[name]['thread']= None

  def StopAllPatternThreads(self):
    for name in self.out_pins.keys():
      self.StopPatternThread(name)

  def PatternLoop(self, th_info, name, t_start, on_off_traj, dt_traj, n_repeat):
    f_set_pin= lambda is_on: self.SetByPin(self.out_pins[name],is_on)
    subt_traj= [0.0]+np.cumsum(dt_traj).tolist()
    t_traj= [k*subt_traj[-1]+t for k in range(n_repeat) for t in subt_traj[:-1]]+[n_repeat*subt_traj[-1]]
    on_off_traj= list(on_off_traj)*n_repeat+[on_off_traj[-1]]
    assert(len(on_off_traj)==len(t_traj))
    rate_adjuster= rospy.Rate(self.out_hz)
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
    f_set_pin(self.out_state[name])
    th_info['thread']= None

  #req: ay_util_msgs.srv.SetPUIRequest
  def SetPUI(self, req):
    print('set_pui: received req=',req)
    if req.action==req.OFF_ALL:  self.TurnOffAll()
    else:
      if req.name not in self.out_pins:
        print('set_pui: Warning: req.name {} not in config'.format(req.name))
        return ay_util_msgs.srv.SetPUIResponse(False)
      self.StopPatternThread(req.name)
      if   req.action==req.ON :  self.SetByName(req.name, is_on=True)
      elif req.action==req.OFF:  self.SetByName(req.name, is_on=False)
      elif req.action==req.PATTERN:
        assert(len(req.on_off_traj)==len(req.dt_traj))
        th_info= self.out_pattern_threads[req.name]
        thread= threading.Thread(name=req.name,
                                target=lambda th_info=th_info,name=req.name,t_start=req.start,on_off_traj=req.on_off_traj,dt_traj=req.dt_traj,n_repeat=req.n_repeat:self.PatternLoop(th_info, name, t_start, on_off_traj, dt_traj, n_repeat))
        th_info['running']= True
        th_info['thread']= thread
        th_info['thread'].start()
    return ay_util_msgs.srv.SetPUIResponse(True)

  def SendFakeDigitalInSignal(self, req):
    name, is_on= req.name, req.is_on
    if name not in self.in_pins:  return ay_util_msgs.srv.SetFlagResponse(False)
    signal_idx= self.in_pins[name]
    signal_trg= self.config['INPUTS']['SIGNAL_ON']
    if self.io_states is not None:
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
    return ay_util_msgs.srv.SetFlagResponse(True)


if __name__=='__main__':
  try:
    is_sim_default= rospy.get_param('robot_code').endswith('_SIM')
  except KeyError:
    is_sim_default= False
  is_sim= (True if '-sim' in sys.argv or '--sim' in sys.argv else
           (False if '-real' in sys.argv or '--real' in sys.argv else is_sim_default))
  def get_arg(opt_name, default):
    exists= [a.startswith(opt_name) for a in sys.argv]
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  node_name= get_arg('-node_name=',get_arg('--node_name=','ur_pui_server'))
  config_yaml= get_arg('-config_yaml=',get_arg('--config_yaml=',None))
  config_yaml_section= get_arg('-config_section=',get_arg('--config_section=','UR_PHYSICAL_UI'))
  config= None
  if config_yaml is not None and config_yaml!='':
    try:
      config= LoadYAML(config_yaml)[config_yaml_section]
      print('Loaded config from YAML={}, section={}'.format(config_yaml,config_yaml_section))
      print('config=',config)
    except Exception:
      print('Failed to load config from YAML={}, section={}'.format(config_yaml,config_yaml_section))
      print('Default config is used.')

  server= TURPhysicalUIServer(node_name=node_name, config=config, is_sim=is_sim)
  server.InitNode()
  server.Connect()
  rospy.spin()
