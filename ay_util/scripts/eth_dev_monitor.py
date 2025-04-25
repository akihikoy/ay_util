#!/usr/bin/python3
#\file    eth_dev_monitor.py
#\brief   Ethernet-connection device monitor via ping.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Apr.21, 2025
import rospy
import roslib
roslib.load_manifest('ay_py')
import std_msgs.msg
from ay_py.core import InsertDict, LoadYAML
from ay_py.ros.base import TROSUtil
import subprocess
import threading
import sys


'''
Ethernet-connection device monitor class via ping per device.
'''
class TEthDevMonitor(threading.Thread):
  '''
  name: Name of the device.
  uri: Address of the device.
  activation_delay: Delay in seconds (int) to decide if the device is active after ping success.
  interval: Interval in seconds (int) to check the device status.
  '''
  def __init__(self, name, uri, activation_delay, interval):
    super(TEthDevMonitor,self).__init__()
    self.name= name
    self.uri= uri
    self.activation_delay= activation_delay
    self.interval= interval
    self.t_active= None
    self.is_active= False
    self.running= False

  def run(self):
    rate= rospy.Rate(1.0 / self.interval)
    while self.running and not rospy.is_shutdown():
      response= subprocess.run(["ping", "-c", "1", "-W", "1", self.uri], stdout=subprocess.DEVNULL)
      if response.returncode==0:
        if self.is_active:
          pass
        else:
          if self.t_active is None:
            self.t_active= rospy.Time.now()
          if (rospy.Time.now()-self.t_active).to_sec()>=self.activation_delay:
            self.is_active= True
      else:
        self.is_active= False
        self.t_active= None
      rate.sleep()


#Ethernet-connection device monitor class via ping.
class TEthDevMonitorNode(TROSUtil):

  '''
  node_name: Name of the node.
  params: Dict of configurations.
  interval: Interval in seconds (int) to check the device status.
  '''
  def __init__(self, node_name, params):
    super(TEthDevMonitorNode,self).__init__()
    self.node_name= node_name
    self.params= params

  def __del__(self):
    self.Cleanup()
    if TEthDevMonitorNode is not None:  super(TEthDevMonitorNode,self).__del__()
    print('TEthDevMonitorNode: done',self)

  def Cleanup(self):
    if TEthDevMonitorNode is not None:  super(TEthDevMonitorNode,self).Cleanup()

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def Setup(self):
    self.interval= self.params['INTERVAL']
    self.devices_config= self.params['DEVICES']
    self.devices= {
      name: TEthDevMonitor(name, cfg['URI'], cfg['ACTIVATION_DELAY'], self.interval)
      for name, cfg in self.devices_config.items()
      }
    for name, cfg in self.devices_config.items():
      self.AddPub(name, f'/eth_dev_monitor/{name}', std_msgs.msg.Empty, queue_size=1)

  def Start(self):
    for name, device in self.devices.items():
      device.running= True
      device.start()

    rate= rospy.Rate(1.0 / self.interval)
    while not rospy.is_shutdown():
      for name, cfg in self.devices_config.items():
        if self.devices[name].is_active:
          self.pub[name].publish(std_msgs.msg.Empty())
      rate.sleep()




if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= [a.startswith(opt_name) for a in sys.argv]
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  node_name= get_arg('-node_name=',get_arg('--node_name=','eth_dev_monitor'))
  config_yaml= get_arg('-config_yaml=',get_arg('--config_yaml=',None))
  config_yaml_section= get_arg('-config_section=',get_arg('--config_section=','ETH_DEV_MONITOR'))
  #List(array) of devices consisting of list of [name(str), URI(str), activation_delay(float)].
  #dev_list overwrites config_yaml.
  dev_list= get_arg('-dev_list=',get_arg('--dev_list=',None))
  config= None
  if config_yaml is not None and config_yaml!='':
    try:
      config= LoadYAML(config_yaml)[config_yaml_section]
      print('Loaded config from YAML={}, section={}'.format(config_yaml,config_yaml_section))
      print('config=',config)
    except Exception:
      print('Failed to load config from YAML={}, section={}'.format(config_yaml,config_yaml_section))
      print('The program is terminated.')
      sys.exit(1)

  if config is None:
    config= {
      'INTERVAL': 1.0,
      'DEVICES': {
        #'robot': {'URI': 'motoman', 'ACTIVATION_DELAY': 5.0},
        #'jetson': {'URI': 'jetson', 'ACTIVATION_DELAY': 3.0},
        }
      }

  if dev_list is not None:
    devices= eval(dev_list)
    for name, uri, activation_delay in devices:
      config_dev= config['DEVICES']
      if name not in config_dev:
        config_dev[name]= {}
      config_dev[name]['URI']= uri
      config_dev[name]['ACTIVATION_DELAY']= activation_delay

  print(f'eth_dev_monitor: config={config}')

  monitor= TEthDevMonitorNode(node_name=node_name, params=config)
  monitor.InitNode()
  monitor.Setup()
  monitor.Start()

