#!/usr/bin/python
#\file    script_node_client.py
#\brief   Client module for SCRIPT ROS Node of ay_trick.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jan.07, 2024
import roslib
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_trick_msgs')
from ay_py.core import CPrint
import sys
import rospy
import std_msgs.msg
import std_srvs.srv
import ay_trick_msgs.msg
import ay_trick_msgs.srv

'''
Client module for SCRIPT ROS Node of ay_trick.
'''
class TScriptNodeClient(object):
  def __init__(self):
    self.script_node_status_names= {
      ay_trick_msgs.msg.ROSNodeMode.BUSY: 'BUSY',
      ay_trick_msgs.msg.ROSNodeMode.READY: 'READY',
      ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING: 'PROGRAM_RUNNING' }

    self.script_node_running= None
    self.script_node_status= None
    self.script_node_status_stamp= None

  def ConnectToScriptNode(self, timeout=20):
    self.connected_to_script_node= False
    try:
      rospy.wait_for_service('/ros_node/wait_finish', timeout=timeout)
      rospy.wait_for_service('/ros_node/get_result_as_yaml', timeout=timeout)
      rospy.wait_for_service('/ros_node/get_attr_as_yaml', timeout=timeout)
      rospy.wait_for_service('/ros_node/set_attr_with_yaml', timeout=timeout)
      self.srvp_wait_finish= rospy.ServiceProxy('/ros_node/wait_finish', std_srvs.srv.Empty, persistent=False)
      self.srvp_get_result_as_yaml= rospy.ServiceProxy('/ros_node/get_result_as_yaml', ay_trick_msgs.srv.GetString, persistent=False)
      self.srvp_get_attr_as_yaml= rospy.ServiceProxy('/ros_node/get_attr_as_yaml', ay_trick_msgs.srv.GetAttrAsString, persistent=False)
      self.srvp_set_attr_with_yaml= rospy.ServiceProxy('/ros_node/set_attr_with_yaml', ay_trick_msgs.srv.SetAttrWithString, persistent=False)
      self.connected_to_script_node= True
    except rospy.ROSException as e:
      CPrint(4,'Error in ConnectToScriptNode:', e)
      self.srvp_wait_finish= None
      self.srvp_get_result_as_yaml= None
      self.srvp_get_attr_as_yaml= None
      self.srvp_set_attr_with_yaml= None
    self.pub_cmd= rospy.Publisher('/ros_node/command', std_msgs.msg.String, queue_size=10)
    self.pub_key= rospy.Publisher('/ros_node/stdin', std_msgs.msg.String, queue_size=10)
    self.sub_stdout= rospy.Subscriber('/ros_node/stdout', std_msgs.msg.String, self.StdOutCallback)
    self.script_node_status= None
    self.sub_script_node_status= rospy.Subscriber('/ros_node/node_status', ay_trick_msgs.msg.ROSNodeMode, self.ScriptNodeStatusCallback)

  def RunFGScript(self, cmd):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.srvp_wait_finish()  #Wait for previously executed scripts.
    self.pub_cmd.publish(std_msgs.msg.String(cmd))
    print '###INFO/RunFGScript###',cmd
    self.srvp_wait_finish()

  def RunBGScript(self, cmd):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.srvp_wait_finish()  #Wait for previously executed scripts.
    self.pub_cmd.publish(std_msgs.msg.String(cmd))
    print '###INFO/RunBGScript###',cmd

  def SendString(self, key):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.pub_key.publish(std_msgs.msg.String(key))

  def WaitForBGScript(self):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.srvp_wait_finish()

  def GetScriptResult(self):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    res= self.srvp_get_result_as_yaml()
    if res.success:
      return yamlload(res.result, Loader=YLoader)
    else:
      return None

  def StdOutCallback(self, msg):
    #sys.stdout.write(msg.data)
    #sys.stdout.flush()
    pass

  def ScriptNodeStatusCallback(self, msg):
    self.script_node_status= msg.mode
    self.script_node_status_stamp= rospy.Time.now()

  #status: ay_trick_msgs.msg.ROSNodeMode.{BUSY,READY,PROGRAM_RUNNING}
  def WaitForScriptNodeStatus(self, status, timeout=10):
    if not hasattr(self,'script_node_status'):
      CPrint(4,'Script node status is not subscribed.')
      return False
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.script_node_status != status:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForScriptNodeStatus timeout.'
        return False
    return True

  def GetScriptNodeStatus(self):
    return self.script_node_status

