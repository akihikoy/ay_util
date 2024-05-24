#!/usr/bin/python
#\file    topic_monitor.py
#\brief   Utility to monitor frequency of arrivals of topics.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jul.07, 2023
# Usage:
#   Basically instantiating the TTopicMonitor object in a program is assumed.
# A simple example use is implemented in the main section of this script working as a ROS node.
# Try:
#   $ rosrun ay_util topic_monitor.py --topics=Robot:/joint_states,Gripper:/gripper_driver/joint_states
import roslib
roslib.load_manifest('ay_py')
from ay_py.core import CPrint, InsertDict
import rospy
import rostopic
import threading

class TTopicMonitor(object):
  def __init__(self, topics_to_monitor):
    self.topics_to_monitor= topics_to_monitor
    self.topic_hz= None
    self.topics_hz= dict()
    self.sub_status_topics= dict()
    self.thread_topics_hz= None
    self.thread_topics_hz_running= False
    self.thread_topics_hz_callback= None

  def StartTopicMonitorThread(self):
    self.ConnectToStatusTopics()
    self.thread_topics_hz_running= True
    self.thread_topics_hz= threading.Thread(name='topics_hz', target=self.UpdateTopicsHzThread)
    self.thread_topics_hz.start()

  def StopTopicMonitorThread(self):
    self.thread_topics_hz_running= False
    if self.thread_topics_hz is not None:
      self.thread_topics_hz.join()
    self.DisconnectStatusTopics()

  def ConnectToStatusTopics(self, window_size=20):
    self.topic_hz= rostopic.ROSTopicHz(window_size)
    self.DisconnectStatusTopics()
    self.sub_status_topics= dict()
    for key,topic in self.topics_to_monitor.iteritems():
      self.sub_status_topics[key]= rospy.Subscriber(topic, rospy.AnyMsg, self.topic_hz.callback_hz, callback_args=topic)

  def DisconnectStatusTopics(self):
    for key,sub in self.sub_status_topics.iteritems():
      sub.unregister()
    self.topics_hz= dict()

  def UpdateTopicsHzThread(self):
    rate= rospy.Rate(0.5)
    while self.thread_topics_hz_running and not rospy.is_shutdown():
      for key,topic in self.topics_to_monitor.iteritems():
        hz= self.topic_hz.get_hz(topic)
        hz= hz[0] if hz is not None else None
        self.topics_hz[key]= hz
      if self.thread_topics_hz_callback is not None:
        self.thread_topics_hz_callback()
      rate.sleep()

  def GetTopicHz(self, key):
    if key not in self.topics_hz:  return None
    return self.topics_hz[key]

  def IsActive(self, key):
    return self.GetTopicHz(key) is not None

if __name__=='__main__':
  import sys
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  topics_to_monitor= get_arg('-topics=',get_arg('--topics=',''))
  topics_to_monitor= None if topics_to_monitor=='' else {p.split(':')[0]:p.split(':')[-1] for p in topics_to_monitor.split(',')}
  assert(topics_to_monitor is not None)

  rospy.init_node('topic_monitor_test')

  tm= TTopicMonitor(topics_to_monitor)
  try:
    tm.StartTopicMonitorThread()
    rate_adjuster= rospy.Rate(1)
    while not rospy.is_shutdown():
      state= {key:(tm.IsActive(key),tm.GetTopicHz(key)) for key,topic in topics_to_monitor.iteritems()}
      rate_adjuster.sleep()
      print state
  except Exception as e:
    print 'Exception:',e
  finally:
    tm.StopTopicMonitorThread()
