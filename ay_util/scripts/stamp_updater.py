#!/usr/bin/python
#\file    stamp_updater.py
#\brief   Convert the time stamp of header to the current time.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.13, 2024
import roslib
import rospy
import rostopic

def Callback(msg, pub):
  if hasattr(msg,'header') and hasattr(msg.header,'stamp'):
    msg.header.stamp= rospy.Time.now()
  pub.publish(msg)

if __name__ == '__main__':
  rospy.init_node('stamp_updater')

  input_topic= rospy.get_param('~in', '/input_topic_name')
  output_topic= rospy.get_param('~out', '/output_topic_name')
  msg_class,_,_= rostopic.get_topic_class(input_topic)

  pub= rospy.Publisher(output_topic, msg_class, queue_size=10)
  rospy.Subscriber(input_topic, msg_class, lambda msg:Callback(msg,pub))
  rospy.spin()
