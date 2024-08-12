#!/usr/bin/python
#\file    tf_static_forwarder.py
#\brief   Forward tf_static topic from src.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.13, 2024
import sys
import threading
import roslib
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

STfBr= None
Forwarded= {}
Locker= None

def CallbackTf(msg):
  global Forwarded
  for tfm in msg.transforms:
    do_send= False
    tf_id= tfm.header.frame_id+'-'+tfm.child_frame_id
    if tf_id not in Forwarded:
      do_send= True
    else:
      if tfm.transform.translation!=Forwarded[tf_id].transform.translation or tfm.transform.rotation!=Forwarded[tf_id].transform.rotation:
        do_send= True
    if do_send:
      print 'Forwarding: {}'.format(tf_id)
      with Locker:
        Forwarded[tf_id]= tfm
      STfBr.sendTransform(tfm)


if __name__=='__main__':
  rospy.init_node('tf_static_forwarder')
  once= '-once' in sys.argv or '--once' in sys.argv
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  src= get_arg('-src=',get_arg('--src=','tf_static_bone'))
  hz= int(get_arg('-hz=',get_arg('--hz=','10')))
  print '''{node_name}
  src: {src}
  hz: {hz}
  once: {once}
  '''.format(node_name=rospy.get_name(),src=src,hz=hz,once=once)

  STfBr= tf2_ros.StaticTransformBroadcaster()
  Locker= threading.RLock()
  sub_tf= rospy.Subscriber(src, tf2_msgs.msg.TFMessage, CallbackTf)

  if once:
    rospy.spin()
  else:
    rate_adjuster= rospy.Rate(hz)
    while not rospy.is_shutdown():
      with Locker:
        for tf_id,tfm in Forwarded.iteritems():
          STfBr.sendTransform(tfm)
      rate_adjuster.sleep()

