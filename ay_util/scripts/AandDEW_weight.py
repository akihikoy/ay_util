#!/usr/bin/python
#\file    AandDEW_weight.py
#\brief   ROS node to read and publish data from the A and D EW weight.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.26, 2019
#TODO: It should be published with a time stamp.

import roslib;
roslib.load_manifest('std_msgs')
import rospy
import std_msgs.msg
import sys
import serial

if __name__=='__main__':
  rospy.init_node('weight')
  dev= sys.argv[1] if len(sys.argv)>1 else '/dev/ttyUSB0'
  baudrate= int(sys.argv[2]) if len(sys.argv)>2 else 2400

  ser= serial.Serial(dev,baudrate,serial.SEVENBITS,serial.PARITY_EVEN)

  pub_value= rospy.Publisher('~value', std_msgs.msg.Float64, queue_size=1)
  pub_raw= rospy.Publisher('~raw', std_msgs.msg.String, queue_size=1)

  try:
    while not rospy.is_shutdown():
      raw= ser.readline()
      print '"{raw}" ({l})'.format(raw=repr(raw), l=len(raw))
      if len(raw)!=17:  continue
      value= float(raw[3:12])

      pub_value.publish(value)
      pub_raw.publish(raw)

  finally:
    ser.close()
