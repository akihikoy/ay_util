#!/bin/bash
#\file    fix_usb_latency.sh
#\brief   Fix the latency issue of USB.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.01, 2018

#Usage(1): $ ./fix_usb_latency.sh ttyUSB0
#Usage(2): $ rosrun ay_util fix_usb_latency.sh ttyUSB0
#Usage(3):
#  In order to let a normal user use this script without a root privelege,
#  $ sudo ln -s `rospack find ay_util`/scripts/fix_usb_latency.sh /sbin/
#  $ sudo visudo
#  (To give a user akihikoy the permission:)
#   akihikoy ALL=PASSWD: ALL, NOPASSWD: /sbin/fix_usb_latency.sh
#  (To give a group dialout the permission:)
#   %dialout ALL=PASSWD: ALL, NOPASSWD: /sbin/fix_usb_latency.sh
#  $ sudo /sbin/fix_usb_latency.sh ttyUSB0      #No password requested.

dev=ttyUSB0
if [ $# -ge 1 ];then
  dev=$1
fi
if [ -f /sys/bus/usb-serial/devices/$dev/latency_timer ] && [ `cat /sys/bus/usb-serial/devices/$dev/latency_timer` -gt 1 ];then
  echo "Fixing the latency issue of $dev..."
  echo 1 | sudo tee /sys/bus/usb-serial/devices/$dev/latency_timer
fi
