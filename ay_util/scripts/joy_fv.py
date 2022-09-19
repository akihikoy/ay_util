#!/usr/bin/python
#\file    joy_fv.py
#\brief   Joy emulator.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.19, 2022
import roslib
roslib.load_manifest('sensor_msgs')
import rospy
import sensor_msgs.msg

class TJoyEmulator(object):
  def __init__(self):
    pass

  def StartVirtualJoyStick(self):
    self.pub_joy= rospy.Publisher('/joy', sensor_msgs.msg.Joy, queue_size=10)
    self.joy_state= sensor_msgs.msg.Joy()
    self.joy_state.axes= [0.0]*6
    self.joy_state.buttons= [0]*16

  '''
  joy_state.axes[0]  #Y, -WX(roll), Gripper step
  joy_state.axes[1]  #X, WY(pitch)
  joy_state.axes[3]  #WZ(yaw)
  joy_state.axes[4]  #Z
  joy_state.axes[5]  #RT: Negative multiplier [1(base),-1]
  joy_state.axes[2]  #LT: Positive multiplier [1(base),-1]
  #multiplier= (1.0+joy_state.axes[5])*0.5 + (1.0-joy_state.axes[2])*2.0
  joy_state.buttons[7]  #START button (quit)
  joy_state.buttons[0]  #A (gripper mode)
  joy_state.buttons[1]  #B (fv.tracko)
  joy_state.buttons[2]  #X (fv.trackf2)
  joy_state.buttons[3]  #Y (fv.pickup2b)
  joy_state.buttons[4]  #LB (0:position, 1:orientation)
  joy_state.buttons[5]  #RB (0:inactive, 1:active)
  joy_state.buttons[11]  #d-pad, LEFT (fv.open)
  joy_state.buttons[12]  #d-pad, RIGHT (fv.grasp)
  joy_state.buttons[13]  #d-pad, UP (fv.hold)
  joy_state.buttons[14]  #d-pad, DOWN (fv.openif)
  '''
  def SetJoy(self, kind, value=None, is_active=0):
    if not hasattr(self,'pub_joy'):
      CPrint(4,'Joystick publisher is not ready.')
      return
    joy_state= self.joy_state
    #reset:
    joy_state.axes[:]= [0.0]*6
    joy_state.axes[5]= -0.5  #RT: Negative multiplier [1(base),-1]
    joy_state.axes[2]= 1.0  #LT: Positive multiplier [1(base),-1]
    joy_state.buttons[:]= [0]*16
    if kind=='reset':
      pass
    elif kind=='xy':
      joy_state.axes[0]= -1.5*value[0]  #Y, -WX(roll), Gripper step
      joy_state.axes[1]= value[1]  #X, WY(pitch)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='z':
      joy_state.axes[4]= value[0]  #Z
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='pitch':
      joy_state.axes[1]= value[0]  #X, WY(pitch)
      joy_state.buttons[4]= 1  #LB (0:position, 1:orientation)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='yaw':
      joy_state.axes[3]= -value[0]  #WZ(yaw)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='grip':
      joy_state.axes[0]= -4.0*value[0]  #Y, -WX(roll), Gripper step
      joy_state.buttons[0]= 1  #A (gripper mode)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='quit':
      joy_state.buttons[7]= 1  #START button (quit)
    elif kind=='trackf_on':
      joy_state.buttons[2]= 1  #X (fv.trackf2)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='trackf_off':
      joy_state.buttons[2]= 1  #X (fv.trackf2)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='pickup_on':
      joy_state.buttons[3]= 1  #Y (fv.pickup2b)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='pickup_off':
      joy_state.buttons[3]= 1  #Y (fv.pickup2b)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='open':
      joy_state.buttons[11]= 1  #d-pad, LEFT (fv.open)
    elif kind=='grasp_on':
      joy_state.buttons[12]= 1  #d-pad, RIGHT (fv.grasp)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='grasp_off':
      joy_state.buttons[12]= 1  #d-pad, RIGHT (fv.grasp)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='hold_on':
      joy_state.buttons[13]= 1  #d-pad, UP (fv.hold)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='hold_off':
      joy_state.buttons[13]= 1  #d-pad, UP (fv.hold)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='openif_on':
      joy_state.buttons[14]= 1  #d-pad, DOWN (fv.openif)
      joy_state.buttons[5]= is_active  #RB (0:inactive, 1:active)
    elif kind=='openif_off':
      joy_state.buttons[14]= 1  #d-pad, DOWN (fv.openif)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    self.pub_joy.publish(joy_state)

