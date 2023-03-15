#!/usr/bin/python
#\file    joy_fv_panel.py
#\brief   Virtual joystick controller.
#\usage   This script does not control directly the robot.
#         Use with a controller such as the j command of ay_trick.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Mar.15, 2023
import roslib
roslib.load_manifest('ay_py')
import os,sys
import rospy
import rospkg
from joy_fv import TJoyEmulator
from ay_py.core import CPrint
from ay_py.tool.py_panel import TSimplePanel, InitPanelApp, RunPanelApp, AskYesNoDialog, QtCore, QtGui
from ay_py.ros.base import SetupServiceProxy

if __name__=='__main__':
  pm= TJoyEmulator()
  set_joy= lambda kind,value=None,is_active=0: pm.SetJoy(kind,value,is_active)

  widgets_joy= {
    'btn_activate': (
      'buttonchk',{
        'text':('Activate','Deactivate'),
        'size_policy': ('expanding', 'fixed'),
        #'font_size_range': (8,24),
        #'onstatuschanged':lambda w,obj,status:None,
        #'onclick':(lambda w,obj:None,
                   #lambda w,obj:None )
        }),
    'btn_push': (
      'buttonchk',{
        'text':('Push','Stop'),
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('trackf_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('trackf_off') )}),
    'btn_hold': (
      'buttonchk',{
        'text':('Hold','Stop'),
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('hold_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('hold_off') )}),
    'btn_pick': (
      'buttonchk',{
        'text':('Pick','Stop'),
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('pickup_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('pickup_off') )}),
    'label_pitch': (
      'label',{
        'text': 'Pitch',
        'size_policy': ('expanding', 'minimum')}),
    'joy_pitch': (
      'virtual_joystick',{
        'kind':'vbox',
        'stick_color':[128,255,128],
        'size_policy': ('minimum', 'expanding'),
        'onstickmoved': lambda w,obj:set_joy('pitch',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_xy': (
      'label',{
        'text': 'XY',
        'size_policy': ('expanding', 'minimum')}),
    'joy_xy': (
      'virtual_joystick',{
        'kind':'circle',
        'stick_color':[128,128,255],
        'onstickmoved': lambda w,obj:set_joy('xy',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_z': (
      'label',{
        'text': '  Z  ',
        'size_policy': ('expanding', 'minimum')}),
    'joy_z': (
      'virtual_joystick',{
        'kind':'vbox',
        'stick_color':[128,128,255],
        'size_policy': ('minimum', 'expanding'),
        'onstickmoved': lambda w,obj:set_joy('z',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_yaw': (
      'label',{
        'text': 'Yaw',
        'size_policy': ('minimum', 'minimum')}),
    'joy_yaw': (
      'virtual_joystick',{
        'kind':'hbox',
        'stick_color':[255,128,128],
        'size_policy': ('expanding','minimum'),
        'onstickmoved': lambda w,obj:set_joy('yaw',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'label_grip': (
      'label',{
        'text': 'Gripper',
        'size_policy': ('minimum', 'minimum')}),
    'joy_grip': (
      'virtual_joystick',{
        'kind':'hbox',
        'stick_color':[255,128,128],
        'size_policy': ('expanding','minimum'),
        'onstickmoved': lambda w,obj:set_joy('grip',obj.position(),is_active=w.widgets['btn_activate'].isChecked()), }),
    'btn_grip_open': (
      'button',{
        'text': '[ + ]',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:(
                      #run_script_during_joy('grip_plus'),
                      set_joy('open'),
                      ), }),
    'btn_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    }

  layout_joy= (
    'boxv',None, (
      ('grid',None,(
        ('btn_activate',0,0,1,2),
        ('label_pitch',1,0,'center'), ('label_xy',1,1,'center'), ('label_z',1,2,'center'),
        ('joy_pitch',2,0), ('joy_xy',2,1), ('joy_z',2,2),
        (('boxh',None,('label_yaw','joy_yaw')),3,0,1,3),
        (('boxh',None,('label_grip','joy_grip','btn_grip_open')),4,0,1,3),
        )),
      ('boxh',None, ('btn_push','btn_hold','btn_pick')),
      ))

  layout_main= (
    'boxv',None,(
      layout_joy,
      'btn_exit',
      ))

  app= InitPanelApp()
  win_size= (500,500)
  panel= TSimplePanel('Virtual Joystick', size=win_size, font_height_scale=300.0)
  panel.AddWidgets(widgets_joy)
  panel.Construct(layout_main)

  rospy.init_node('joy_fv_panel')
  #rospy.sleep(0.1)
  pm.StartVirtualJoyStick()
  RunPanelApp()
