#!/usr/bin/python
#\file    dxlg_gui.py
#\brief   DxlGripper GUI control panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Nov.08, 2017
import roslib; roslib.load_manifest('ay_py')
from ay_py.tool.py_gui import RunTerminalTab

if __name__=='__main__':
  E= 'Enter'
  terminals= [
    ('main',[
      ('Init(USB0)',(':all','ros',E,'dxlgripper0',E)),
      ('Init(USB1)',(':all','ros',E,'dxlgripper1',E)),
      ('Exit',':close') ]),
    ('roscore',[
      (':pair', ('roscore',['roscore',E]),
                ('kill',['C-c']) )  ]),
    ('Joystick',[
      (':pair', ('js0',['rosrun joy joy_node _dev:=/dev/input/js0',E]),
                ('kill',['C-c']) ),
      (':pair', ('js1',['rosrun joy joy_node _dev:=/dev/input/js1',E]),
                ('kill',['C-c']) )  ]),
    ('Monitor-joy',[
      (':pair', ('echo-joy',['rostopic echo /joy',E]),
                ('kill',['C-c']) )  ]),
    ('Test_DxlGripper',[
      ('test',['rosrun ay_py dxlg1.py',E])  ]),
    ('rviz',[
      (':pair', ('rviz',['rviz',E]),
                ('kill',['C-c']) )  ]),
    ('time',[
      (':pair', ('time',['rosrun ay_vision disp_rostime',E]),
                ('kill',['C-c']) )  ]),
    ('aypi11',[
      (':pair', ('stream',['ssh aypi11 "./stream.sh"',E]),
                ('stop',[E]) ),
      ('reboot',['ssh aypi11 "sudo reboot"',E]),
      ('shutdown',['ssh aypi11 "sudo halt -p"',E])  ]),
    ('fv11',[
      (':pair', ('start',['roslaunch ay_vision visual_skin_2fay11a2.launch',E]),
                ('kill',['C-c']) )  ]),
    ('JoyStickDemo',[
      (':pair', ('start',['rosrun ay_util fix_usb_latency.sh',E,'rosrun ay_trick direct_run.py "robot \'dxlg\'" j',E]),
                ('quit',['q',E]) )  ]),
    #('aypi3',[
      #(':pair', ('stream',['ssh hm@aypi3 "./stream2.sh"',E]),
                #('stop',[E]) ),
      #('reboot',['ssh hm@aypi3 "sudo reboot"',E])  ]),
    #('monitor1',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8080/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    #('monitor2',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi3:8081/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    ]
  exit_command= [E,'C-c']
  RunTerminalTab('DxlGripper Launcher',terminals,exit_command)
