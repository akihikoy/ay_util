#!/usr/bin/python
#\file    ur_gui.py
#\brief   UR GUI control panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.25, 2018
#\version 0.2
#\date    Jun.24, 2019
#         Updated for UR3e, UR5e
import roslib; roslib.load_manifest('ay_py')
from ay_py.tool.py_gui import RunTerminalTab

if __name__=='__main__':
  E= 'Enter'
  widgets= [
    ('main',[
      ('Init',(':all','ros',E)),
      ('Exit',':close') ]),
    ('roscore',[
      (':pair', ('roscore',['roscore',E]),
                ('kill',['C-c']) )  ]),
    ('URType',':cmb',['UR3','UR3DxlG','UR3ThG','UR3_SIM','UR3DxlG_SIM','UR3ThG_SIM', 'UR3e','UR3eThG','UR3e_SIM','UR3eThG_SIM',
    'UR5e','UR5eThG','UR5e_SIM','UR5eThG_SIM']),
    ('JoyUSB',':radio',['js0','js1']),
    ('System',[
      (':pair', ('run',['roslaunch ay_util ur_selector.launch robot_code:={URType} jsdev:=/dev/input/{JoyUSB}',E]),
                ('kill',['C-c']) )  ]),
    ('DxlUSB',':radio',['USB0','USB1']),
    ('Dynamixel',[
      ('fix_usb',['rosrun ay_util fix_usb_latency.sh tty{DxlUSB}',E])  ]),
    #('Mikata',[
      #('survo-off',['rosrun ay_py mikata_off.py',E]),
      #('reboot',['rosrun ay_py mikata_reboot.py',E])  ]),
    ('Monitor-joy',[
      (':pair', ('echo-joy',['rostopic echo /joy',E]),
                ('kill',['C-c']) )  ]),
    ('rviz',[
      (':pair', ('rviz',['rviz',E]),
                ('kill',['C-c']) )  ]),
    ('time',[
      (':pair', ('time',['rosrun ay_vision disp_rostime',E]),
                ('kill',['C-c']) )  ]),
    #('m100',[
      #(':pair', ('start',['roslaunch ay_3dvision sentis_tof_m100_s.launch',E]),
                #('kill',['C-c']) )  ]),
    #('pose_est',[
      #(':pair', ('start',['roslaunch ay_3dvision rt_pose_estimator_m100.launch',E]),
                #('kill',['C-c']) )  ]),
    ('PiID',':radio',['pi13','pi14','pi15']),
    ('aypiX',[
      (':pair', ('stream',['ssh ayg@ay{PiID} "./stream.sh"',E]),
                ('stop',[E]) ),
      ('reboot',['ssh ayg@ay{PiID} "sudo reboot"',E]),
      ('shutdown',['ssh ayg@ay{PiID} "sudo halt -p"',E])  ]),
    ('aypiX-2',[
      ('config',['ssh ayg@ay{PiID} "./conf_elp.sh"',E]),  ]),
    ('fingervision',[
      (':pair', ('start',['roslaunch ay_fv_extra fv_{PiID}.launch',E]),
                ('kill',['C-c']) )  ]),
    #('aypi13-no3',[
      #(':pair', ('stream',['ssh aypi13 "./stream_no3.sh"',E]),
                #('stop',[E]) )  ]),
    #('monitor11-no3',[
      #(':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi13:8082/?action=stream&dummy=file.mjpg"',E]),
                #('kill',['C-c']) )  ]),
    ('aypi10',[
      (':pair', ('stream1',['ssh ayg@aypi10 "./stream1.sh"',E]),
                ('stop',[E]) ),
      #(':pair', ('stream2',['ssh ayg@aypi10 "./stream.sh"',E]),
                #('stop',[E]) ),
      ('reboot',['ssh ayg@aypi10 "sudo reboot"',E]),
      ('shutdown',['ssh ayg@aypi10 "sudo halt -p"',E])  ]),
    ('aypi10-2',[
      ('config1',['ssh ayg@aypi10 "./conf_elp1.sh"',E]),
      #('config2',['ssh ayg@aypi10 "./conf_elp.sh"',E])
      ]),
    ('monitor10',[
      (':pair', ('run',['~/prg/testl/cv/capture.out "http://aypi10:8080/?action=stream&dummy=file.mjpg"',E]),
                ('kill',['C-c']) )  ]),
    ('segm_obj',[
      (':pair', ('start',['roslaunch ay_vision segm_obj3.launch',E]),
                ('kill',['C-c']) )  ]),
    #('fv10',[
      #(':pair', ('start',['roslaunch ay_fv_extra fv_pi10.launch',E]),
                #('kill',['C-c']) )  ]),
    ('JoyStickDemo',[
      (':pair', ('start(real)',['rosrun ay_trick direct_run.py "robot \'{URType}\',\'/dev/tty{DxlUSB}\'" "fv.fv \'on\'" "viz \'\'" j',E]),
                ('quit',['q',E]) ),
      (':pair', ('start(k-sim)',['rosrun ay_trick direct_run.py "robot \'urs\'" j',E]),
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
  RunTerminalTab('UR Launcher',widgets,exit_command)
