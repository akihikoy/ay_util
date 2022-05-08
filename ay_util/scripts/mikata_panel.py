#!/usr/bin/python
#\file    mikata_panel.py
#\brief   Mikata Arm + FV demo panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    May.07, 2022

import roslib
roslib.load_manifest('ay_trick_msgs')
import sys
import rospy
import rospkg
import ay_trick_msgs.msg
import ay_trick_msgs.srv

roslib.load_manifest('ay_py')
from ay_py.core import CPrint
from ay_py.tool.py_panel import TSimplePanel, InitPanelApp, RunPanelApp, AskYesNoDialog, QtCore, QtGui
from ay_py.ros.base import SetupServiceProxy
import subprocess
import threading
import std_msgs.msg
import std_srvs.srv
import sensor_msgs.msg

class TProcessManager(QtCore.QObject):
  def __init__(self):
    QtCore.QObject.__init__(self)
    self.procs= {}

  #command: list of command and arguments.
  def RunFGProcess(self, command, shell=False):
    p= subprocess.Popen(command, shell=shell)
    p.wait()

  #command: list of command and arguments.
  def RunBGProcess(self, name, command, shell=False):
    self.TerminateBGProcess(name)
    p= subprocess.Popen(command, shell=shell)
    self.procs[name]= p

  def TerminateBGProcess(self, name):
    if name not in self.procs:
      print 'No process named',name
      return
    self.procs[name].terminate()
    self.procs[name].wait()
    #TODO: wait(): It is safer to have timeout.  For ver<3.3, implement like:
    #while p.poll() is None:
      #print 'Process still running...'
      #time.sleep(0.1)
    del self.procs[name]

  def TerminateAllBGProcesses(self):
    for name,p in self.procs.iteritems():
      print 'Terminating',name
      p.terminate()
      p.wait()
      #TODO: wait(): It is safer to have timeout.  For ver<3.3, implement like:
      #while p.poll() is None:
        #print 'Process still running...'
        #time.sleep(0.1)
    self.procs= {}

  #WARNING: This is not safe.  When killing roscore, rosmaster is still alive.
  def KillBGProcess(self, name):
    if name not in self.procs:
      print 'No process named',name
      return
    self.procs[name].kill()
    self.procs[name].wait()
    #TODO: wait(): It is safer to have timeout.  For ver<3.3, implement like:
    #while p.poll() is None:
      #print 'Process still running...'
      #time.sleep(0.1)
    del self.procs[name]

  def IsBGProcessRunning(self, name):
    return self.procs[name].poll() is None

  def InitNode(self):
    rospy.init_node('mikata_panel')
    rospy.sleep(0.1)

  def ConnectToScriptNode(self, timeout=5.0):
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
    print '###DEBUG/RunFGScript###',cmd
    self.srvp_wait_finish()

  def RunBGScript(self, cmd):
    if not hasattr(self,'connected_to_script_node'):
      CPrint(4,'Not connected to the script node.')
      return
    self.srvp_wait_finish()  #Wait for previously executed scripts.
    self.pub_cmd.publish(std_msgs.msg.String(cmd))
    print '###DEBUG/RunBGScript###',cmd

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
  joy_state.axes[5], joy_state.axes[2]  #RT, LT: Negative and positive multiplier [1(base),-1]
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
  def SetJoy(self, kind, value=None):
    if not hasattr(self,'pub_joy'):
      CPrint(4,'Joystick publisher is not ready.')
      return
    joy_state= self.joy_state
    #reset:
    joy_state.axes[:]= [0.0]*6
    joy_state.axes[2]= 1.0
    joy_state.axes[5]= 1.0
    joy_state.buttons[:]= [0]*16
    if kind=='reset':
      pass
    elif kind=='xy':
      joy_state.axes[0]= -value[0]  #Y, -WX(roll), Gripper step
      joy_state.axes[1]= value[1]  #X, WY(pitch)
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='z':
      joy_state.axes[4]= value[0]  #Z
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='pitch':
      joy_state.axes[1]= value[0]  #X, WY(pitch)
      joy_state.buttons[4]= 1  #LB (0:position, 1:orientation)
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='grip':
      joy_state.axes[0]= -value[0]  #Y, -WX(roll), Gripper step
      joy_state.buttons[0]= 1  #A (gripper mode)
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='quit':
      joy_state.buttons[7]= 1  #START button (quit)
    elif kind=='trackf_on':
      joy_state.buttons[2]= 1  #X (fv.trackf2)
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='trackf_off':
      joy_state.buttons[2]= 1  #X (fv.trackf2)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='pickup_on':
      joy_state.buttons[3]= 1  #Y (fv.pickup2b)
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='pickup_off':
      joy_state.buttons[3]= 1  #Y (fv.pickup2b)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    elif kind=='hold_on':
      joy_state.buttons[13]= 1  #d-pad, UP (fv.hold)
      joy_state.buttons[5]= 1  #RB (0:inactive, 1:active)
    elif kind=='hold_off':
      joy_state.buttons[13]= 1  #d-pad, UP (fv.hold)
      joy_state.buttons[5]= 0  #RB (0:inactive, 1:active)
    self.pub_joy.publish(joy_state)


if __name__=='__main__':
  fullscreen= True if '-fullscreen' in sys.argv or '--fullscreen' in sys.argv else False
  is_sim= True if '-sim' in sys.argv or '--sim' in sys.argv else False

  RVIZ_CONFIG= rospkg.RosPack().get_path('ay_util')+'/config/mikata.rviz'
  #Parameters:
  config={
    'JoyUSB': 'js0',
    'DxlUSB': 'USB0',
    'FV_L_DEV': '/media/video_fv1',
    #'ShutdownRobotAfterUse': False,
    'TIMEOUT': 20.0,
    }

  #List of commands (name: [[command/args],'fg'/'bg']).
  cmds= {
    'roscore': ['roscore','bg'],
    #'shutdown_pc': ['/sbin/shutdown -h now','fg'],
    'fix_usb': ['sudo /sbin/fix_usb_latency.sh tty{DxlUSB}','fg'],
    'mikata_driver': ['roslaunch ay_util mikata_rot_real2.launch jsdev:=/dev/input/{JoyUSB} dev:=/dev/tty{DxlUSB}','bg'],
    'fv_l': ['roslaunch ay_fv_extra fvp_mikata.launch','bg'],
    'config_fv_l': ['rosrun fingervision conf_cam2.py {FV_L_DEV} "file:CameraParams:0:`rospack find ay_fv_extra`/config/fvp_mikata.yaml"','fg'],
    #'show_fv_l': ['rosservice call /fingervision/fvp_mikata/show_windows','fg'],
    #'hide_fv_l': ['rosservice call /fingervision/fvp_mikata/hide_windows','fg'],
    'ay_trick_ros': ['rosrun ay_trick ros_node.py','bg'],
    #'rviz': ['rosrun rviz rviz -d {0}'.format(RVIZ_CONFIG),'bg'],
    }
  if is_sim:
    cmds['mikata_driver']= ['roslaunch ay_util mikata_rot_ksim.launch','bg']
    for c in ('fix_usb','fv_l','config_fv_l'):
      cmds[c][1]= None
  for key in cmds.iterkeys():
    if isinstance(cmds[key][0],str):
      cmds[key][0]= cmds[key][0].format(**config).split(' ')

  pm= TProcessManager()
  run_cmd= lambda name: pm.RunBGProcess(name,cmds[name][0]) if cmds[name][1]=='bg' else\
                        pm.RunFGProcess(cmds[name][0]) if cmds[name][1]=='fg' else\
                        None
  stop_cmd= lambda name: pm.TerminateBGProcess(name)

  #List of script commands (name: [[script/args],'fg'/'bg']).
  scripts= {
    'setup': ['mikata.setup "mikata",True','fg'],
    'joy': ['j','bg'],
    #'stop_joy': ['q','fg'],
    #'move_to_init': ['ct.robot.MoveToQ({Q_INIT},dt=3.0,blocking=True)','fg'],
    'move_to_init': ['mikata.move_to_init','fg'],
    'move_to_park': ['mikata.move_to_park','fg'],
    'grip_minus': ['ct.robot.MoveGripper(ct.robot.GripperPos()-0.015, blocking=True)','fg'],
    'grip_plus':  ['fv.open ct.robot.Arm, ct.robot.GripperPos()+0.015, True','fg'],
    }
  if is_sim:
    scripts['setup']= ['mikata.setup "sim",True','fg']
  for key in scripts.iterkeys():
    if isinstance(scripts[key][0],str):
      scripts[key][0]= scripts[key][0].format(**config)

  run_script= lambda name: pm.RunBGScript(scripts[name][0]) if scripts[name][1]=='bg' else\
                           pm.RunFGScript(scripts[name][0]) if scripts[name][1]=='fg' else\
                           None

  set_joy= lambda kind,value=None: pm.SetJoy(kind,value)
  #stop_joy= lambda: run_script('stop_joy')
  stop_joy= lambda: pm.SendString('q')

  widgets_main= {
    #'status_signal_bar1': (
      #'primitive_painer',{
        #'color': (0,255,0),
        #'onstatuschanged':lambda w,obj,status:(
                      #obj.setPaintColor({'red':(255,0,0),'green':(0,255,0),'yellow':(255,255,0),None:(128,128,128)}[
                        #pm.status_color[0] if len(pm.status_color)>0 else None]), ),
        #'margin': (0,0),
        #'minimum_size': (None,20),
        #'maximum_size': (None,20),
        #'size_policy': ('expanding', 'fixed')}),
    #'status_signal_bar2': (
      #'primitive_painer',{
        #'color': (0,255,0),
        #'onstatuschanged':lambda w,obj,status:(
                      #obj.setPaintColor({'red':(255,0,0),'green':(0,255,0),'yellow':(255,255,0),None:(128,128,128)}[
                        #pm.status_color[1] if len(pm.status_color)>1 else \
                          #pm.status_color[0] if len(pm.status_color)>0 else None]), ),
        #'margin': (0,0),
        #'minimum_size': (None,20),
        #'maximum_size': (None,20),
        #'size_policy': ('expanding', 'fixed')}),
    #'rviz': (  #NOTE: To be replaced by RViz widget.
      #'primitive_painer',{
        #'shape': 'square',
        #'color': (0,120,240),
        #'margin': (0.1,0.1),
        #'size_policy': ('expanding', 'expanding')}),
    'rviz': (
      'rviz',{
        'config': RVIZ_CONFIG,
        'size_policy': ('expanding', 'expanding')}),
    #'status_textbox': (
      #'textedit',{
        #'read_only': True,
        #'font_size_range': (6,24),
        #'text': 'Status Text Box',
        #'onstatuschanged': UpdateStatusTextBox, }),
    #'spacer_h1': ('spacer', {
        #'w': 400,
        #'h': 1,
        #'size_policy': ('fixed', 'fixed')      }),
    'btn_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('fixed', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    'btn_start': (
      'buttonchk',{
        'text':('START','QUIT'),
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:(
                      #run_cmd('roscore'),
                      #rospy.wait_for_service('/rosout/get_loggers', timeout=5.0),
                      #run_cmd('fix_usb'),
                      #pm.InitNode(),
                      #pm.StartVirtualJoyStick(),
                      run_cmd('mikata_driver'),
                      run_cmd('fv_l'),
                      run_cmd('config_fv_l'),
                      run_cmd('ay_trick_ros'),
                      w.widgets['rviz'].setup(),
                      pm.ConnectToScriptNode(timeout=config['TIMEOUT']),
                      pm.WaitForScriptNodeStatus(ay_trick_msgs.msg.ROSNodeMode.READY),
                      rospy.sleep(0.2),
                      run_script('setup'),
                      run_script('joy'),
                      ),
                   lambda w,obj:(
                      stop_joy(),
                      run_script('move_to_park'),
                      stop_cmd('ay_trick_ros'),
                      stop_cmd('fv_l'),
                      stop_cmd('mikata_driver'),
                      ) )}),
    'btn_init_pose': (
      'button',{
        'text': 'Init Pose',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:(
                      stop_joy(),
                      run_script('move_to_init'),
                      run_script('joy'),
                      ), }),
    #'btn_init2': (
      #'buttonchk',{
        #'text':('(2)Power on robot','[2]Power off robot'),
        #'onstatuschanged':lambda w,obj,status:(
                      #obj.setEnabled(status in (pm.POWER_OFF,pm.TORQUE_ENABLED,pm.ROBOT_READY)),
                      #obj.setChecked(status in (pm.TORQUE_ENABLED,pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP) ),
                      #status==pm.ROBOT_EMERGENCY_STOP and (
                        #stop_cmd('ur_gripper'),
                        #pm.RunURDashboard('stop'),
                        #pm.WaitForProgramRunning(False),
                        #pm.RunURDashboard('power_off'),
                        #),
                      #),
        #'onclick':(lambda w,obj:(
                      #run_cmd('ur_gripper'),
                      #pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.POWER_OFF),
                      #pm.RunURDashboard('power_on'),
                      #pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.IDLE),
                      #pm.RunURDashboard('brake_release'),
                      #pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.RUNNING),
                      #rospy.sleep(0.2),
                      #pm.RunURDashboard('play'),
                      #pm.WaitForProgramRunning(True),
                     #),
                   #lambda w,obj:(
                      #stop_cmd('ur_gripper'),
                      #pm.RunURDashboard('stop'),
                      #pm.WaitForProgramRunning(False),
                      #pm.RunURDashboard('power_off'),
                     #) )}),
    'btn_push': (
      'buttonchk',{
        'text':('Push','Stop'),
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('trackf_on'),
                   lambda w,obj:set_joy('trackf_off') )}),
    'btn_hold': (
      'buttonchk',{
        'text':('Hold','Stop'),
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('hold_on'),
                   lambda w,obj:set_joy('hold_off') )}),
    'btn_pick': (
      'buttonchk',{
        'text':('Pick','Stop'),
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('pickup_on'),
                   lambda w,obj:set_joy('pickup_off') )}),
    #'btn_go_initial': (
      #'button',{
        #'text': 'Initial pose',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.WAIT_REQUEST,)) ),
        #'onclick': lambda w,obj: run_script('moveto_obsrv'), }),
    #'btn_go_release': (
      #'button',{
        ##'text': 'Dump to container',
        #'text': 'Above container',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.WAIT_REQUEST,)) ),
        #'onclick': lambda w,obj: (
                      #run_script('moveto_wait'),
                      ##pm.WaitForScriptNodeStatus(ay_trick_msgs.msg.ROSNodeMode.READY), #NOTE:These are commented since it does not accept START/STOP GUI buttons.
                      ##run_script('grip_release'),
                      #), }),
    #'btn_grip_open': (
      #'button',{
        #'text': 'Gripper open',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.WAIT_REQUEST,)) ),
        #'onclick': lambda w,obj: run_script('grip_release'), }),
    #'btn_go_store': (
      #'button',{
        #'text': 'Storing pose',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.WAIT_REQUEST,)) ),
        #'onclick': lambda w,obj: run_script('moveto_store'), }),
    #'btn_start': (
      #'button',{
        #'text': 'START',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROGRAM_RUNNING,)) ),
        #'onclick': lambda w,obj: pm.SendFakeDigitalInDignal(config['START_BTN']['SIGNAL_IDX'], config['START_BTN']['SIGNAL_ON']), }),
    #'btn_stop': (
      #'button',{
        #'text': 'STOP',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROGRAM_RUNNING,)) ),
        #'onclick': lambda w,obj: pm.SendFakeDigitalInDignal(config['STOP_BTN']['SIGNAL_IDX'], config['STOP_BTN']['SIGNAL_ON']), }),
    'label_pitch': (
      'label',{
        'text': 'P',
        'size_policy': ('expanding', 'minimum')}),
    'joy_pitch': (
      'virtual_joystick',{
        'kind':'vbox',
        'stick_color':[128,255,128],
        'size_policy': ('minimum', 'expanding'),
        'onstickmoved': lambda w,obj:set_joy('pitch',obj.position()), }),
    'label_xy': (
      'label',{
        'text': 'XY',
        'size_policy': ('expanding', 'minimum')}),
    'joy_xy': (
      'virtual_joystick',{
        'kind':'circle',
        'stick_color':[128,128,255],
        'onstickmoved': lambda w,obj:set_joy('xy',obj.position()), }),
    'label_z': (
      'label',{
        'text': 'Z',
        'size_policy': ('expanding', 'minimum')}),
    'joy_z': (
      'virtual_joystick',{
        'kind':'vbox',
        'stick_color':[128,128,255],
        'size_policy': ('minimum', 'expanding'),
        'onstickmoved': lambda w,obj:set_joy('z',obj.position()), }),
    'label_grip': (
      'label',{
        'text': 'Gripper',
        'size_policy': ('minimum', 'minimum')}),
    #'joy_grip': (
      #'virtual_joystick',{
        #'kind':'hbox',
        #'stick_color':[255,128,128],
        #'size_policy': ('expanding','minimum'),
        #'onstickmoved': lambda w,obj:set_joy('grip',obj.position()), }),
    'btn_grip_plus': (
      'button',{
        'text': '[ + ]',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:(
                      stop_joy(),
                      run_script('grip_plus'),
                      run_script('joy'),
                      ), }),
    'btn_grip_minus': (
      'button',{
        'text': '[ - ]',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:(
                      stop_joy(),
                      run_script('grip_minus'),
                      run_script('joy'),
                      ), }),
    }

  #layout_init= (
    #'grid',None,(
      #('btn_init1',0,0), ('btn_init2',0,1),
      #('btn_init3',1,0), (('boxv',None,('btn_reset_estop','btn_shutdown_ur','btn_shutdown_pc')),1,1),
      #))
  #layout_config= (
    #'boxv',None,(
      #('boxv',None,(
        #'label_target_item',
        #'target_item',)),
      #'spacer_c1',
      #('boxv',None,(
        #'label_conveyor_speed',
        #'conveyor_speed',
        #'slider_conveyor_speed',)),
      #'spacer_c2',
      #))
  #layout_operation= (
    #'boxv',None,(
      #'btn_op_normal',
      #'btn_op_step',
      #('boxh',None, ('label_motion_speed','motion_speed')),
      #('boxh',None, ('btn_go_initial','btn_go_release','btn_go_store','btn_grip_open')),
      #('boxh',None, ('btn_start','btn_stop')),
      #))
  layout_main= (
    'boxh',None,(
      ('boxv',None, ('rviz',('boxh',None,('btn_exit','btn_start')))),
      ('boxv',None, (
        ('grid',None,(
          ('label_pitch',0,0,'center'), ('label_xy',0,1,'center'), ('label_z',0,2,'center'),
          ('joy_pitch',1,0), ('joy_xy',1,1), ('joy_z',1,2),
          #(('boxh',None,('label_grip','joy_grip')),2,0,1,3),
          (('boxh',None,('label_grip','btn_grip_plus','btn_grip_minus')),2,0,1,3),
          )),
        ('boxh',None, ('btn_init_pose','btn_push','btn_hold','btn_pick')),
        )),
      ))

  app= InitPanelApp()
  win_size= (800,400)
  if fullscreen:  #NOTE: fullscreen mode will work only with Qt5.
    print 'Screen size:', app.screens()[0].size()
    screen_size= app.screens()[0].size()
    win_size= (screen_size.width(),screen_size.height())
  panel= TSimplePanel('Mikata Operation Panel', size=win_size, font_height_scale=300.0)
  panel.AddWidgets(widgets_main)
  panel.Construct(layout_main)
  #for tab in panel.layouts['maintab'].tab:
    #tab.setFont(QtGui.QFont('', 24))
  #panel.layouts['maintab'].tabs.setFont(QtGui.QFont('', 18))
  #panel.showFullScreen()
  #panel.showMaximized()

  #ROS system initialization.
  run_cmd('roscore')
  pm.InitNode()
  rospy.wait_for_service('/rosout/get_loggers', timeout=config['TIMEOUT'])
  run_cmd('fix_usb')
  pm.StartVirtualJoyStick()
  panel.close_callback= lambda event: (
      pm.TerminateAllBGProcesses(),  #including roscore
      #stop_cmd('roscore'),
      True)[-1]

  RunPanelApp()
