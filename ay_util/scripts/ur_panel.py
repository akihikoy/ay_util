#!/usr/bin/python
#\file    ur_panel.py
#\brief   UR + FV demo panel.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    May.30, 2022
import roslib
roslib.load_manifest('ay_trick_msgs')
import os,sys
import subprocess
import rospy
import rospkg
import ay_trick_msgs.msg
import ay_trick_msgs.srv
from ur_dashboard_gui import *
from proc_manager import TSubProcManager
from topic_monitor import TTopicMonitor
from script_node_client import *
from joy_fv import TJoyEmulator
from ay_py.core import InsertDict, LoadYAML, SaveYAML

class TProcessManagerJoyUR(TProcessManagerUR, TJoyEmulator):
  def __init__(self, node_name='ur_panel'):  #, ur_status_pins=None
    TProcessManagerUR.__init__(self, node_name=node_name)  #, config=ur_status_pins
    TJoyEmulator.__init__(self)


class TProcessManagerJoyGen3(QtCore.QObject, TSubProcManager, TScriptNodeClient, TTopicMonitor, TJoyEmulator):
  #Definition of states:
  UNDEFINED= -10
  NO_CORE_PROGRAM= 0
  #IDLE= 3
  TORQUE_ENABLED= 4
  ROBOT_READY= 5
  WAIT_REQUEST= 6
  PROGRAM_RUNNING= 7

  onstatuschanged= QtCore.pyqtSignal(int)
  ontopicshzupdated= QtCore.pyqtSignal()

  def UpdateStatus(self):
    t_now= rospy.Time.now()
    self.gen3_ros_running= self.IsActive('Robot')
    self.script_node_running= ((t_now-self.script_node_status_stamp).to_sec() < 0.4) if self.script_node_status_stamp is not None else False

    status= self.UNDEFINED
    if not self.gen3_ros_running:
      status= self.NO_CORE_PROGRAM
    else:
      status= self.TORQUE_ENABLED
      if not self.script_node_running:
        status= self.ROBOT_READY
      elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.READY:
        status= self.WAIT_REQUEST
      elif self.script_node_status == ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING:
        status= self.PROGRAM_RUNNING

    #NOTE: Do not use yellow as it is used by other modules.
    if status in (self.UNDEFINED,):
      self.status_color= ['red']
    elif status in (self.WAIT_REQUEST,):
      self.status_color= ['green']
    elif status in (self.PROGRAM_RUNNING,):
      self.status_color= ['green']
    else:
      self.status_color= []

    if self.status != status:
      self.status= status
      self.OnStatusChanged()

  def OnStatusChanged(self):
    self.onstatuschanged.emit(self.status)

    self.TurnOffLEDAll()
    for color in self.status_color:  self.SetLEDLight(color, True)

    if self.status in (self.PROGRAM_RUNNING,):
      self.SetStartStopLEDs(True, True)

  def UpdateStatusThread(self):
    rate= rospy.Rate(20)
    while self.thread_status_update_running and not rospy.is_shutdown():
      self.UpdateStatus()
      rate.sleep()

  def StartUpdateStatusThread(self):
    self.StartTopicMonitorThread()
    self.thread_status_update_running= True
    self.thread_status_update= threading.Thread(name='status_update', target=self.UpdateStatusThread)
    self.thread_status_update.start()

  def StopUpdateStatusThread(self):
    self.thread_status_update_running= False
    if self.thread_status_update is not None:
      self.thread_status_update.join()
    self.StopTopicMonitorThread()

  def __init__(self, node_name='ur_dashboard', topics_to_monitor=None, is_sim=False):
    #config_base= {
        #'PIN_STATE_LED_RED': 0,
        #'PIN_STATE_LED_YELLOW': 1,
        #'PIN_STATE_LED_GREEN': 2,
        #'PIN_STATE_BEEP': 3,
        #'PIN_START_BTN_LED': 4,
        #'PIN_STOP_BTN_LED': 5,
      #}
    #if config is not None:  InsertDict(config_base, config)
    #self.config= config_base

    topics_to_monitor_base= {
      'Robot': '/gen3a/joint_states',
      'Gripper': '/gripper_driver/joint_states',
      }
    if topics_to_monitor is not None:  InsertDict(topics_to_monitor_base, topics_to_monitor)
    TTopicMonitor.__init__(self, topics_to_monitor_base)
    self.thread_topics_hz_callback= lambda: self.ontopicshzupdated.emit()

    QtCore.QObject.__init__(self)
    TSubProcManager.__init__(self)
    TScriptNodeClient.__init__(self)
    self.node_name= node_name
    self.is_sim= is_sim

    self.status_names= {
      self.UNDEFINED:           'UNDEFINED'           ,
      self.NO_CORE_PROGRAM:     'NO_CORE_PROGRAM'     ,
      #self.IDLE:                'IDLE'                ,
      self.TORQUE_ENABLED:      'TORQUE_ENABLED'      ,
      self.ROBOT_READY:         'ROBOT_READY'         ,
      self.WAIT_REQUEST:        'WAIT_REQUEST'        ,
      self.PROGRAM_RUNNING:     'PROGRAM_RUNNING'     ,
      }

    #self.panel= None
    self.srvp_set_pui= None
    self.status= self.UNDEFINED
    self.status_color= []  #List of 'red','green' (yellow is used by the other modules).
    self.gen3_ros_running= None

    self.thread_status_update= None
    self.thread_status_update_running= False

  def InitNode(self):
    rospy.init_node(self.node_name)
    rospy.sleep(0.1)

  def TurnOffLEDAll(self):
    self.SetLEDLight('red', False)
    self.SetLEDLight('green', False)
    #self.SetLEDLight('yellow', False)
    #self.SetBeep(False)
    self.SetStartStopLEDs(False, False)

  #color: 'red','yellow','green'
  def SetLEDLight(self, color, is_on):
    if self.is_sim:  return
    if self.srvp_set_pui is None:  return
    config_name= {'red':'STATE_LED_RED',
                  'yellow':'STATE_LED_YELLOW',
                  'green':'STATE_LED_GREEN'}[color]
    req= ay_util_msgs.srv.SetPUIRequest()
    req.name= config_name
    req.action= req.ON if is_on else req.OFF
    return self.srvp_set_pui(req)

  def SetStartStopLEDs(self, is_start_on, is_stop_on):
    if self.is_sim:  return
    if self.srvp_set_pui is None:  return
    req= ay_util_msgs.srv.SetPUIRequest()
    req.name= 'START_BTN_LED'
    req.action= req.ON if is_start_on else req.OFF
    self.srvp_set_pui(req)
    req= ay_util_msgs.srv.SetPUIRequest()
    req.name= 'STOP_BTN_LED'
    req.action= req.ON if is_stop_on else req.OFF
    self.srvp_set_pui(req)

  def Disconnect(self):
    if self.is_sim:  return
    self.TurnOffLEDAll()

  #gen3_ros_running: True or False
  def WaitForGen3ROSRunning(self, gen3_ros_running, timeout=20):
    if self.is_sim:  return True
    t_start= rospy.Time.now()
    rate= rospy.Rate(20)
    while self.gen3_ros_running != gen3_ros_running:
      rate.sleep()
      if (rospy.Time.now()-t_start).to_sec()>timeout:
        print 'WaitForGen3ROSRunning timeout.'
        return False
    return True


def UpdateProcList(pm,combobox):
  combobox.clear()
  for name,proc in pm.procs.iteritems():
    combobox.addItem('{0}/{1}'.format(name,proc.pid))


if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  robot_code= get_arg('-robot_code=',get_arg('--robot_code=','UR3e125hzDxlpY1'))
  sim_robot_code= get_arg('-sim_robot_code=',get_arg('--sim_robot_code=','UR3eDxlpY1_SIM'))
  joy_dev= get_arg('-joy_dev=',get_arg('--joy_dev=','js0'))
  dxl_dev= get_arg('-dxl_dev=',get_arg('--dxl_dev=','USB1'))
  fullscreen= True if '-fullscreen' in sys.argv or '--fullscreen' in sys.argv else False
  is_sim= True if '-sim' in sys.argv or '--sim' in sys.argv else False

  robot_mode= 'ur' if robot_code.startswith('UR') else 'gen3' if robot_code.startswith('Gen3') else None

  RVIZ_CONFIG= os.environ['HOME']+'/.rviz/default.rviz'
  #Parameters:
  config={
    'RobotCode': robot_code,
    'RobotCode_SIM': sim_robot_code,
    'JoyUSB': joy_dev,
    'DxlUSB': dxl_dev,
    'IS_SIM': is_sim,
    'FV_L_DEV': '/media/video_fv1',
    'FV_R_DEV': '/media/video_fv2',
    'FV_BASE_DIR': subprocess.check_output('rospack find ay_fv_extra'.split(' ')).strip(),
    'FV_L_CONFIG': 'config/fvp_3_l.yaml',
    'FV_R_CONFIG': 'config/fvp_3_r.yaml',
    'FV_CTRL_CONFIG': '{}/data/config/fv_ctrl.yaml'.format(os.environ['HOME']),
    #'ShutdownRobotAfterUse': False,
    'Q_INIT': [0.0357, -2.0273, 1.6516, -1.1895, -1.571, 0.0]
              if robot_mode=='ur' else
              [0.16, 0.0, 0.17, 2.0, 2.9, -0.7, 2.0] if robot_mode=='gen3'
              else None,
    'X_INIT': None if robot_mode=='ur' else
              [0.3418398342045766, -0.11518828661688434, 0.4262027524524184, 0.7042117631586926, 0.6690845996953141, 0.1738926582036271, 0.16178051335795401] if robot_mode=='gen3'
              else None,
    #'Q_PARK': [0.03572946786880493, -2.027292076741354, 1.6515636444091797, -1.1894968191729944, -1.5706136862384241, -3.1061676184283655],
    }
  print config

  #List of commands (name: [[command/args],'fg'/'bg']).
  cmds= {
    'roscore': ['roscore','bg'],
    'fix_usb': ['sudo /sbin/fix_usb_latency.sh tty{DxlUSB}','fg'],
    'robot_ros': ['roslaunch ay_util robot_selector.launch robot_code:={RobotCode} jsdev:=/dev/input/{JoyUSB} dxldev:=/dev/tty{DxlUSB} with_gripper:=false','bg'],
    'robot_gripper': ['roslaunch ay_util robot_gripper_selector.launch robot_code:={RobotCode} dxldev:=/dev/tty{DxlUSB} is_sim:={IS_SIM}','bg'],
    'ur_calib': ['roslaunch ay_util ur_calib.launch robot_code:={RobotCode}','fg'],
    'ur_pui_server': ['rosrun ay_util ur_pui_server.py','bg'],
    'fvp': ['roslaunch fingervision fvp_general.launch pkg_dir:={FV_BASE_DIR} config1:={FV_L_CONFIG} config2:={FV_R_CONFIG}','bg'],
    'fvp_file': ['roslaunch ay_fv_extra fvp_file1.launch','bg'],
    'config_fv_l': ['rosrun fingervision conf_cam2.py {FV_L_DEV} file:CameraParams:0:{FV_BASE_DIR}/{FV_L_CONFIG}','fg'],
    'config_fv_r': ['rosrun fingervision conf_cam2.py {FV_R_DEV} file:CameraParams:0:{FV_BASE_DIR}/{FV_R_CONFIG}','fg'],
    'realsense': ['roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=true depth_fps:=15 color_fps:=30 depth_width:=640 depth_height:=480 color_width:=640 color_height:=480','bg'],
    'ay_trick_ros': ['rosrun ay_trick ros_node.py','bg'],
    'rviz': ['rosrun rviz rviz -d {0}'.format(RVIZ_CONFIG),'bg'],
    'reboot_dxlg': ['roslaunch ay_util gripper_reboot.launch robot_code:={RobotCode} dxldev:=/dev/tty{DxlUSB} command:=Reboot','fg'],
    'factory_reset_dxlg': ['roslaunch ay_util gripper_reboot.launch robot_code:={RobotCode} dxldev:=/dev/tty{DxlUSB} command:=FactoryReset','fg'],
    }
  if robot_mode!='ur':
    cmds['ur_calib'][1]= None
    cmds['ur_pui_server'][1]= None
  if is_sim:
    config['RobotCode']= config['RobotCode_SIM']
    cmds['fvp']= cmds['fvp_file']
    for c in ('fix_usb','ur_calib','config_fv_l','config_fv_r','realsense'):
      cmds[c][1]= None
  for key in cmds.iterkeys():
    if isinstance(cmds[key][0],str):
      cmds[key][0]= cmds[key][0].format(**config).split(' ')

  pm= (TProcessManagerJoyUR() if robot_mode=='ur'
        else TProcessManagerJoyGen3() if robot_mode=='gen3'
        else None)
  run_cmd= lambda name: pm.RunBGProcess(name,cmds[name][0]) if cmds[name][1]=='bg' else\
                        pm.RunFGProcess(cmds[name][0]) if cmds[name][1]=='fg' else\
                        None
  stop_cmd= lambda name: pm.TerminateBGProcess(name)

  #List of script commands (name: [[script/args],'fg'/'bg']).
  scripts= {
    'setup': ['ur.setup None,True','fg'] if robot_mode=='ur'
             else ['gen3.setup None,True','fg'] if robot_mode=='gen3'
             else None,
    'joy': ['j','bg'],
    #'stop_joy': ['q','fg'],
    'move_to_init': ['ct.robot.MoveToQ({Q_INIT},dt=10.0,blocking=True)','fg'] if robot_mode=='ur'
                    else ['ct.robot.MoveToX({X_INIT},dt=10.0,blocking=True)','fg'] if robot_mode=='gen3'
                    else None,
    'move_pick_demo': ['fv.move_pick_demo','fg'],
    'repeat_p_p_demo_on': ['fv.repeat_p_p_demo "on"','fg'],
    'repeat_p_p_demo_off': ['fv.repeat_p_p_demo "off"','fg'],
    #'move_to_park': ['ct.robot.MoveToQ({Q_PARK},dt=5.0,blocking=True)','fg'],
    'grip_plus':  ['fv.open ct.robot.Arm, ct.robot.GripperPos()+0.015, True','fg'],
    }
  for key in scripts.iterkeys():
    if isinstance(scripts[key],list) and isinstance(scripts[key][0],str):
      scripts[key][0]= scripts[key][0].format(**config)

  run_script= lambda name: None if scripts[name] is None else\
                           pm.RunBGScript(scripts[name][0]) if scripts[name][1]=='bg' else\
                           pm.RunFGScript(scripts[name][0]) if scripts[name][1]=='fg' else\
                           None

  set_joy= lambda kind,value=None,is_active=0: pm.SetJoy(kind,value,is_active)
  #stop_joy= lambda: run_script('stop_joy')
  pm.joy_script_active= False
  start_joy= lambda: (setattr(pm,'joy_script_active',True), run_script('joy'))
  stop_joy= lambda: (setattr(pm,'joy_script_active',False), pm.SendString('q')) if pm.joy_script_active and pm.GetScriptNodeStatus()==ay_trick_msgs.msg.ROSNodeMode.PROGRAM_RUNNING else setattr(pm,'joy_script_active',False)
  def run_script_during_joy(name):
    if pm.joy_script_active:
      stop_joy()
      run_script(name)
      start_joy()
    else:
      run_script(name)

  #One-time commands (utility):
  #run_cmd('ur_calib')

  #Emergency commands:
  #pm.RunURDashboard('stop')
  #pm.RunURDashboard('unlock_protective_stop')

  def UpdateStatusTextBox(w,obj,status):
    #obj= w.widgets['status_textbox']
    obj.document().setPlainText(
'''Status: {status}
SafetyMode: {safety_mode}
RobotMode: {robot_mode}
URProgram: {program_running}
MainProgram: {script_status}'''.format(
      status=pm.status_names[pm.status],
      safety_mode=(pm.ur_safety_mode_names[pm.ur_safety_mode] if pm.ur_ros_running and pm.ur_safety_mode in pm.ur_safety_mode_names else 'UNRECOGNIZED') if robot_mode=='ur' else 'N/A',
      robot_mode=(pm.ur_robot_mode_names[pm.ur_robot_mode] if pm.ur_ros_running and pm.ur_robot_mode in pm.ur_robot_mode_names else 'UNRECOGNIZED') if robot_mode=='ur' else 'N/A',
      program_running=(pm.ur_ros_running and pm.ur_program_running) if robot_mode=='ur'
                      else (pm.gen3_ros_running)  if robot_mode=='gen3' else 'N/A',
      script_status=pm.script_node_status_names[pm.script_node_status] if pm.script_node_running and pm.script_node_status in pm.script_node_status_names else 'UNRECOGNIZED' ))


  #UI for configuring FV control parameters:
  ctrl_config= {
      #Common control parameters:
      'min_gstep': 0.0005,
      #Parameters used in fv.hold, fv.pickup2a, fv.pickup2b:
      'hold_sensitivity_slip': 0.08,  #Sensitivity of slip detection (smaller is more sensitive).
      'hold_sensitivity_oc': 0.2,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'hold_sensitivity_oo': 0.5,  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
      'hold_sensitivity_oa': 0.4,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      'pickup2a_area_drop_ratio': 0.3,  #If object area becomes smaller than this ratio, it's considered as dropped.
      'pickup2a_z_final': 0.05,  #Final height (offset from the beginning).
      'pickup2a_obj_area_filter_len': 5,  #Filter length for obj_area.
      #Parameters used in fv.openif:
      'openif_sensitivity_slip': 0.6,  #Sensitivity of slip detection (smaller is more sensitive).
      'openif_sensitivity_oc': 0.4,  #Sensitivity of object-center-movement detection (smaller is more sensitive).
      'openif_sensitivity_oo': 4.0,  #Sensitivity of object-orientation-movement detection (smaller is more sensitive).
      'openif_sensitivity_oa': 0.6,  #Sensitivity of object-area-change detection (smaller is more sensitive).
      'openif_sensitivity_force':0.9,  #Sensitivity of each force element; if the norm of force change is larger than this threshold, the point is counted as a force change point.
      'openif_nforce_threshold': 20,  #Threshold of number of force changing points to open the gripper.
      'openif_dw_grip': 0.02,  #Displacement of gripper movement.
    }
  if os.path.exists(config['FV_CTRL_CONFIG']):
    InsertDict(ctrl_config, LoadYAML(config['FV_CTRL_CONFIG']))
  def UpdateCtrlConfig(name, value):
    ctrl_config[name]= value
    SaveYAML(ctrl_config, config['FV_CTRL_CONFIG'], interactive=False)
  def AddCtrlConfigSliderWidget(widgets, name, prange):
    widgets['slider_ctrl_config_{}'.format(name)]= (
      'sliderh',{
        'range': prange,
        'value': ctrl_config[name],
        'n_labels': 3,
        'slider_style':1,
        'font_size_range': (10,12),
        #'size_policy': ('minimum', 'minimum'),
        'onvaluechange': lambda w,obj:UpdateCtrlConfig(name,obj.value())} )
    widgets['label_ctrl_config_{}'.format(name)]= (
      'label',{
        'text': name,
        'font_size_range': (12,14),
        'size_policy': ('minimum', 'minimum')} )
  def CtrlConfigSliderLayout(name):
    return ('label_ctrl_config_{}'.format(name),'slider_ctrl_config_{}'.format(name))


  widgets_common= {
    'status_signal_bar1': (
      'primitive_painer',{
        'color': (0,255,0),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setPaintColor({'red':(255,0,0),'green':(0,255,0),'yellow':(255,255,0),None:(128,128,128)}[
                        pm.status_color[0] if len(pm.status_color)>0 else None]), ),
        'margin': (0,0),
        'minimum_size': (None,20),
        'maximum_size': (None,20),
        'size_policy': ('expanding', 'fixed')}),
    'status_signal_bar2': (
      'primitive_painer',{
        'color': (0,255,0),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setPaintColor({'red':(255,0,0),'green':(0,255,0),'yellow':(255,255,0),None:(128,128,128)}[
                        pm.status_color[1] if len(pm.status_color)>1 else \
                          pm.status_color[0] if len(pm.status_color)>0 else None]), ),
        'margin': (0,0),
        'minimum_size': (None,20),
        'maximum_size': (None,20),
        'size_policy': ('expanding', 'fixed')}),
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
    'status_textbox': (
      'textedit',{
        'read_only': True,
        'font_size_range': (6,24),
        'text': 'Status Text Box',
        'onstatuschanged': UpdateStatusTextBox, }),
    'spacer_cmn1': ('spacer', {
        'w': 400,
        'h': 1,
        'size_policy': ('fixed', 'fixed')      }),
    }

  widgets_init= {
    'btn_init1': (
      'buttonchk',{
        'text':('(1)Robot core program','[3]Stop core program'),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled((status in (pm.NO_CORE_PROGRAM,pm.POWER_OFF)) if robot_mode=='ur'
                                     else (status in (pm.NO_CORE_PROGRAM,pm.ROBOT_READY)) ),
                      obj.setChecked(status not in (pm.NO_CORE_PROGRAM,) ),
                      ),
        'onclick':(lambda w,obj:(
                      #run_cmd('roscore'),
                      #rospy.wait_for_service('/rosout/get_loggers', timeout=5.0),
                      run_cmd('fix_usb'),
                      run_cmd('robot_ros'),
                      run_cmd('ur_pui_server') if robot_mode=='ur' else None,
                      #pm.InitNode(),//
                      #pm.StartUpdateStatusThread(),
                      pm.ConnectToURDashboard() if robot_mode=='ur' else None,
                      pm.WaitForURROSRunning(True) if robot_mode=='ur'  #Should be done after ConnectToURDashboard
                          else pm.WaitForGen3ROSRunning(True) if robot_mode=='gen3'
                          else None,
                      w.widgets['rviz'].setup(),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.POWER_OFF)  if robot_mode=='ur' else None,
                     ),
                   lambda w,obj:(
                      #pm.RunURDashboard('shutdown') if config['ShutdownRobotAfterUse'] else None,
                      pm.DisconnectUR() if robot_mode=='ur' else None,  #NOTE: This should be done after shutdown.
                      #pm.StopUpdateStatusThread(),
                      stop_cmd('ur_pui_server')  if robot_mode=='ur' else None,
                      stop_cmd('robot_ros'),
                      #stop_cmd('roscore'),
                     ) )}),
    'btn_init2': (
      'buttonchk',{
        'text':('(2)Power on robot','[2]Power off robot'),
        'enabled': not is_sim,
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled((status in (pm.POWER_OFF,pm.TORQUE_ENABLED,pm.ROBOT_READY)) if robot_mode=='ur'
                                     else False),
                      obj.setChecked((status in (pm.TORQUE_ENABLED,pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP)) if robot_mode=='ur'
                                     else False),
                      status==pm.ROBOT_EMERGENCY_STOP if robot_mode=='ur' else False and (
                        stop_cmd('robot_gripper'),
                        pm.RunURDashboard('stop')  if robot_mode=='ur' else None,
                        pm.WaitForProgramRunning(False)  if robot_mode=='ur' else None,
                        pm.RunURDashboard('power_off')  if robot_mode=='ur' else None,
                        ),
                      ),
        'onclick':(lambda w,obj:(
                      #run_cmd('robot_gripper'),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.POWER_OFF),
                      pm.RunURDashboard('power_on'),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.IDLE),
                      pm.RunURDashboard('brake_release'),
                      pm.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.RUNNING),
                      rospy.sleep(0.2),
                      pm.RunURDashboard('play'),
                      pm.WaitForProgramRunning(True),
                     ),
                   lambda w,obj:(
                      #stop_cmd('robot_gripper'),
                      pm.RunURDashboard('stop'),
                      pm.WaitForProgramRunning(False),
                      pm.RunURDashboard('power_off'),
                     ) )}),
    'btn_init3': (
      'buttonchk',{
        'text':('(3)Start program','[1]Stop program'),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled((status in (pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP)) if robot_mode=='ur'
                                     else (status in (pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING))),
                      obj.setChecked(status in (pm.WAIT_REQUEST,pm.PROGRAM_RUNNING) ),
                      status==pm.ROBOT_EMERGENCY_STOP if robot_mode=='ur' else False and (
                        #stop_cmd('rviz'),
                        stop_cmd('ay_trick_ros'),
                        stop_cmd('fvp'),
                        ),
                      ),
        'onclick':(lambda w,obj:(
                      run_cmd('robot_gripper'),
                      run_cmd('fvp'),
                      rospy.sleep(0.2),
                      run_cmd('config_fv_l'),
                      run_cmd('config_fv_r'),
                      run_cmd('ay_trick_ros'),
                      #run_cmd('rviz'),
                      pm.ConnectToScriptNode(),
                      pm.WaitForScriptNodeStatus(ay_trick_msgs.msg.ROSNodeMode.READY),
                      rospy.sleep(0.2),
                      run_script('setup'),
                     ),
                   lambda w,obj:(
                      #stop_cmd('rviz'),
                      stop_cmd('ay_trick_ros'),
                      stop_cmd('fvp'),
                      stop_cmd('robot_gripper'),
                     ) )}),
    'btn_reset_estop': (
      'button',{
        'text': 'Reset E-Stop',
        'enabled': not is_sim and robot_mode=='ur',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled((status in (pm.FAULT,)) if robot_mode=='ur' else False) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: (
                      pm.RunURDashboard('close_safety_popup'),
                      pm.RunURDashboard('restart_safety'),
                      ) if AskYesNoDialog(w,'1. Please make sure that everything is safe.\n2. Release the E-Stop.\n3. Press Yes.',title='Reset E-Stop') else None }),
    'btn_shutdown_ur': (
      'button',{
        'text': 'Shutdown robot',
        'enabled': not is_sim and robot_mode=='ur',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled((status in (pm.ROBOT_EMERGENCY_STOP,pm.POWER_OFF)) if robot_mode=='ur' else False) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: pm.RunURDashboard('shutdown'), }),
    'btn_exit': (
      'button',{
        'text': 'Exit',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled((status in (pm.NO_CORE_PROGRAM,pm.ROBOT_EMERGENCY_STOP)) if robot_mode=='ur'
                                     else (status in (pm.NO_CORE_PROGRAM,))) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    #'btn_shutdown_pc': (
      #'button',{
        #'text': 'Shutdown PC',
        #'onstatuschanged':lambda w,obj,status:(
                      #obj.setEnabled(status in (pm.NO_CORE_PROGRAM,pm.ROBOT_EMERGENCY_STOP)) ),
        #'size_policy': ('expanding', 'fixed'),
        #'onclick': lambda w,obj: run_cmd('shutdown_pc'), }),
    }
  layout_init= (
    'grid',None,(
      ('btn_init1',0,0), ('btn_init2',0,1),
      ('btn_init3',1,0), (('boxv',None,('btn_reset_estop','btn_shutdown_ur','btn_exit')),1,1),
      ))

  widgets_joy= {
    'btn_activate': (
      'buttonchk',{
        'text':('Activate','Deactivate'),
        'size_policy': ('expanding', 'fixed'),
        #'font_size_range': (8,24),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setChecked(False) if status not in (pm.WAIT_REQUEST,pm.PROGRAM_RUNNING) else None,
                      stop_joy() if status not in (pm.WAIT_REQUEST,pm.PROGRAM_RUNNING) else None ),
        'onclick':(lambda w,obj:(
                      start_joy(),
                     ),
                   lambda w,obj:(
                      stop_joy(),
                     ) )}),
    'btn_init_pose': (
      'button',{
        'text': 'Init Pose',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:run_script_during_joy('move_to_init'), }),
    'btn_push': (
      'buttonchk',{
        'text':('Push','Stop'),
        'enabled': True,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('trackf_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('trackf_off') )}),
    'btn_hold': (
      'buttonchk',{
        'text':('Hold','Stop'),
        'enabled': True,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('hold_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('hold_off') )}),
    'btn_grasp': (
      'buttonchk',{
        'text':('Grasp','Stop'),
        'enabled': True,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('grasp_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('grasp_off') )}),
    'btn_openif': (
      'buttonchk',{
        'text':('OpenIf','Stop'),
        'enabled': True,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('openif_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('openif_off') )}),
    'btn_pick': (
      'buttonchk',{
        'text':('Pick','Stop'),
        'enabled': True,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:set_joy('pickup_on',is_active=w.widgets['btn_activate'].isChecked()),
                   lambda w,obj:set_joy('pickup_off') )}),
    'btn_move_pick_demo': (
      'button',{
        'text': 'Move/Pick/Demo',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj:run_script_during_joy('move_pick_demo'), }),
    'btn_repeat_p_p_demo': (
      'buttonchk',{
        'text':('Repeat/PP/Demo','Stop'),
        'enabled': True,
        'size_policy': ('expanding', 'fixed'),
        'onclick':(lambda w,obj:((stop_joy(), w.widgets['btn_activate'].setChecked(False)) if pm.joy_script_active else None,
                                 run_script('repeat_p_p_demo_on')),
                   lambda w,obj:run_script('repeat_p_p_demo_off') )}),
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
      ('boxh',None, ('btn_init_pose',)),
      ('boxh',None, ('btn_push','btn_hold','btn_grasp','btn_openif')),
      ('boxh',None, ('btn_pick','btn_move_pick_demo','btn_repeat_p_p_demo')),
      ))

  widgets_ctrl_config= {
    }
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'min_gstep', (0.0,0.01,0.0001))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_slip', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oo', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'hold_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'pickup2a_area_drop_ratio', (0.0,1.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'pickup2a_z_final', (0.0,0.15,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'pickup2a_obj_area_filter_len', (1,20,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_slip', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oc', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oo', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_oa', (0.0,2.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_sensitivity_force', (0.0,4.0,0.01))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_nforce_threshold', (1,100,1))
  AddCtrlConfigSliderWidget(widgets_ctrl_config, 'openif_dw_grip', (0.0,0.1,0.001))

  layout_ctrl_config1= (
    'boxv',None, (
        ('boxh',None, CtrlConfigSliderLayout('min_gstep') ),
        #('boxh',None, CtrlConfigSliderLayout('grasp_th') ),
        #('boxh',None, CtrlConfigSliderLayout('grasp_filter_len') ),
        #('boxh',None, CtrlConfigSliderLayout('grasp_dstate_th') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oo') ),
        ('boxh',None, CtrlConfigSliderLayout('hold_sensitivity_oa') ),
        ('boxh',None, CtrlConfigSliderLayout('pickup2a_area_drop_ratio') ),
        ('boxh',None, CtrlConfigSliderLayout('pickup2a_z_final') ),
        ('boxh',None, CtrlConfigSliderLayout('pickup2a_obj_area_filter_len') ),
      ))
  layout_ctrl_config2= (
    'boxv',None, (
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_slip') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oc') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oo') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_oa') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_sensitivity_force') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_nforce_threshold') ),
        ('boxh',None, CtrlConfigSliderLayout('openif_dw_grip') ),
      ))

  widgets_recovery= {
    'chkbx_safety_to_recover': (
      'checkbox',{
        'text': 'Please check the safety of\nthe robot and environment',
        'onstatuschanged':lambda w,obj,status:(obj.setEnabled((status in (pm.PROTECTIVE_STOP,)) if robot_mode=='ur' else False) ),
        'onclick': lambda w,obj: w.widgets['btn_revoery'].setEnabled(obj.isChecked()) }),
    'btn_revoery': (
      'button',{
        'text':'Recovery motion',
        'enabled':False,
        'onclick':lambda w,obj:(
                      pm.RunURDashboard('stop'),
                      pm.WaitForProgramRunning(False),
                      pm.RunURDashboard('unlock_protective_stop'),
                      pm.WaitForSafetyMode(ur_dashboard_msgs.msg.SafetyMode.NORMAL),
                      rospy.sleep(0.2),
                      pm.RunURDashboard('play'),
                      pm.WaitForProgramRunning(True),
                      #run_script('moveto_wait_r'),
                      w.widgets['chkbx_safety_to_recover'].setChecked(False),
                      obj.setEnabled(False),
                     ) if w.widgets['chkbx_safety_to_recover'].isChecked() else None }),
    #'btn_start_r': ('duplicate', 'btn_start'),
    #'btn_stop_r': ('duplicate', 'btn_stop'),
    }
  layout_recovery= (
    'boxv',None,(
      'chkbx_safety_to_recover',
      'btn_revoery',
      ))

  widgets_debug= {
    'btn_rviz': (
      'buttonchk',{
        'text':('RViz','Stop RViz'),
        'font_size_range': (8,24),
        'onclick':(lambda w,obj:(
                      run_cmd('rviz'),
                     ),
                   lambda w,obj:(
                      stop_cmd('rviz'),
                     ) )}),
    'label_ur': (
      'label',{
        'text': 'UR: ',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'btn_ur_power_on': (
      'button',{
        'text': 'PowerOn',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('power_on'), }),
    'btn_ur_brake_release': (
      'button',{
        'text': 'BrakeRelease',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('brake_release'), }),
    'btn_ur_power_off': (
      'button',{
        'text': 'PowerOff',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('power_off'), }),
    'btn_ur_play': (
      'button',{
        'text': 'Play',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('play'), }),
    'btn_ur_stop': (
      'button',{
        'text': 'Stop',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('stop'), }),
    'btn_ur_shutdown': (
      'button',{
        'text': 'Shutdown',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('shutdown'), }),
    'btn_ur_unlock_protective_stop': (
      'button',{
        'text': 'UnlockProtectiveStop',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('unlock_protective_stop'), }),
    'btn_ur_restart_safety': (
      'button',{
        'text': 'RestartSafety',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('restart_safety'), }),
    'btn_ur_close_safety_popup': (
      'button',{
        'text': 'CloseSafetyPopup',
        'enabled': not is_sim and robot_mode=='ur',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('close_safety_popup'), }),
    'label_dxlg': (
      'label',{
        'text': 'DynamixelGripper: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'btn_dxlg_reboot': (
      'button',{
        'text': 'Reboot',
        'font_size_range': (8,24),
        #'size_policy': ('minimum', 'minimum'),
        'onclick': lambda w,obj: run_cmd('reboot_dxlg'), }),
    'btn_dxlg_factory_reset': (
      'button',{
        'text': 'FactoryReset',
        'font_size_range': (8,24),
        #'size_policy': ('minimum', 'minimum'),
        'onclick': lambda w,obj: run_cmd('factory_reset_dxlg'), }),
    'label_processes': (
      'label',{
        'text': 'Processes: ',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'minimum')}),
    'combobox_procs': (
      'combobox',{
        'options':('(Process_name/PID)',),
        'font_size_range': (8,24),
        'size_adjust_policy': 'all_contents',
        'onactivated': lambda w,obj:None}),
    'btn_update_proc_list': (
      'button',{
        'text': 'Update',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: (UpdateProcList(pm,w.widgets['combobox_procs']),
                                  #w.widgets['combobox_procs'].resize(w.widgets['combobox_procs'].sizeHint())
                                  ) }),
    'btn_terminate_proc': (
      'button',{
        'text': 'Terminate',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.TerminateBGProcess(str(w.widgets['combobox_procs'].currentText()).split('/')[0]), }),
    'btn_kill_proc': (
      'button',{
        'text': 'Kill',
        'font_size_range': (8,24),
        'size_policy': ('minimum', 'fixed'),
        'onclick': lambda w,obj: pm.KillBGProcess(str(w.widgets['combobox_procs'].currentText()).split('/')[0]), }),
    'btn_dbg_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    }
  layout_debug= (
    'boxv',None,(
      ('boxv',None, ('btn_rviz',)),
      ('boxh',None, ('label_ur', ('boxv',None, (
                        ('boxh',None, (
                            'btn_ur_power_on',
                            'btn_ur_brake_release',
                            'btn_ur_power_off',
                            'btn_ur_shutdown',)),
                        ('boxh',None, (
                            'btn_ur_play',
                            'btn_ur_stop',
                            'btn_ur_unlock_protective_stop',)),
                        ('boxh',None, (
                            'btn_ur_restart_safety',
                            'btn_ur_close_safety_popup')),
                        ))
                     )),
      ('boxh',None, ('label_dxlg', ('boxv',None, (
                        ('boxh',None, ('btn_dxlg_reboot','btn_dxlg_factory_reset')),
                        ))
                     )),
      ('boxh',None, ('label_processes', ('boxv',None, (
                        'combobox_procs',
                        ('boxh',None, ('btn_update_proc_list','btn_terminate_proc','btn_kill_proc')),
                        ('boxh',None, ('btn_dbg_exit',)),
                        ))
                     )),
      ))

  layout_main= (
    'boxv',None,(
      'status_signal_bar1',
      ('boxh',None, (
        ('boxv',None, ('rviz','status_textbox')),
        ('boxv',None, (
        ('tab','maintab',(
          ('Initialize',layout_init),
          ('Joy',layout_joy),
          ('Config/1',layout_ctrl_config1),
          ('Config/2',layout_ctrl_config2),
          ('Recovery',layout_recovery),
          ('Debug',layout_debug),
          )),'spacer_cmn1')),
        )),
      'status_signal_bar2',
      ))

  app= InitPanelApp()
  win_size= (1000,800)
  if fullscreen:  #NOTE: fullscreen mode will work only with Qt5.
    print 'Screen size:', app.screens()[0].size()
    screen_size= app.screens()[0].size()
    win_size= (screen_size.width(),screen_size.height())
  panel= TSimplePanel('Robot Operation Panel', size=win_size, font_height_scale=300.0)
  panel.AddWidgets(widgets_common)
  panel.AddWidgets(widgets_init)
  panel.AddWidgets(widgets_joy)
  panel.AddWidgets(widgets_ctrl_config)
  panel.AddWidgets(widgets_recovery)
  panel.AddWidgets(widgets_debug)
  panel.Construct(layout_main)
  #for tab in panel.layouts['maintab'].tab:
    #tab.setFont(QtGui.QFont('', 24))
  panel.layouts['maintab'].tabs.setFont(QtGui.QFont('', 18))
  #panel.showFullScreen()
  #panel.showMaximized()

  #Since the onstatuschanged signal is emitted from TProcessManager,
  #we connect the onstatuschanged slots of panel to it.
  for w_name, (w_type, w_param) in panel.widgets_in.iteritems():
    if 'onstatuschanged' in w_param and w_param['onstatuschanged'] is not None:
      pm.onstatuschanged.connect(lambda status,w_param=w_param,w_name=w_name: w_param['onstatuschanged'](panel,panel.widgets[w_name],status))

  #ROS system initialization.
  run_cmd('roscore')
  pm.InitNode()
  rospy.wait_for_service('/rosout/get_loggers', timeout=5.0)
  run_cmd('fix_usb')
  pm.StartVirtualJoyStick()
  if not is_sim:  pm.StartUpdateStatusThread()
  panel.close_callback= lambda event: (
      pm.TerminateAllBGProcesses(),  #including roscore
      pm.StopUpdateStatusThread(),
      #stop_cmd('roscore'),
      True)[-1]

  RunPanelApp()
