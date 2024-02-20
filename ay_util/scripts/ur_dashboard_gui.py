#!/usr/bin/python
#\file    ur_dashboard_gui.py
#\brief   Dashboard GUI for monitoring and controlling Universal Robots.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.16, 2021
import roslib
roslib.load_manifest('ay_py')
from ay_py.core import CPrint
from ay_py.tool.py_panel import TSimplePanel, InitPanelApp, RunPanelApp, AskYesNoDialog, QtCore, QtGui
import sys
import rospy
from proc_manager_ur import TProcessManagerUR


if __name__=='__main__':
  try:
    is_sim_default= rospy.get_param('robot_code').endswith('_SIM')
  except KeyError:
    is_sim_default= False
  is_sim= True if '-sim' in sys.argv or '--sim' in sys.argv else is_sim_default
  def get_arg(opt_name, default):
    exists= map(lambda a:a.startswith(opt_name),sys.argv)
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  topics_to_monitor= get_arg('-topics=',get_arg('--topics=',''))
  topics_to_monitor= None if topics_to_monitor=='' else {p.split(':')[0]:p.split(':')[-1] for p in topics_to_monitor.split(',')}

  #NOTE: Some constants are defined in ctrl_paramX.yaml
  #Parameters:
  config={
    'START_BTN': {
      'SIGNAL_IDX': 4,
      'SIGNAL_ON': True,
      },
    'STOP_BTN': {
      'SIGNAL_IDX': 5,
      'SIGNAL_ON': True,
      },
    }

  pm= TProcessManagerUR(is_sim=is_sim, topics_to_monitor=topics_to_monitor)

  def UpdateStatusTextBox(w,obj,status):
    #obj= w.widgets['status_textbox']
    obj.document().setPlainText(
'''Status: {status}
SafetyMode: {safety_mode}
RobotMode: {robot_mode}
URProgram: {program_running}
Gripper: {gripper_status}
MainProgram: {script_status}'''.format(
      status=pm.status_names[pm.status],
      safety_mode=pm.ur_safety_mode_names[pm.ur_safety_mode] if pm.robot_ros_running and pm.ur_safety_mode in pm.ur_safety_mode_names else 'UNRECOGNIZED',
      robot_mode=pm.ur_robot_mode_names[pm.ur_robot_mode] if pm.robot_ros_running and pm.ur_robot_mode in pm.ur_robot_mode_names else 'UNRECOGNIZED',
      program_running=pm.robot_ros_running and pm.ur_program_running,
      gripper_status='active' if pm.IsActive('Gripper') else 'disabled',
      script_status=pm.script_node_status_names[pm.script_node_status] if pm.script_node_running and pm.script_node_status in pm.script_node_status_names else 'UNRECOGNIZED' ))

  status_grid_list_text= [
      dict(label='Status', type='text', state='UNRECOGNIZED'),
      dict(label='MainProgram', type='text', state='UNRECOGNIZED'),
      dict(label='SafetyMode', type='text', state='UNRECOGNIZED'),
    ]
  status_grid_list_color= [
      dict(label='Safety', type='color', state='red'),
      dict(label='RobotMode', type='color', state='red'),
      dict(label='URProgram', type='color', state='red'),
      dict(label='ScriptServer', type='color', state='red'),
    ] + [dict(label=key, type='color', state='red') for key in sorted(pm.topics_to_monitor.iterkeys())]
  def UpdateStatusGridText(w,obj,status=None):
    obj.UpdateStatus('Status', pm.status_names[pm.status])
    if pm.robot_ros_running:
      obj.UpdateStatus('SafetyMode', pm.ur_safety_mode_names[pm.ur_safety_mode] if pm.ur_safety_mode in pm.ur_safety_mode_names else 'UNRECOGNIZED')
    obj.UpdateStatus('MainProgram', pm.script_node_status_names[pm.script_node_status] if pm.script_node_running and pm.script_node_status in pm.script_node_status_names else 'UNRECOGNIZED')
  def UpdateStatusGridColor(w,obj,status=None):
    if pm.robot_ros_running:
      obj.UpdateStatus('Safety', 'green' if pm.ur_safety_mode in pm.ur_safety_mode_names and pm.ur_safety_mode_names[pm.ur_safety_mode]=='NORMAL' else 'red')
      if pm.ur_robot_mode in pm.ur_robot_mode_names:
        obj.UpdateStatus('RobotMode', 'green' if pm.ur_robot_mode_names[pm.ur_robot_mode]=='RUNNING' else 'red' if pm.ur_robot_mode_names[pm.ur_robot_mode]=='POWER_OFF' else 'yellow')
      else:
        obj.UpdateStatus('RobotMode', 'red')
      obj.UpdateStatus('URProgram', 'green' if pm.ur_program_running else 'red')
    obj.UpdateStatus('ScriptServer', 'green' if pm.script_node_running else 'red')
    #obj= panel.widgets['status_grid1']
    for key,topic in pm.topics_to_monitor.iteritems():
      obj.UpdateStatus(key, 'green' if pm.IsActive(key) else 'red')

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
    #'status_textbox': (
      #'textedit',{
        #'read_only': True,
        #'font_size_range': (6,24),
        #'text': 'Status Text Box',
        #'onstatuschanged': UpdateStatusTextBox, }),
    'status_grid_text': (
      'status_grid',{
        'list_status': status_grid_list_text,
        'direction':'vertical',
        'shape':'square',
        'margin':(0.05,0.05),
        'rows':None,
        'columns':1,
        'font_size_range': (6,24),
        'onstatuschanged': UpdateStatusGridText,
        'ontopicshzupdated': UpdateStatusGridText, }),
    'status_grid_color': (
      'status_grid',{
        'list_status': status_grid_list_color,
        'direction':'vertical',
        'shape':'square',
        'margin':(0.05,0.05),
        'rows':None,
        'columns':2,
        'font_size_range': (6,24),
        'onstatuschanged': UpdateStatusGridColor,
        'ontopicshzupdated': UpdateStatusGridColor, }),
    'spacer_cmn1': ('spacer', {
        'w': 400,
        'h': 1,
        'size_policy': ('fixed', 'fixed')      }),
    }

  widgets_init= {
    'btn_ur_power': (
      'buttonchk',{
        'text':('Power on robot','Power off robot'),
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.POWER_OFF,pm.TORQUE_ENABLED,pm.ROBOT_READY)),
                      obj.setChecked(status in (pm.TORQUE_ENABLED,pm.ROBOT_READY,pm.WAIT_REQUEST,pm.PROGRAM_RUNNING,pm.PROTECTIVE_STOP) ),
                      status==pm.ROBOT_EMERGENCY_STOP and (
                        #stop_cmd('ur_gripper'),
                        pm.RunURDashboard('stop'),
                        pm.WaitForProgramRunning(False),
                        pm.RunURDashboard('power_off'),
                        ),
                      ),
        'onclick':(lambda w,obj:(
                      #run_cmd('ur_gripper'),
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
                      #stop_cmd('ur_gripper'),
                      pm.RunURDashboard('stop'),
                      pm.WaitForProgramRunning(False),
                      pm.RunURDashboard('power_off'),
                     ) )}),
    'btn_reset_estop': (
      'button',{
        'text': 'Reset E-Stop',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.FAULT,)) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: (
                      pm.RunURDashboard('close_safety_popup'),
                      pm.RunURDashboard('restart_safety'),
                      ) if AskYesNoDialog(w,'1. Please make sure that everything is safe.\n2. Release the E-Stop.\n3. Press Yes.',title='Reset E-Stop') else None }),
    'btn_shutdown_ur': (
      'button',{
        'text': 'Shutdown robot',
        'onstatuschanged':lambda w,obj,status:(
                      obj.setEnabled(status in (pm.ROBOT_EMERGENCY_STOP,pm.POWER_OFF)) ),
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: pm.RunURDashboard('shutdown'), }),
    }

  widgets_operation= {
    'btn_start': (
      'button',{
        'text': 'START',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROGRAM_RUNNING,)) ),
        'onclick': lambda w,obj: pm.SendFakeDigitalInSignal(config['START_BTN']['SIGNAL_IDX'], config['START_BTN']['SIGNAL_ON']), }),
    'btn_stop': (
      'button',{
        'text': 'STOP',
        #'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROGRAM_RUNNING,)) ),
        'onclick': lambda w,obj: pm.SendFakeDigitalInSignal(config['STOP_BTN']['SIGNAL_IDX'], config['STOP_BTN']['SIGNAL_ON']), }),
    }

  widgets_recovery= {
    'chkbx_safety_to_recover': (
      'checkbox',{
        'text': 'Please check the safety of\nthe robot and environment',
        'onstatuschanged':lambda w,obj,status:(obj.setEnabled(status in (pm.PROTECTIVE_STOP,)) ),
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
    }

  widgets_debug= {
    'btn_ur_power_on': (
      'button',{
        'text': 'PowerOn',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('power_on'), }),
    'btn_ur_brake_release': (
      'button',{
        'text': 'BrakeRelease',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('brake_release'), }),
    'btn_ur_power_off': (
      'button',{
        'text': 'PowerOff',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('power_off'), }),
    'btn_ur_play': (
      'button',{
        'text': 'Play',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('play'), }),
    'btn_ur_stop': (
      'button',{
        'text': 'Stop',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('stop'), }),
    'btn_ur_shutdown': (
      'button',{
        'text': 'Shutdown',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('shutdown'), }),
    'btn_ur_unlock_protective_stop': (
      'button',{
        'text': 'UnlockProtectiveStop',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('unlock_protective_stop'), }),
    'btn_ur_restart_safety': (
      'button',{
        'text': 'RestartSafety',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('restart_safety'), }),
    'btn_ur_close_safety_popup': (
      'button',{
        'text': 'CloseSafetyPopup',
        'font_size_range': (8,24),
        'onclick': lambda w,obj: pm.RunURDashboard('close_safety_popup'), }),
    'btn_dbg_exit': (
      'button',{
        'text': 'Exit',
        'size_policy': ('expanding', 'fixed'),
        'onclick': lambda w,obj: w.close(), }),
    }

  layout_main= (
    'boxv',None,(
      'status_signal_bar1',
      ('boxh',None, (
        ('boxv',None, ('status_grid_text','status_grid_color','btn_start','btn_stop')),  #'status_textbox',
        ('boxv',None, (
          ('boxh',None, ('btn_ur_power','btn_shutdown_ur')),
          ('boxv',None, (
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
                        )),
          ('boxh',None, ('chkbx_safety_to_recover','btn_revoery','btn_reset_estop')),
          'btn_dbg_exit',
          'spacer_cmn1'
          )),
        )),
      'status_signal_bar2',
      ))

  app= InitPanelApp()
  win_size= (800,400)
  panel= TSimplePanel('UR Dashboard', size=win_size, font_height_scale=300.0)
  panel.AddWidgets(widgets_common)
  panel.AddWidgets(widgets_init)
  panel.AddWidgets(widgets_operation)
  panel.AddWidgets(widgets_recovery)
  panel.AddWidgets(widgets_debug)
  panel.Construct(layout_main)

  #Since the onstatuschanged signal is emitted from TProcessManager,
  #we connect the onstatuschanged slots of panel to it.
  for w_name, (w_type, w_param) in panel.widgets_in.iteritems():
    if 'onstatuschanged' in w_param and w_param['onstatuschanged'] is not None:
      pm.onstatuschanged.connect(lambda status,w_param=w_param,w_name=w_name: w_param['onstatuschanged'](panel,panel.widgets[w_name],status))
  #Similarly, connect to ontopicshzupdated slots:
  for w_name, (w_type, w_param) in panel.widgets_in.iteritems():
    if 'ontopicshzupdated' in w_param and w_param['ontopicshzupdated'] is not None:
      pm.ontopicshzupdated.connect(lambda w_param=w_param,w_name=w_name: w_param['ontopicshzupdated'](panel,panel.widgets[w_name]))

  pm.InitNode()
  pm.StartUpdateStatusThread()
  pm.ConnectToURDashboard()
  if not pm.WaitForRobotROSRunning(True):  #Should be done after ConnectToURDashboard
    CPrint(4, 'UR ROS driver is not running!')
  else:
    CPrint(2, 'The process manager is ready.')
  panel.close_callback= lambda event: (
      pm.TerminateAllBGProcesses(),
      pm.StopUpdateStatusThread(),
      pm.DisconnectUR(),
      True)[-1]

  RunPanelApp()
