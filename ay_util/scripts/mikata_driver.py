#!/usr/bin/python3
#\file    mikata_driver.py
#\brief   Mikata ROS node.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Aug.29, 2018
#\version 0.2
#\date    Nov.01, 2018
#         Added support of Crane-X7
import roslib;
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('ay_py')
roslib.load_manifest('ay_util_msgs')
import rospy
import sensor_msgs.msg
import trajectory_msgs.msg
import actionlib
import control_msgs.msg
import sys
import importlib
import ay_util_msgs.msg
import ay_util_msgs.srv
from ay_py.misc.dxl_util import DxlPortHandler

class TMikataDriver(object):
  def __init__(self, node_name='mikata_driver',
               dev='/dev/ttyUSB0', robot_type='Mikata',
               robot_module=None, class_name=None, interpolation='spline',
               disable_at_exit=False):
    self.node_name= node_name
    self.dev= dev
    self.robot_type= robot_type
    self.disable_at_exit= disable_at_exit

    rospy.init_node(self.node_name)

    # If module/class are not specified, fall back to robot_type-based mapping:
    if robot_module is None or class_name is None:
      if self.robot_type == 'Mikata':
        robot_module = 'ay_py.misc.dxl_mikata'
        class_name   = 'TMikata'
      elif self.robot_type == 'CraneX7':
        robot_module = 'ay_py.misc.dxl_cranex7'
        class_name   = 'TCraneX7'
      elif self.robot_type == 'Mikata6':
        robot_module = 'ay_py.misc.dxl_mikata6'
        class_name   = 'TMikata6'
      else:
        raise Exception(f'Invalid robot type: {self.robot_type}')

    # Dynamically import the module and class
    self.robot_module= robot_module
    self.class_name= class_name
    mod = importlib.import_module(self.robot_module)
    cls = getattr(mod, self.class_name)

    # Instantiate the robot object
    self.mikata = cls(dev=self.dev, interpolation=interpolation)

    #Set callback to exit when Ctrl+C is pressed.
    DxlPortHandler.ReopenCallback= lambda: not rospy.is_shutdown()

    self.pub_js= rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=1)
    self.pub_state= rospy.Publisher('~state', ay_util_msgs.msg.SimpleRobotState, queue_size=1)

    self.js= None
    self.state_msg= None
    self.joint_names= self.mikata.JointNames()

    print('Initializing and activating {robot_type} arm...'.format(robot_type=self.robot_type))
    if not self.mikata.Setup():
      raise Exception('Failed to setup {robot_type} arm.'.format(robot_type=self.robot_type))
    #self.mikata.EnableTorque()
    self.mikata.StartStateObs(self.JointStatesCallback)

    self.sub_jpc= rospy.Subscriber('/joint_path_command', trajectory_msgs.msg.JointTrajectory, self.PathCmdCallback)
    #self.sub_jsc= rospy.Subscriber('/joint_speed_command', trajectory_msgs.msg.JointTrajectory, self.SpeedCmdCallback, queue_size=1)

    self.ftaction_feedback= control_msgs.msg.FollowJointTrajectoryFeedback()
    self.ftaction_result= control_msgs.msg.FollowJointTrajectoryResult()
    self.ftaction_name= '/follow_joint_trajectory'
    self.ftaction_actsrv= actionlib.SimpleActionServer(self.ftaction_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.FollowTrajActionCallback, auto_start=False)
    self.ftaction_actsrv.start()

    self.srv_io= rospy.Service('~robot_io', ay_util_msgs.srv.DxlIO, self.DxlIOHandler)

  def __del__(self):
    self.Cleanup()

  def Cleanup(self):
    print(f'{self.robot_type}: Cleanup')
    self.mikata.StopStateObs()
    if self.disable_at_exit:  self.mikata.DisableTorque()
    self.mikata.Quit()

  def JointStatesCallback(self, state):
    if rospy.is_shutdown():
      return False  #Stops the state observer loop.

    if self.state_msg is None:
      self.state_msg= ay_util_msgs.msg.SimpleRobotState()
    self.state_msg.header.stamp= rospy.Time(state['stamp'])
    self.state_msg.is_normal= self.mikata.IsNormal()
    self.state_msg.is_error= self.mikata.IsError()
    self.state_msg.torque_enabled= self.mikata.TorqueEnabled()
    self.pub_state.publish(self.state_msg)

    if None in state['position'] or None in state['velocity'] or None in state['effort']:
      return True  #Do not publish the state, but keep the state observer running.
    if self.js is None:
      self.js= sensor_msgs.msg.JointState()
      self.js.name= state['name']
      self.js.header.seq= 0
    self.js.header.seq= self.js.header.seq+1
    self.js.header.stamp= rospy.Time.now()
    self.js.position= state['position']
    self.js.velocity= state['velocity']
    self.js.effort= state['effort']
    self.pub_js.publish(self.js)
    return True

  def PathCmdCallback(self, msg):
    def callback(context,t,q,dq):
      if context=='loop_begin':
        if rospy.is_shutdown():  return False
        return True
    q_traj= [p.positions for p in msg.points]
    t_traj= [p.time_from_start.to_sec() for p in msg.points]
    self.mikata.FollowTrajectory(msg.joint_names, q_traj, t_traj, blocking=False, callback=callback)

  # Callback for actionlib.SimpleActionServer.
  def FollowTrajActionCallback(self, goal):
    def callback(context,t,q,dq):
      #print context,t,q,dq,'---',self.ftaction_result.error_code
      if context=='loop_begin':
        if rospy.is_shutdown():  return False
        if self.ftaction_actsrv.is_preempt_requested():
          print('%s: Preempted' % self.ftaction_name)
          self.ftaction_actsrv.set_preempted()
          self.ftaction_result.error_code= None
          return False
        self.ftaction_actsrv.publish_feedback(self.ftaction_feedback)
        return True
      elif context=='loop_end':
        pass
      elif context=='final':
        pass
    self.ftaction_result.error_code= self.ftaction_result.SUCCESSFUL
    q_traj= [p.positions for p in goal.trajectory.points]
    t_traj= [p.time_from_start.to_sec() for p in goal.trajectory.points]
    self.mikata.FollowTrajectory(goal.trajectory.joint_names, q_traj, t_traj, blocking=True, callback=callback)
    if self.ftaction_result.error_code==self.ftaction_result.SUCCESSFUL:
      self.ftaction_result.error_code= self.ftaction_result.SUCCESSFUL
      print('%s: Succeeded' % self.ftaction_name)
      self.ftaction_actsrv.set_succeeded(self.ftaction_result)

  # Handler of robot_io service (ay_util_msgs/DxlIO).
  def DxlIOHandler(self, req):
    res= ay_util_msgs.srv.DxlIOResponse()
    if req.command=='Read':  #Read from Dynamixel. input: joint_names, data_s (address name).  return: res_ia.
      with self.mikata.port_locker:
        res.res_ia= [self.mikata.dxl[j].Read(req.data_s) for j in req.joint_names]
    elif req.command=='Write':  #Write to Dynamixel. input: joint_names, data_s (address name), data_ia (values).
      with self.mikata.port_locker:
        for j,value in zip(req.joint_names,req.data_ia):
          self.mikata.dxl[j].Write(req.data_s, value)
    elif req.command=='EnableTorque':  #Enable joint_names (joint_names is [], all joints are enabled).
      if len(req.joint_names)==0:  self.mikata.EnableTorque()
      else:  self.mikata.EnableTorque(req.joint_names)
    elif req.command=='DisableTorque':  #Disable joint_names (joint_names is [], all joints are disabled).
      if len(req.joint_names)==0:  self.mikata.DisableTorque()
      else:  self.mikata.DisableTorque(req.joint_names)
    elif req.command=='Reboot':  #Reboot joint_names (joint_names is [], all joints are rebooted).
      if len(req.joint_names)==0:  self.mikata.Reboot()
      else:  self.mikata.Reboot(req.joint_names)
    elif req.command=='MoveTo':  #Move to target position.  input: joint_names, data_fa (joint positions in radian), data_b (blocking).
      #print 'MoveTo',dict(zip(req.joint_names,req.data_fa))
      self.mikata.MoveTo(dict(list(zip(req.joint_names,req.data_fa))), blocking=req.data_b)
    elif req.command=='SetCurrent':  #Set current.  input: joint_names, data_fa (currents in mA).
      self.mikata.SetCurrent(dict(list(zip(req.joint_names,req.data_fa))))
    elif req.command=='SetVelocity':  #Set velocity.  input: joint_names, data_fa (velocities in rad/s).
      self.mikata.SetVelocity(dict(list(zip(req.joint_names,req.data_fa))))
    elif req.command=='SetPWM':  #Set PWM.  input: joint_names, data_fa (PWM values in percentage).
      #print 'SetPWM',dict(zip(req.joint_names,req.data_fa))
      self.mikata.SetPWM(dict(list(zip(req.joint_names,req.data_fa))))

    j= req.joint_names[-1] if len(req.joint_names)>0 else self.joint_names[-1]
    res.result= self.mikata.dxl[j].dxl_result  #dynamixel.getLastTxRxResult
    res.error= self.mikata.dxl[j].dxl_err  #dynamixel.getLastRxPacketError
    return res

if __name__=='__main__':
  def get_arg(opt_name, default):
    exists= [a.startswith(opt_name) for a in sys.argv]
    if any(exists):  return sys.argv[exists.index(True)].replace(opt_name,'')
    else:  return default
  kwargs= dict(
    node_name= get_arg('-node_name=',get_arg('--node_name=','mikata_driver')),
    dev= get_arg('-dev=',get_arg('--dev=','/dev/ttyUSB0')),
    robot_type= get_arg('-robot_type=',get_arg('--robot_type=','Mikata')),
    robot_module= get_arg('-robot_module=',get_arg('--robot_module=',None)),
    class_name= get_arg('-class_name=',get_arg('--class_name=',None)),
    interpolation= get_arg('-interpolation=',get_arg('--interpolation=','spline')),
    disable_at_exit= (True if '-disable_at_exit' in sys.argv or '--disable_at_exit' in sys.argv else
                      False if '-no_disable_at_exit' in sys.argv or '--no_disable_at_exit' in sys.argv else False),
    )

  for k, v in kwargs.items():
    print(f'{k} = {v}')
  try:
    robot= TMikataDriver(**kwargs)
    #rospy.on_shutdown(lambda rt=kwargs['robot_type']: (print(f'{rt}: Shutdown sleep...'), rospy.sleep(2.0)))
    rospy.spin()
  finally:
    robot.Cleanup()
