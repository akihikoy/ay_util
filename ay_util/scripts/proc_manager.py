#!/usr/bin/python3
#\file    proc_manager.py
#\brief   Sub-process manager;
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.19, 2022
import subprocess
import re
import os
import signal
import time

class TSubProcManager(object):
  def __init__(self, debug=True):
    self.procs= {}
    self.re_split= re.compile(r'''((?:[^\ "']|"[^"]*"|'[^']*')+)''')
    self.file_ps_dump= '/tmp/proc_manager{}'.format(os.getpid()) if debug else None

  def DumpPS(self, file_name=None):
    if file_name is None:  file_name= self.file_ps_dump
    if file_name is None:  return
    with open(file_name,'w') as fp:
      for name,proc in self.procs.items():
        fp.write('{} {}\n'.format(proc.pid, name))

  #Split a given string command into a list according to the mode.
  #mode:
  #  'auto': Split if command is a string including spaces.
  #  True: Split.
  #  False: Not split.
  def SplitCommand(self, command, mode):
    if mode=='auto':
      mode= isinstance(command,str) and ' ' in command
    if mode and isinstance(command,str):
      command= self.re_split.split(command)[1::2]
      command= [c[1:-1] if len(c)>2 and (c[0]==c[-1]=='\'' or c[0]==c[-1]=='"') else c for c in command]
    return command

  #command: command string or list of command and arguments.
  #A string command is split into a list according to the split_cmd mode (cf. SplitCommand).
  #Return: exit_code (0: Normal exit, 1,2,..: Command error exit, -1: Python error exit).
  def RunFGProcess(self, command, shell=False, split_cmd='auto'):
    command= self.SplitCommand(command, split_cmd)
    print(f'''Run(FG): {' '.join(command)}''')
    try:
      kwargs=dict(
        shell=shell,
        #stdout=subprocess.PIPE,
        #stderr=subprocess.PIPE
        )
      p= subprocess.Popen(command, **kwargs)
      p.wait()
      exit_code= p.returncode
      return exit_code
    except OSError as e:
      print('RunFGProcess failed: {}'.format(e))
      return -1

  #command: command string or list of command and arguments.
  #A string command is split into a list according to the split_cmd mode (cf. SplitCommand).
  def RunBGProcess(self, name, command, shell=False, split_cmd='auto'):
    self.TerminateBGProcess(name)
    command= self.SplitCommand(command, split_cmd)
    print(f'''Run(BG:{name}): {' '.join(command)}''')
    try:
      kwargs=dict(
        shell=shell,
        preexec_fn=os.setsid,
        #stdout=subprocess.PIPE,
        #stderr=subprocess.PIPEn
        )
      p= subprocess.Popen(command, **kwargs)
      self.procs[name]= p
      self.DumpPS()
    except OSError as e:
      print('RunBGProcess failed: {}'.format(e))

  def TerminateBGProcess(self, name):
    if name not in self.procs:
      print('No process named',name)
      return
    print(f'''Terminate(BG:{name})''')
    self.TerminateProc(self.procs[name], name)
    del self.procs[name]
    self.DumpPS()

  def TerminateAllBGProcesses(self):
    for name,p in self.procs.items():
      print('Terminating',name)
      self.TerminateProc(p, name)
    self.procs= {}
    self.DumpPS()

  #WARNING: This is not safe.  When killing roscore, rosmaster is still alive.
  def KillBGProcess(self, name):
    if name not in self.procs:
      print('No process named',name)
      return
    self.procs[name].kill()
    self.procs[name].wait()
    #TODO: wait(): It is safer to have timeout.  For ver<3.3, implement like:
    #while p.poll() is None:
      #print 'Process still running...'
      #time.sleep(0.1)
    del self.procs[name]
    self.DumpPS()

  def IsBGProcessRunning(self, name):
    return self.procs[name].poll() is None

  def TerminateProc(self, p, name=''):
    #p.terminate()
    #p.wait()
    try:
      os.killpg(os.getpgid(p.pid), signal.SIGINT)

      timeout= 5.0
      start_time= time.time()
      while time.time() - start_time < timeout:
        if p.poll() is not None:
          print(f'Process[{name}]: Terminated gracefully.')
          return True
        time.sleep(0.1)

      print(f'Process[{name}]: SIGINT failed, sending SIGTERM...')
      os.killpg(os.getpgid(p.pid), signal.SIGTERM)

      start_time= time.time()
      while time.time() - start_time < timeout:
        if p.poll() is not None:
          print(f'Process[{name}]: Terminated with SIGTERM.')
          return True
        time.sleep(0.1)

      print(f'Process[{name}]: SIGTERM failed, sending SIGKILL...')
      os.killpg(os.getpgid(p.pid), signal.SIGKILL)
      p.wait()
      print(f'Process[{name}]: Forcefully killed.')
      return False

    except Exception as e:
      print(f'Process[{name}]: Exception during termination: {e}')
      return False
