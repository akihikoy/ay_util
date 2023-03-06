#!/usr/bin/python
#\file    proc_manager.py
#\brief   Sub-process manager;
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.19, 2022
import subprocess
import re

class TSubProcManager(object):
  def __init__(self):
    self.procs= {}
    self.re_split= re.compile(r'''((?:[^\ "']|"[^"]*"|'[^']*')+)''')

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
    return command

  #command: command string or list of command and arguments.
  #A string command is split into a list according to the split_cmd mode (cf. SplitCommand).
  def RunFGProcess(self, command, shell=False, split_cmd='auto'):
    command= self.SplitCommand(command, split_cmd)
    p= subprocess.Popen(command, shell=shell)
    p.wait()

  #command: command string or list of command and arguments.
  #A string command is split into a list according to the split_cmd mode (cf. SplitCommand).
  def RunBGProcess(self, name, command, shell=False, split_cmd='auto'):
    self.TerminateBGProcess(name)
    command= self.SplitCommand(command, split_cmd)
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

