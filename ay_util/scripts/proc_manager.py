#!/usr/bin/python
#\file    proc_manager.py
#\brief   Sub-process manager;
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Sep.19, 2022
import subprocess

class TSubProcManager(object):
  def __init__(self):
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

