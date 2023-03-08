#!/usr/bin/python
#\file    merge_yaml.py
#\brief   Merge YAML files and save as a YAML file.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Mar.08, 2023
import roslib
roslib.load_manifest('ay_py')
from ay_py.core.util import InsertDict, LoadYAML, SaveYAML
import sys

#Merge dictionaries in_dict_base, loaded from in_file_names YAML files, and in_dict_priority in this order.
#NOTE: in_dict_base is overwritten by the merged dictionary.
#The merged dictionary is saved into out_file_name if it is not None.
#The merged dictionary is also returned.
def MergeYAML(in_dict_base, in_file_names, in_dict_priority, out_file_name=None):
  for in_f in in_file_names:
    in_d= LoadYAML(in_f)
    if in_d is not None:
      InsertDict(in_dict_base, in_d)
  InsertDict(in_dict_base, in_dict_priority)
  if out_file_name is not None:
    SaveYAML(in_dict_base, out_file_name, interactive=False, directive='%YAML:1.0')
  return in_dict_base

if __name__=='__main__':
  in_file_names= sys.argv[1:]
  if len(in_file_names)<2:
    raise Exception('Error: merge_yaml requires more than two arguments.')

  out_file_name= in_file_names[-1]
  in_file_names= in_file_names[:-1]
  print 'Input files: {}\nOutput file: {}'.format(in_file_names, out_file_name)

  MergeYAML({}, in_file_names, {}, out_file_name)
