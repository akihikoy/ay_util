#!/usr/bin/python3
#\file    yaml_checker.py
#\brief   Check syntax and content of YAML files.
#\author  Akihiko Yamaguchi, info@akihikoy.net
#\version 0.1
#\date    Jun.07, 2025
from yamllint import linter
from yamllint.config import YamlLintConfig
import yaml
import re
import roslib
roslib.load_manifest('ay_py')
from ay_py.core.util import CPrint
import sys,os

# Speedup YAML using CLoader/CDumper
try:
  from yaml import CLoader as YLoader
except ImportError:
  from yaml import Loader as YLoader

#NOTE: Modify to ignore loading opencv-matrix data in YAML files.
import yaml
def opencv_matrix_ignore(loader, node):
  return None
yaml.add_constructor(u'tag:yaml.org,2002:opencv-matrix', opencv_matrix_ignore, Loader=YLoader)


yamllint_config='''
extends: default

rules:
  line-length: disable
  empty-lines: disable
  comments: disable
  comments-indentation: disable
  colons: disable
  commas: disable
  brackets: disable
  indentation: disable
  trailing-spaces: disable
  document-start: disable
'''


class TLineList(list):
  def __init__(self, *args, line_num=None, **kwargs):
    super().__init__(*args, **kwargs)
    self.__line__= line_num


def IsMixedTypesInvalid(types):
  numeric_types= {int, float, type(None)}
  str_types= {str, type(None)}

  if types.issubset(numeric_types) or types.issubset(str_types):
    return False

  if int in types and str in types:
    return True

  if float in types and str in types:
    return True

  return False


def CheckListUniformity(lst, path='root', line_num=None):
  types= {type(x) for x in lst}
  line_info= f": L.{line_num}" if line_num else ""

  if IsMixedTypesInvalid(types):
    CPrint(4,f"WARNING{line_info}: List '{path}' has irregular mixed types: {types}")

  for i, val in enumerate(lst):
    if isinstance(val, str) and re.match(r'^-?\d+(\.\d+)?\s+-?\d+(\.\d+)?$', val):
      CPrint(4,f"WARNING{line_info}: Possible missing comma in '{path}[{i}]': '{val}'")


def CustomYAMLCheck(yaml_content, yamllint_config):
  config= YamlLintConfig(yamllint_config)

  # Run yamllint
  problems= list(linter.run(yaml_content, config))
  has_syntax_error= False
  for problem in problems:
    col= 4 if problem.level=='error' else 3
    CPrint(col,f"{problem.level.upper()}: L.{problem}")
    if problem.rule=='syntax':
      has_syntax_error= True

  if has_syntax_error:
    CPrint(4,"ERROR: YAML syntax error detected. Stopping further checks.")
    return

  # Load YAML with PyYAML and track line numbers
  class LineLoader(YLoader):
    pass

  def construct_mapping(loader, node, deep=False):
    mapping= loader.construct_mapping(node, deep=deep)
    mapping['__line__']= node.start_mark.line + 1
    return mapping

  def construct_sequence(loader, node, deep=False):
    seq= loader.construct_sequence(node, deep=deep)
    return TLineList(seq, line_num=node.start_mark.line + 1)

  LineLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, construct_mapping)
  LineLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_SEQUENCE_TAG, construct_sequence)

  try:
    data= yaml.load(yaml_content, Loader=LineLoader)
  except yaml.YAMLError as exc:
    print(f"ERROR: YAML parsing error: {exc}")
    return

  # Recursive check for lists with line numbers
  def recursive_check(data, path='root', parent_line=None):
    line_num= getattr(data, '__line__', parent_line)
    if isinstance(data, list):
      CheckListUniformity(data, path, line_num)
    elif isinstance(data, dict):
      for key, value in data.items():
        if key != '__line__':
          recursive_check(value, f"{path}.{key}", line_num)

  recursive_check(data)


if __name__ == '__main__':
  input_files= sys.argv[1:]
  if len(input_files)==0:
    raise Exception('ERROR: yaml_checker: No input files.')

  for input_file in input_files:
    print('')
    CPrint(2,f'###Checking a YAML file: {input_file}...')
    if not os.path.exists(input_file):
      CPrint(4, f'ERROR: Input file does not exist: {input_file}')
      continue
    with open(input_file) as fp:
      text= fp.read()
      text= re.sub(r'^\s*%YAML:1\.0\s*', '', text)  #Check a wrong directive compatible for OpenCV.

      CustomYAMLCheck(text, yamllint_config)
