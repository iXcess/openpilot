#!/usr/bin/env python3

import itertools
import os
import sys
import time
import numpy as np
from typing import Tuple, Dict, Union, Any

from openpilot.selfdrive.modeld.runners.runmodel_pyx import RunModel  #(TODO add 1)
from openpilot.selfdrive.modeld.constants import ModelConstants
from rknnlite.api import RKNNLite

ORT_TYPES_TO_NP_TYPES = {'tensor(float16)': np.float16, 'tensor(float)': np.float32, 'tensor(uint8)': np.uint8}

class RKNNModel(RunModel): #(TODO add 2)
# class RKNNModel():
  def __init__(self, path, output, runtime, use_tf8, cl_context): # runtime and cl_context are not used in this case
    self.inputs = {}
    self.output = output
    self.use_tf8 = use_tf8
    self.input_names = ['input_imgs','big_input_imgs','desire','traffic_convention','lateral_control_params','prev_desired_curv','nav_features','nav_instructions','features_buffer']  # noqa: E501

    self.input_shapes = {
    'input_imgs': [1, 12, 128, 256],
    'big_input_imgs': [1, 12, 128, 256],
    'desire': [1, 100, 8],
    'traffic_convention': [1, 2],
    'lateral_control_params': [1, 2],
    'prev_desired_curv': [1, 100, 1],
    'nav_features': [1, 256],
    'nav_instructions': [1, 150],
    'features_buffer': [1, 99, 512]
    }

    self.input_dtypes = {
    'input_imgs': np.float16,
    'big_input_imgs': np.float16,
    'desire': np.float16,
    'traffic_convention': np.float16,
    'lateral_control_params': np.float16,
    'prev_desired_curv': np.float16,
    'nav_features': np.float16,
    'nav_instructions': np.float16,
    'features_buffer': np.float16
    }

    ### TODO current problem with getting the input names, shape and dtypes needed by the rknn model (through python)

    # self.session = create_rknn_session(path, fp16_to_fp32=True)
    # self.input_names = [x.name for x in self.session.get_inputs()]
    # self.input_shapes = {x.name: [1, *x.shape[1:]] for x in self.session.get_inputs()}
    # self.input_dtypes = {x.name: ORT_TYPES_TO_NP_TYPES[x.type] for x in self.session.get_inputs()}

    ################################################################
    # initialise NPU on Rockchip
    self.rknn = RKNNLite(verbose=False)
    self.rknn.load_rknn(path)
    self.rknn.init_runtime()
    ################################################################


  def addInput(self, name, buffer):
    assert name in self.input_names
    self.inputs[name] = buffer

  def setInputBuffer(self, name, buffer):
    assert name in self.inputs
    self.inputs[name] = buffer

  def getCLBuffer(self, name):
    return None

  def execute(self):
    # input shaping and formatting
    inputs = {k: (v.view(np.uint8) / 255. if self.use_tf8 and k == 'input_img' else v) for k,v in self.inputs.items()}
    inputs = {k: v.reshape(self.input_shapes[k]).astype(self.input_dtypes[k]) for k,v in inputs.items()}
    # running inputs through model
    mt1 = time.perf_counter()
    outputs = self.rknn.inference(inputs=[inputs[input_name] for input_name in self.input_names], data_format=['nchw','nchw', 'nchw','nchw' , 'nchw', 'nchw', 'nchw', 'nchw', 'nchw'])
    mt2 = time.perf_counter()
    print("Raw execution time: " + str(mt2-mt1))
    # check that the output is valid
    assert len(outputs) == 1, "Only single model outputs are supported"
    self.output[:] = outputs[0]
    return self.output

if __name__ == '__main__':
  OUTPUT_SIZE = 6504

  # initialise input and output size, and start NPU chip
  output = np.zeros(OUTPUT_SIZE, dtype=np.float32)
  inputs = {
      'desire': np.zeros(ModelConstants.DESIRE_LEN * (ModelConstants.HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'traffic_convention': np.zeros(ModelConstants.TRAFFIC_CONVENTION_LEN, dtype=np.float32),
      'lateral_control_params': np.zeros(ModelConstants.LATERAL_CONTROL_PARAMS_LEN, dtype=np.float32),
      'prev_desired_curv': np.zeros(ModelConstants.PREV_DESIRED_CURV_LEN * (ModelConstants.HISTORY_BUFFER_LEN+1), dtype=np.float32),
      'nav_features': np.zeros(ModelConstants.NAV_FEATURE_LEN, dtype=np.float32),
      'nav_instructions': np.zeros(ModelConstants.NAV_INSTRUCTION_LEN, dtype=np.float32),
      'features_buffer': np.zeros(ModelConstants.HISTORY_BUFFER_LEN * ModelConstants.FEATURE_LEN, dtype=np.float32),
      }


  model = RKNNModel(path = '../models/supercombo.rknn', output = output, runtime = 0, use_tf8 = False, cl_context= None)

  # prefill all values with zeros
  model.addInput("input_imgs", np.zeros(393216, dtype=np.float32))
  model.addInput("big_input_imgs", np.zeros(393216, dtype=np.float32))

  for k,v in inputs.items():
      model.addInput(k, v)

  # run model with all zeros inputs

  for k,v in inputs.items():
      inputs[k][:] = v

  for i in range(1000):
      print(model.execute())

