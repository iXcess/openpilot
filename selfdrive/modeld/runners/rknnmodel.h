#pragma once
#pragma clang diagnostic ignored "-Wdeprecated-declarations"

#include "rknn_api.h"
#include "selfdrive/modeld/runners/runmodel.h"


struct RKNNModelInput : public ModelInput {
  float *rknn_buffer;

  RKNNModelInput(const std::string _name, float *_buffer, int _size, float *_rknn_buffer) : ModelInput(_name, _buffer, _size), rknn_buffer(std::move(_rknn_buffer)) {}
  void setBuffer(float *_buffer, int _size) {
    ModelInput::setBuffer(_buffer, _size);
  }
};

class RKNNModel : public RunModel {
public:
  RKNNModel(const std::string path, float *_output, size_t _output_size, int runtime = 0, bool use_tf8 = false, cl_context context = NULL);
  void addInput(const std::string name, float *buffer, int size);
  void execute();

private:
  float *output;
  size_t output_size;
  rknn_context ctx;
  rknn_input_output_num io_num;
  rknn_input rknn_inputs[9]; // TODO: make it dynamically allocated
  rknn_output rknn_outputs[1]; // TODO: make it dynamically allocated
};
