#pragma clang diagnostic ignored "-Wexceptions"

#include "selfdrive/modeld/runners/rknnmodel.h"

#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "common/util.h"
#include "common/timing.h"

static void dump_tensor_attr(rknn_tensor_attr* attr)
{
  LOGD("index=%d, name=%s, n_dims=%d, dims=[%d, %d, %d, %d], n_elems=%d, size=%d, fmt=%s, type=%s, qnt_type=%s, "
         "zp=%d, scale=%f\n",
         attr->index, attr->name, attr->n_dims, attr->dims[0], attr->dims[1], attr->dims[2], attr->dims[3],
         attr->n_elems, attr->size, get_format_string(attr->fmt), get_type_string(attr->type),
         get_qnt_type_string(attr->qnt_type), attr->zp, attr->scale);
}

static unsigned char* load_model(const char* filename, int* model_size)
{
  FILE* fp = fopen(filename, "rb");
  if (fp == nullptr) {
    LOGD("fopen %s fail!\n", filename);
    return NULL;
  }
  fseek(fp, 0, SEEK_END);
  int            model_len = ftell(fp);
  unsigned char* model     = (unsigned char*)malloc(model_len);
  fseek(fp, 0, SEEK_SET);
  if (model_len != fread(model, 1, model_len, fp)) {
    LOGD("fread %s fail!\n", filename);
    free(model);
    return NULL;
  }
  *model_size = model_len;
  if (fp) {
    fclose(fp);
  }
  return model;
}

RKNNModel::RKNNModel(const std::string path, float *_output, size_t _output_size, int runtime, bool _use_tf8, cl_context context) {
  output = _output;
  output_size = _output_size;
  ctx = 0;

  int model_len = 0;
  unsigned char* modelptr = load_model(path.c_str(), &model_len);

  //std::string model_data;
  //model_data = util::read_file(path);
  //unsigned char lala[model_data.length() + 1];
  //std::copy(model_data.data(), model_data.data() + model_data.length() + 1, lala);

  // load model
  int ret = rknn_init(&ctx, modelptr, model_len, 0, NULL);
  assert(model_len > 0);
  LOGD("loaded model with size: %d", model_len);
  free(modelptr);

  // NPU core setting to use all of them
  rknn_set_core_mask(ctx, RKNN_NPU_CORE_0_1_2);
  // TODO: What is this for? rknn_set_batch_core_num(ctx, 2);

  // get sdk and driver version
  rknn_sdk_version sdk_ver;
  ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &sdk_ver, sizeof(sdk_ver));
  LOGD("rknnrt version: %s, driver version: %s\n", sdk_ver.api_version, sdk_ver.drv_version);

  // get model input output info
  ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
  LOGD("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

  LOGD("input tensors: \n");
  rknn_tensor_attr input_attrs[io_num.n_input];
  memset(input_attrs, 0, io_num.n_input * sizeof(rknn_tensor_attr));
  for (uint32_t i = 0; i < io_num.n_input; i++) {
    input_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]), sizeof(rknn_tensor_attr));
    assert(ret == RKNN_SUCC);
    dump_tensor_attr(&input_attrs[i]);
  }

  LOGD("output tensors:\n");
  rknn_tensor_attr output_attrs[io_num.n_output];
  memset(output_attrs, 0, io_num.n_output * sizeof(rknn_tensor_attr));
  for (uint32_t i = 0; i < io_num.n_output; i++) {
    output_attrs[i].index = i;
    ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]), sizeof(rknn_tensor_attr));
    assert(ret == RKNN_SUCC);
    dump_tensor_attr(&output_attrs[i]);
  }

  // initialise inputs
  memset(rknn_inputs, 0, io_num.n_input * sizeof(rknn_input));
  for (uint32_t i = 0; i < io_num.n_input; i++) {
    rknn_inputs[i].index = i;
    rknn_inputs[i].pass_through = 0;
    rknn_inputs[i].type = input_attrs[i].type;
    rknn_inputs[i].fmt = input_attrs[i].fmt;
    rknn_inputs[i].size = input_attrs[i].size;
    rknn_inputs[i].buf = NULL;
  }

  // initialises output
  memset(rknn_outputs, 0, io_num.n_output * sizeof(rknn_output));
  for (uint32_t i = 0; i < io_num.n_output; ++i) {
    rknn_outputs[i].want_float  = 1;
    rknn_outputs[i].index       = i;
    rknn_outputs[i].is_prealloc = 0;
  }
}

void RKNNModel::addInput(const std::string name, float *buffer, int size) {
  inputs.push_back(std::unique_ptr<RKNNModelInput>(new RKNNModelInput(name, buffer, size, nullptr)));
}

void RKNNModel::execute() {
  for (uint32_t i = 0; i < io_num.n_input; i++) {
    rknn_inputs[i].buf = inputs[i]->buffer;
  }

  rknn_inputs_set(ctx, io_num.n_input, rknn_inputs);
  int ret = rknn_run(ctx, NULL);
  assert(ret != -1);
  rknn_outputs_get(ctx, io_num.n_output, rknn_outputs, NULL);

  // Model only has one output
  assert(io_num.n_output == 1);
  memcpy(output, (float *) rknn_outputs[0].buf, rknn_outputs[0].size);
  output_size = rknn_outputs[0].size;
}
