#pragma once

#include <rk_mpi.h>
#include <rk_venc_cfg.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "system/loggerd/encoder/encoder.h"
#include "system/loggerd/loggerd.h"

class MppEncoder : public VideoEncoder {
public:
  MppEncoder(const EncoderInfo &encoder_info, int in_width, int in_height);
  ~MppEncoder();
  int encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra);
  void encoder_open(const char* path);
  void encoder_close();

private:
  int segment_num = -1;
  int counter = 0;
  bool is_open = false;
  FILE *file;

  MppCtx mpp_ctx;
  MppApi *mpp_mpi;
  MppFrame frame;
  MppPacket packet;
  MppEncCfg cfg;
  MppBuffer mpp_buf;
  //MppBufferGroup buf_grp;

  std::vector<uint8_t> convert_buf;
};
