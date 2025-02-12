#include <cassert>
#include <cstdio>
#include <cstdlib>

#define __STDC_CONSTANT_MACROS

#include "system/loggerd/encoder/mpp_encoder.h"
#include "third_party/libyuv/include/libyuv.h"

#include "common/swaglog.h"
#include "common/util.h"
#include "rga/rga.h"
#include "rga/im2d.h"
#define MPP_ALIGN(x, a) (((x) + ((a) - 1)) & ~((a) - 1))

MppEncoder::MppEncoder(const EncoderInfo &encoder_info, int in_width, int in_height)
    : VideoEncoder(encoder_info, in_width, in_height) {
}

MppEncoder::~MppEncoder() {
  encoder_close();
}

void MppEncoder::encoder_open(const char* path) {
    assert(mpp_create(&mpp_ctx, &mpp_mpi) == MPP_OK);
    // Todo, change to the right format h264 or h265
    assert(mpp_init(mpp_ctx, MPP_CTX_ENC, MPP_VIDEO_CodingAVC) == MPP_OK);

    mpp_enc_cfg_init(&cfg);
    mpp_mpi->control(mpp_ctx, MPP_ENC_GET_CFG, cfg);

    mpp_enc_cfg_set_u32(cfg, "codec:type", MPP_VIDEO_CodingAVC);
    mpp_enc_cfg_set_s32(cfg, "prep:width", 526);
    mpp_enc_cfg_set_s32(cfg, "prep:hor_stride", 528);
    mpp_enc_cfg_set_s32(cfg, "prep:height", 330);
    mpp_enc_cfg_set_s32(cfg, "prep:ver_stride", 336);
    LOGE("%d %d", out_width, out_height);
    mpp_enc_cfg_set_s32(cfg, "prep:format", MPP_FMT_YUV420SP);
    mpp_enc_cfg_set_s32(cfg, "prep:range", MPP_FRAME_RANGE_JPEG);

    // **Rate Control Settings (CBR for streaming, VBR for quality)**
    mpp_enc_cfg_set_u32(cfg, "rc:mode", MPP_ENC_RC_MODE_CBR); // CBR
    mpp_enc_cfg_set_s32(cfg, "rc:bps_target", 500000);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_max", 600000);
    mpp_enc_cfg_set_s32(cfg, "rc:bps_min", 400000);

    // **Frame Rate Settings**
    mpp_enc_cfg_set_u32(cfg, "rc:fps_in_flex", 0);
    mpp_enc_cfg_set_u32(cfg, "rc:fps_in_num", 30);  // Input FPS
    mpp_enc_cfg_set_u32(cfg, "rc:fps_in_denorm", 1);
    mpp_enc_cfg_set_u32(cfg, "rc:fps_out_flex", 0);
    mpp_enc_cfg_set_u32(cfg, "rc:fps_out_num", 20); // Output FPS
    mpp_enc_cfg_set_u32(cfg, "rc:fps_out_denorm", 1);

    // **Group of Pictures (GOP) - Keyframe Interval**
    mpp_enc_cfg_set_u32(cfg, "rc:gop", 90); // 2-second GOP for 30 FPS

    // **Slice settings
    mpp_enc_cfg_set_s32(cfg, "split:mode", MPP_ENC_SPLIT_NONE);

    //**Profile & Level Settings (Low Quality)**
    mpp_enc_cfg_set_u32(cfg, "h264:profile", 100);
    mpp_enc_cfg_set_u32(cfg, "h264:level", 31);

    // **Entropy Mode (CABAC for better compression)**
    mpp_enc_cfg_set_u32(cfg, "h264:cabac_en", 1);  // Enable CABAC
    mpp_enc_cfg_set_s32(cfg, "h264:cabac_idc", 0);
    mpp_enc_cfg_set_s32(cfg, "h264:trans8x8", 1);
    mpp_enc_cfg_set_s32(cfg, "h264:constraint_set", 0);

    // QP settings
    mpp_enc_cfg_set_s32(cfg, "rc:qp_init", 45);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max", 51);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min", 30);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_max_i", 51);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_min_i", 30);
    mpp_enc_cfg_set_s32(cfg, "rc:qp_ip", 6);

    mpp_mpi->control(mpp_ctx, MPP_ENC_SET_CFG, cfg);

    is_open = true;
    segment_num++;
    counter = 0;

    file = fopen("lala.h264", "wb");
}

void MppEncoder::encoder_close() {
    if (!is_open) return;

    mpp_destroy(mpp_ctx);
    is_open = false;

    fclose(file);
}

int MppEncoder::encode_frame(VisionBuf* buf, VisionIpcBufExtra *extra) {
    assert(buf->width == this->in_width);
    assert(buf->height == this->in_height);

    int alw = MPP_ALIGN(526, 16);
    int alh = MPP_ALIGN(330, 16);
    if (buf->type == 0 || buf->type == 1) {
      return 1;
    }

    // Rescaling
    void *dst_buf = malloc(alw * alh * 3 / 2);

    // Initialize RGA input & output structures
    rga_buffer_t src = wrapbuffer_virtualaddr(buf->addr, 1920, 1080, RK_FORMAT_YCbCr_420_SP);
    rga_buffer_t dst = wrapbuffer_virtualaddr(dst_buf, alw, alh, RK_FORMAT_YCbCr_420_SP);

    assert(imresize(src, dst, (double)526 / 1920, (double)330 / 1080, IM_SYNC) >= 0);

    // Allocate frame buffer
    assert(mpp_buffer_get(NULL, &mpp_buf, alw * alh *3/2) == MPP_OK);

    mpp_frame_init(&frame);
    mpp_frame_set_width(frame, 528);
    mpp_frame_set_height(frame, 336);
    mpp_frame_set_hor_stride(frame, MPP_ALIGN(526, 16));
    mpp_frame_set_ver_stride(frame, MPP_ALIGN(330, 16));
    mpp_frame_set_fmt(frame, MPP_FMT_YUV420SP);

    // Copy NV12 data into buffer
    //memcpy(mpp_buffer_get_ptr(mpp_buf), dst_buf, out_width * out_height *3 /2);
    memcpy(mpp_buffer_get_ptr(mpp_buf), dst_buf, alw * alh *3/2);
    //uint8_t *dst = (uint8_t *)mpp_buffer_get_ptr(mpp_buf);
    //uint8_t *src_y = buf->y;
    //uint8_t *src_uv = buf->uv;  // Assuming contiguous NV12 layout

    //memcpy(dst, src_y, in_width * in_height);               // Copy Y plane
    //memcpy(dst + in_width * in_height,
    //       src_uv, in_width * in_height / 2);               // Copy UV plane


    mpp_frame_set_buffer(frame, mpp_buf);
    LOGE("%d %d", alw, alh);
    //FILE *file1 = fopen("lala.yuv", "wb");
    //fwrite(dst_buf, 1, alw * alh * 3/2, file1);
    //fclose(file1);
    free(dst_buf);
    assert(mpp_mpi->encode_put_frame(mpp_ctx, frame) == MPP_OK);
    mpp_frame_deinit(&frame);

    assert(mpp_mpi->encode_get_packet(mpp_ctx, &packet) == MPP_OK);

    //uint8_t *bufz = (uint8_t*)pkt;
    size_t pkt_size = mpp_packet_get_length(packet);
    fwrite(mpp_packet_get_pos(packet), 1, pkt_size, file);

    //publisher_publish(this, segment_num, counter, *extra,
    //  0, // TODO
    //  kj::arrayPtr<capnp::byte>(bufz, (size_t)0), // TODO: get the header
    //  kj::arrayPtr<capnp::byte>(bufz, pkt_size));

    counter++;
    mpp_packet_deinit(&packet);
    mpp_buffer_put(mpp_buf);
    return 1;
}

