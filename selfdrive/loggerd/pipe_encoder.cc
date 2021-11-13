#include <thread>

#include "cereal/messaging/messaging.h"
#include "cereal/services.h"
#include "cereal/visionipc/visionipc.h"
#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/camerad/cameras/camera_common.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"

#include "selfdrive/loggerd/encoder.h"
#include "selfdrive/loggerd/omx_encoder.h"

constexpr int FPS = 20;
const int BITRATE = 512000;

namespace {
ExitHandler do_exit;

void encoder_thread() {
  VisionIpcClient vipc_client = VisionIpcClient("camerad", VISION_STREAM_YUV_FRONT, false);
  OmxEncoder *encoder = NULL;

  while (!do_exit) {
    if (!vipc_client.connect(false)) {
      util::sleep_for(1);
      continue;
    }
    LOGD("connected");
    VisionBuf buf_info = vipc_client.buffers[0];

    if (encoder == NULL) {
      LOGD("encoder init %dx%d", buf_info.width, buf_info.height);
      encoder = new OmxEncoder(NULL, buf_info.width, buf_info.height, FPS, BITRATE, false, false, false, true);
      encoder->encoder_open(NULL);
      LOGD("created encoder");
    }

    while (!do_exit) {
      // recv frame
      VisionIpcBufExtra extra;
      VisionBuf* buf = vipc_client.recv(&extra);
      if (buf == nullptr) continue;
      LOGD("frame");

      // encode and pipe to stderr
      int out_id = encoder->encode_frame(buf->y, buf->u, buf->v, buf->width, buf->height, extra.timestamp_eof);
      if (out_id == -1) {
        LOGE("out_id not valid");
      }

      LOGD("encoded");
    }

    if(encoder != NULL) {
      encoder->encoder_close();
    }
  }
}

} // namespace


int main(int argc, char** argv) {
  LOGD("starting pipe encoder");

  Context::create();

  std::thread encoding_thread = std::thread(encoder_thread);

  while (!do_exit) {
    // do main loop stuff`2
  }

  encoding_thread.join();
  
  LOGD("stopped pipe encoder");
}