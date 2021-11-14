#include <sys/resource.h>
#include <limits.h>

#include <cstdio>
#include <cstdlib>

#include "cereal/visionipc/visionipc_client.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"
#include "selfdrive/modeld/models/tank.h"

ExitHandler do_exit;

void run_model(TankModelState &model, VisionIpcClient &vipc_client) {
  PubMaster pm({"driverState"});

  while (!do_exit) {
    VisionIpcBufExtra extra = {};
    VisionBuf *buf = vipc_client.recv(&extra);
    if (buf == nullptr) continue;

    TankModelResult res = tankmodel_eval_frame(&model, buf->addr, buf->width, buf->height);
    printf("Bump prob: %f\n", res.bump_prob);

    // send dm packet
    MessageBuilder msg;
    auto framed = msg.initEvent().initDriverState();
    framed.setFrameId(extra.frame_id);
    framed.setOccludedProb(res.bump_prob);
    pm.send("driverState", msg);
  }
}

int main(int argc, char **argv) {
  setpriority(PRIO_PROCESS, 0, -15);

  // init the models
  TankModelState model;
  tankmodel_init(&model);

  VisionIpcClient vipc_client = VisionIpcClient("camerad", VISION_STREAM_RGB_WIDE, true);
  while (!do_exit && !vipc_client.connect(false)) {
    util::sleep_for(100);
  }

  // run the models
  if (vipc_client.connected) {
    LOGW("connected with buffer size: %d", vipc_client.buffers[0].len);
    run_model(model, vipc_client);
  }

  tankmodel_free(&model);
  return 0;
}
