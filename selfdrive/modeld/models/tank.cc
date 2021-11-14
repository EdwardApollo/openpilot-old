#include <cstring>

#include "selfdrive/common/mat.h"
#include "selfdrive/common/params.h"
#include "selfdrive/common/timing.h"
#include "selfdrive/hardware/hw.h"
#include "selfdrive/modeld/models/tank.h"

void tankmodel_init(TankModelState* s) {
  s->m = new SNPEModel("../../models/tankmodel.dlc", &s->output[0], OUTPUT_SIZE, USE_DSP_RUNTIME);
}

TankModelResult tankmodel_eval_frame(TankModelState* s, void* stream_buf, int width, int height) {
  for (int w=0; w<WIDTH; w++) {
    for (int h=0; h<HEIGHT; h++) {
      for (int c=0; c<CHANNELS; c++) {
        if (w%2 == 0 && h%2 == 0)
        s->net_input_buf[c*WIDTH/2*HEIGHT/2*sizeof(float) + w*HEIGHT/2*sizeof(float) + h*sizeof(float)] =
          (float)(((uint8_t*)stream_buf)[w*HEIGHT*CHANNELS*sizeof(float) + h*CHANNELS*sizeof(float) + c*sizeof(float)]);
      }
    }
  }

  double t1 = millis_since_boot();
  s->m->execute(s->net_input_buf, INPUT_SIZE);
  double t2 = millis_since_boot();

  TankModelResult ret = {0};
  ret.bump_prob = s->output[0];
  ret.execution_time = (t2 - t1) / 1000.;
  return ret;
}

void tankmodel_free(TankModelState* s) {
  delete s->m;
}
