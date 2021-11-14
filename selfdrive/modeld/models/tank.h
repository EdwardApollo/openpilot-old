#pragma once

#include <vector>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/util.h"
#include "selfdrive/modeld/models/commonmodel.h"
#include "selfdrive/modeld/runners/run.h"

#define OUTPUT_SIZE 1
#define WIDTH 1928
#define HEIGHT 1208
#define CHANNELS 3
#define INPUT_SIZE (WIDTH / 2 * HEIGHT / 2 * CHANNELS)

typedef struct TankModelResult {
  float bump_prob;
  float execution_time;
} TankModelResult;

typedef struct TankModelState {
  RunModel *m;
  float net_input_buf[INPUT_SIZE];
  float output[OUTPUT_SIZE];
  std::vector<float> net_input_buf;
} TankModelState;

void tankmodel_init(TankModelState* s);
TankModelResult tankmodel_eval_frame(TankModelState* s, void* stream_buf, int width, int height);
void tankmodel_free(TankModelState* s);
