#!/usr/bin/env python

import time
import subprocess

PIPE_ENCODER = "../loggerd/pipe_encoder"


encoder_proc = None
def open_stream():
  global encoder_proc
  encoder_proc = subprocess.Popen(PIPE_ENCODER, bufsize=0, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)


cnt = 0
prev_t = 0
if __name__ == "__main__":
  open_stream()

  while True:
    cnt += len(encoder_proc.stderr.read(100)) * 8
    if time.monotonic() - prev_t > 1:
      print(f"{cnt/1e3} kbps")
      prev_t = time.monotonic()
      cnt = 0

