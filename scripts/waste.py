#!/usr/bin/env python3
import os
import numpy as np
from common.realtime import sec_since_boot
from multiprocessing import Process
from setproctitle import setproctitle  # pylint: disable=no-name-in-module

def waste(core):
  os.sched_setaffinity(0, [core,])  # pylint: disable=no-member

  m1 = np.zeros((200, 200)) + 0.8
  m2 = np.zeros((200, 200)) + 1.2

  i = 1
  st = sec_since_boot()
  j = 0
  while 1:
    if (i % 100) == 0:
      setproctitle("%3d: %8d" % (core, i))
      lt = sec_since_boot()
      print("%3d: %8d %f  %.2f" % (core, i, lt-st, j))
      st = lt
    i += 1
    j = np.sum(np.matmul(m1, m2))

def main(gctx=None):
  print("1-2 seconds is baseline")
  for i in range(os.cpu_count()):
    p = Process(target=waste, args=(i,))
    p.start()

if __name__ == "__main__":
  main()
