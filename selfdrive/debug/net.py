#!/usr/bin/env python3

import os
import random
import time

def log(msg):
  os.system(f"log -t crasher '{msg}'")

cnt = 0
while True:

  # TODO: also do tethering

  log(f"run #{cnt}")
  cnt += 1

  a = random.choice(['enable', 'disable'])
  os.system(f"LD_LIBRARY_PATH= svc data {a}")
  time.sleep(random.uniform(0., 5.))

  a = random.choice(['enable', 'disable'])
  os.system(f"LD_LIBRARY_PATH= svc wifi {a}")
  time.sleep(random.uniform(0., 5.))
