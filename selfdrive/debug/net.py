#!/usr/bin/env python3
import os
import random
import time
import multiprocessing

speedtest = "/tmp/speedtest-cli"

def log(msg):
  os.system(f"log -t crasher '{msg}'")

def speedtest():
  while True:
    args = random.choice(['', '--no-download', '--no-upload'])
    os.system(f"{speedtest} --timeout 1 {args}")

def crasher():
  cnt = 0
  start = time.monotonic()
  while True:
    # TODO: also do tethering

    cnt += 1
    d = random.choice(['enable', 'disable'])
    w = random.choice(['enable', 'disable'])
    w = 'enable'

    log(f"#{str(cnt).ljust(4)}: data={d} wifi={w}, {round(time.monotonic() - start)}s")
    print(f"#{str(cnt).ljust(4)}: data={d} wifi={w}, {round(time.monotonic() - start)}s")

    os.system(f"LD_LIBRARY_PATH= svc data {d}")
    time.sleep(random.uniform(0., 10.))

    os.system(f"LD_LIBRARY_PATH= svc wifi {w}")
    time.sleep(random.uniform(0., 10.))


if __name__ == "__main__":
  os.system(f"curl -Lo {speedtest} https://raw.githubusercontent.com/sivel/speedtest-cli/master/speedtest.py")
  os.system(f"chmod +x {speedtest}")

  procs = []
  try:
    for _ in range(5):
      p = multiprocessing.Process(target=speedtest)
      p.daemon = True
      p.start()
      procs.append(p)
    crasher()
  finally:
    for p in procs:
      p.terminate()
      p.join(1)
      if p.exitcode is None:
        p.kill()
      p.join()
