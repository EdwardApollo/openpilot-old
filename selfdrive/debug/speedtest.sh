#!/usr/bin/bash

curl -Lo speedtest-cli https://raw.githubusercontent.com/sivel/speedtest-cli/master/speedtest.py
chmod +x speedtest-cli

while true; do
  ./speedtest-cli
done
