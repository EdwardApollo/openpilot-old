#!/usr/bin/env python
import json
import time
import random
import asyncio
import threading
from typing import OrderedDict
import numpy as np
from av import VideoFrame
from aiortc import (
  RTCIceCandidate,
  RTCPeerConnection,
  RTCSessionDescription,
  RTCRtpCodecCapability,
  VideoStreamTrack,
)
from aiortc.contrib.media import MediaBlackhole
from aiortc.contrib.signaling import BYE, ApprtcSignaling, object_from_string, object_to_string
from cereal.visionipc.visionipc_pyx import VisionIpcClient, VisionStreamType # pylint: disable=no-name-in-module, import-error

class VisionIpcTrack(VideoStreamTrack):
  def __init__(self, vision_stream_type):
    super().__init__()
    self.vipc_client = VisionIpcClient("camerad", vision_stream_type, True)

  async def recv(self):
    pts, time_base = await self.next_timestamp()

    # Connect if not connected
    while not self.vipc_client.is_connected():
      self.vipc_client.connect(True)
      print("connected")

    raw_frame = None
    while raw_frame is None or not raw_frame.any():
      raw_frame = self.vipc_client.recv()

    raw_frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((self.vipc_client.height, self.vipc_client.width, 3))
    frame = VideoFrame.from_ndarray(raw_frame, "bgr24")
    frame.pts = pts
    frame.time_base = time_base

    return frame


class AthenaSignaling:
  def __init__(self):
    self._rx_buffer = []
    self._tx_buffer = []

  async def connect(self):
    return {"is_initiator": False}

  async def close(self):
    pass

  async def receive(self):
    while len(self._rx_buffer) == 0:
      await asyncio.sleep(0.1)

    item = self._rx_buffer[0]
    del self._rx_buffer[0]

    print("RX", item)

    return object_from_string(item)

  async def send(self, descr):
    print("TX", descr)
    self._tx_buffer.append(object_to_string(descr))

  # Methods used by athena
  def push(self, obj):
    if isinstance(obj, dict):
      obj = json.dumps(obj)

    print("Push", obj)
    self._rx_buffer.append(obj)

  def pop(self):
    if len(self._tx_buffer) == 0:
      return None

    item = self._tx_buffer[0]
    del self._tx_buffer[0]

    print("Pop", item)
    return item

last_kick_time = time.monotonic()
TIMEOUT = 60

async def run(pc, signaling, recorder):
  global last_kick_time

  def add_video_track():
    pc.addTrack(VisionIpcTrack(VisionStreamType.VISION_STREAM_RGB_FRONT))
    for t in pc.getTransceivers():
      if t.kind == "video":
        # this uses a CPU encoder. Tried to get it to use omx, but it's complaining about "No in or out port found"
        t.setCodecPreferences([
          RTCRtpCodecCapability(
            mimeType="video/H264",
            clockRate=90000,
            channels=None,
            parameters=OrderedDict([
              ("packetization-mode", "1"),
              ("level-asymmetry-allowed", "1"),
              ("profile-level-id", "42001f"),
            ])
          )
        ])

  @pc.on("track")
  def on_track(track):
    recorder.addTrack(track)

  # Setup
  params = await signaling.connect()
  if params["is_initiator"] == "true":
    # send offer
    add_video_track()
    await pc.setLocalDescription(await pc.createOffer())
    await signaling.send(pc.localDescription)

  # Event loop
  while time.monotonic() - last_kick_time <= TIMEOUT:
    obj = await signaling.receive()
    if isinstance(obj, RTCSessionDescription):
      await pc.setRemoteDescription(obj)
      if obj.type == "offer":
        # send answer
        add_video_track()
        answer = await pc.createAnswer()
        print(answer)
        await pc.setLocalDescription(answer)
        await signaling.send(pc.localDescription)
    elif isinstance(obj, RTCIceCandidate):
      await pc.addIceCandidate(obj)
    elif obj is BYE:
      print("Exiting")
      break

def kick_streamer():
  global last_kick_time
  last_kick_time = time.monotonic()

def start_streamer(signaling):
  pc = RTCPeerConnection()
  recorder = MediaBlackhole()

  # Run event loop
  loop = asyncio.new_event_loop()
  try:
    loop.run_until_complete(
      run(pc, signaling, recorder)
    )
  except KeyboardInterrupt:
    pass
  finally:
    loop.run_until_complete(recorder.stop())
    loop.run_until_complete(signaling.close())
    loop.run_until_complete(pc.close())


if __name__ == "__main__":
  # Setup signaling and peer connection over apprtc for standalone testing
  room = "".join([random.choice("0123456789") for x in range(10)])
  print(f"Joining {room}")
  signaling = ApprtcSignaling(room)

  thread = threading.Thread(target=start_streamer, args=(signaling,))
  thread.start()

  while True:
    kick_streamer()
    time.sleep(1)
