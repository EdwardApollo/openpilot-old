#!/usr/bin/env python

import random
import asyncio
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
from aiortc.contrib.signaling import BYE, ApprtcSignaling

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


async def run(pc, signaling, recorder):
  def add_video_track():
    pc.addTrack(VisionIpcTrack(VisionStreamType.VISION_STREAM_RGB_FRONT))
    for t in pc.getTransceivers():
      if t.kind == "video":
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
  while True:
    obj = await signaling.receive()
    if isinstance(obj, RTCSessionDescription):
      await pc.setRemoteDescription(obj)
      if obj.type == "offer":
        # send answer
        add_video_track()
        await pc.setLocalDescription(await pc.createAnswer())
        await signaling.send(pc.localDescription)
    elif isinstance(obj, RTCIceCandidate):
      await pc.addIceCandidate(obj)
    elif obj is BYE:
      print("Exiting")
      break

if __name__ == "__main__":
  # Setup signaling and peer connection
  room = "".join([random.choice("0123456789") for x in range(10)])
  print(f"Joining {room}")
  signaling = ApprtcSignaling(room)
  pc = RTCPeerConnection()
  recorder = MediaBlackhole()

  # Run event loop
  loop = asyncio.get_event_loop()
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