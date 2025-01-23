import numpy as np
import time
from cereal import messaging
from cereal.messaging import PubMaster, SubMaster
from cereal.visionipc import VisionIpcClient, VisionStreamType, VisionBuf

#vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_DRIVER, True)
vipc_client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
while not vipc_client.connect(False):
  time.sleep(0.1)
assert vipc_client.is_connected()
print("connected w buffer size: " + str(vipc_client.buffer_len))

while True:
  buf = vipc_client.recv()
  if buf is None:
    continue
  else:
    print(len(buf.data), buf.height, buf.width, buf.stride, buf.uv_offset)
    np.save('data.npy', buf.data)
    exit()
