# === Lidar_Handler.py ===
# Connects to robot and handles LIDAR data input

import asyncio
import logging
import numpy as np
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from Main.Lidar.occupancy_map import update_occupancy_grid

LIDAR_TOPIC = "rt/utlidar/voxel_map_compressed"
ROBOT_IP = "192.168.12.1"

async def start_lidar_stream():
    conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP)
    await conn.connect()
    await conn.datachannel.disableTrafficSaving(True)
    conn.datachannel.set_decoder(decoder_type='libvoxel')
    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")

    def callback(msg):
        print("[📥] LIDAR message received:", msg)
        asyncio.create_task(process_lidar(msg))


    conn.datachannel.pub_sub.subscribe(LIDAR_TOPIC, callback)

    while True:
        await asyncio.sleep(1)


async def process_lidar(message):
    try:
        positions = message["data"]["data"].get("positions", [])
        print(f"[🧩] Extracted {len(positions)} position values")  # add this

        points = np.array([positions[i:i+3] for i in range(0, len(positions), 3)], dtype=np.float32)

        if points.size == 0:
            return

        # Apply rotation to align map correctly
        rotated = points @ np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).T

        await update_occupancy_grid(rotated)

    except Exception as e:
        logging.error(f"LIDAR processing error: {e}")