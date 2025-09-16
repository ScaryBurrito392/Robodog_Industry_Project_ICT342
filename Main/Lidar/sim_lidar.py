# Main/Lidar/sim_lidar.py
# simulated lidar environment readings from dog
import asyncio
import numpy as np
from Main.Lidar.Lidar_Handler import process_lidar

def _make_room_pts(t: float) -> np.ndarray:
    """
    Synthetic 'room' around the robot:
    - a box of walls ~5m wide
    - a pillar on the right that slowly 'moves' (sim turn/scan)
    """
    rng = np.random.default_rng(42)

    # Walls (top, bottom, left, right)
    x = np.linspace(-2.5, 2.5, 400, dtype=np.float32)
    y_top = np.full_like(x, 3.0);     top = np.c_[x, y_top, np.zeros_like(x)]
    y_bot = np.full_like(x, -2.5);    bot = np.c_[x, y_bot, np.zeros_like(x)]

    y = np.linspace(-2.5, 2.5, 400, dtype=np.float32)
    x_left = np.full_like(y, -2.5);   left  = np.c_[x_left, y, np.zeros_like(y)]
    x_right = np.full_like(y,  2.5);  right = np.c_[x_right, y, np.zeros_like(y)]

    # Pillar that orbits slowly
    ang = t * 0.4
    cx, cy = 1.2 + 0.4*np.cos(ang), 0.8 + 0.4*np.sin(ang)
    theta = np.linspace(0, 2*np.pi, 200, dtype=np.float32)
    pillar = np.c_[cx + 0.25*np.cos(theta), cy + 0.25*np.sin(theta), np.zeros_like(theta)]

    pts = np.vstack([top, bot, left, right, pillar]).astype(np.float32)

    # Add mild noise like real scans
    pts[:, :2] += rng.normal(scale=0.01, size=pts[:, :2].shape).astype(np.float32)
    return pts

async def start_sim_lidar_stream():
    print("[🧪] SIM MODE: generating synthetic LIDAR frames (no robot needed).")
    # Values that keep your pipeline happy; resolution=1.0 avoids unit confusion.
    meta = {
        "origin": [0.0, 0.0, 0.0],
        "resolution": 1.0,
        "width": [256, 256, 1],   # arbitrary here
    }

    t = 0.0
    while True:
        pts = _make_room_pts(t)  # Nx3 float32
        message = {
            "type": "msg",
            "topic": "rt/utlidar/voxel_map_compressed",
            "data": {
                **meta,
                "data": {
                    "positions": pts  # process_lidar() already accepts ndarray Nx3
                }
            }
        }
        await process_lidar(message)
        await asyncio.sleep(0.1)  # ~10 Hz
        t += 0.1
