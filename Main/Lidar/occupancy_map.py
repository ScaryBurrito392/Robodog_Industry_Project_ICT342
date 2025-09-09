# === occupancy_map.py ===
import numpy as np
import asyncio

MAP_SIZE = 300
RESOLUTION = 0.05
ROBOT_CENTER = MAP_SIZE // 2

occupancy_grid = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
map_lock = asyncio.Lock()

async def update_occupancy_grid(points):
    global occupancy_grid
    async with map_lock:
        for x, y, _ in points:
            mx = int(ROBOT_CENTER + x / RESOLUTION)
            my = int(ROBOT_CENTER + y / RESOLUTION)
            if 0 <= mx < MAP_SIZE and 0 <= my < MAP_SIZE:
                occupancy_grid[my, mx] = 255

async def get_occupancy_grid_copy():
    async with map_lock:
        return occupancy_grid.copy()
