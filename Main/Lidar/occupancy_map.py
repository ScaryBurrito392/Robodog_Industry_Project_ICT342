import numpy as np
from threading import Lock

MAP_SIZE = 400          # grid cells (square)
RESOLUTION = 0.05       # meters per cell (5 cm)
ROBOT_CENTER = MAP_SIZE // 2

_occupancy = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
_lock = Lock()
_frame_counter = 0
_nonzero_last = 0


def clear_grid():
    with _lock:
        _occupancy.fill(0)


def update_occupancy_grid(points_xy, *, clamp_radius_m: float = 100.0):
    """
    Mark cells as occupied from Nx2 point array (meters), robot at (0,0).
    """
    global _frame_counter, _nonzero_last
    if points_xy is None or points_xy.size == 0:
        return

    with _lock:
        r2 = np.sum(points_xy ** 2, axis=1)
        keep = r2 <= (clamp_radius_m ** 2)
        pts = points_xy[keep]

        if pts.size == 0:
            return

        mx = (ROBOT_CENTER + (pts[:, 0] / RESOLUTION)).astype(np.int32)
        my = (ROBOT_CENTER + (pts[:, 1] / RESOLUTION)).astype(np.int32)
        mask = (mx >= 0) & (mx < MAP_SIZE) & (my >= 0) & (my < MAP_SIZE)
        _occupancy[my[mask], mx[mask]] = 255

        # cheap heartbeat for the UI overlay
        _frame_counter += 1
        if (_frame_counter % 10) == 0:
            _nonzero_last = int((_occupancy > 0).sum())


def get_occupancy_grid_copy():
    with _lock:
        return _occupancy.copy()


def get_debug_occupancy_count():
    with _lock:
        return int((_occupancy > 0).sum())
