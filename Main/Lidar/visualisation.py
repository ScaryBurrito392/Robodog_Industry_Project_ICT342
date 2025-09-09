# === visualisation.py ===
# Displays grid using OpenCV

import cv2
import asyncio
import numpy as np
from .occupancy_map import get_occupancy_grid_copy

UPDATE_INTERVAL_MS = 100


def start_visualisation_loop():
    while True:
        grid = asyncio.run(get_occupancy_grid_copy())

        display = cv2.resize(grid, (600, 600), interpolation=cv2.INTER_NEAREST)
        display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

        # Draw robot at center
        cv2.rectangle(display, (295, 295), (305, 305), (0, 255, 0), -1)

        cv2.imshow("LIDAR Grid Map", display)
        if cv2.waitKey(UPDATE_INTERVAL_MS) == 27:
            break

    cv2.destroyAllWindows()
