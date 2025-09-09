import cv2
import numpy as np
from Main.Lidar.occupancy_map import get_occupancy_grid_copy, get_debug_occupancy_count, MAP_SIZE

UPDATE_INTERVAL_MS = 100

def start_visualisation_loop():
    print("[🧠] Launching visualisation...")
    while True:
        grid = get_occupancy_grid_copy()

        display = cv2.resize(grid, (600, 600), interpolation=cv2.INTER_NEAREST)
        display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

        # Robot marker (center)
        cx = cy = 600 // 2
        cv2.rectangle(display, (cx - 5, cy - 5), (cx + 5, cy + 5), (0, 255, 0), -1)

        # HUD
        cv2.putText(display, "LIDAR Grid Map (Top-Down)", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(display, f"Grid: {MAP_SIZE}x{MAP_SIZE}", (10, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1, cv2.LINE_AA)
        cv2.putText(display, f"Occupied px: {get_debug_occupancy_count()}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 255), 1, cv2.LINE_AA)

        cv2.imshow("LIDAR Grid Map", display)
        if cv2.waitKey(UPDATE_INTERVAL_MS) == 27:  # ESC
            break

    cv2.destroyAllWindows()
