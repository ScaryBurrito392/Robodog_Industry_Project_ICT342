# === main.py ===
# Entry point that launches LIDAR streaming, mapping, and decision-making

import asyncio
from Main.Lidar.Lidar_Handler import start_lidar_stream
from Main.Lidar.visualisation import start_visualisation_loop

if __name__ == "__main__":
    print("[🟢] Starting autonomous LIDAR mapper...")

    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Launch LIDAR data handling task
        loop.create_task(start_lidar_stream())

        # Start OpenCV loop (runs forever)
        start_visualisation_loop()

    except KeyboardInterrupt:
        print("\n[🟥] Program stopped by user.")
