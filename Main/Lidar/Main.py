# === Main.py ===
# Entry point: spins an asyncio loop in a background thread for LIDAR,
# runs the OpenCV window on the main thread.

import asyncio
import threading
from Main.Lidar.Lidar_Handler import start_lidar_stream
from Main.Lidar.visualisation import start_visualisation_loop


def _run_loop(loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(loop)
    loop.run_forever()


if __name__ == "__main__":
    print("[🟢] Starting autonomous LIDAR mapper...")
    try:
        # Background asyncio loop
        loop = asyncio.new_event_loop()
        print("[🔁] Creating LIDAR stream task...")
        bg = threading.Thread(target=_run_loop, args=(loop,), daemon=True)
        bg.start()

        # Schedule the async LIDAR stream onto that loop
        asyncio.run_coroutine_threadsafe(start_lidar_stream(), loop)

        # Run visualisation on the main thread
        start_visualisation_loop()

    except KeyboardInterrupt:
        print("\n[🟥] Program stopped by user.")
