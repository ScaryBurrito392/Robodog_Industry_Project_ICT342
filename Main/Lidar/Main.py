# === Main.py ===
# Entry point: spins an asyncio loop in a background thread for LIDAR,
# runs the OpenCV window on the main thread.

import sys
import argparse
import asyncio
import threading
from Main.Lidar.Lidar_Handler import start_lidar_stream
from Main.Lidar.visualisation import start_visualisation_loop




def _run_loop(loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

def _parse_args():
    p = argparse.ArgumentParser(description="Go2 LIDAR mapper")
    p.add_argument("--sim", action="store_true",
                   help="run with synthetic LIDAR (no robot needed)")
    return p.parse_args()

if __name__ == "__main__":
    args = _parse_args()
    print("[🟢] Starting autonomous LIDAR mapper...")

    try:
        # Background asyncio loop
        loop = asyncio.new_event_loop()
        print("[🔁] Creating LIDAR task...")
        bg = threading.Thread(target=_run_loop, args=(loop,), daemon=True)
        bg.start()

        if args.sim:
            print("[🧪] SIM MODE enabled")
            # import here to avoid pulling RTC deps when sim only
            from Main.Lidar.sim_lidar import start_sim_lidar_stream
            asyncio.run_coroutine_threadsafe(start_sim_lidar_stream(), loop)
        else:
            from Main.Lidar.Lidar_Handler import start_lidar_stream
            asyncio.run_coroutine_threadsafe(start_lidar_stream(), loop)

        # Run visualisation on the main thread
        start_visualisation_loop()

    except KeyboardInterrupt:
        print("\n[🟥] Program stopped by user.")
