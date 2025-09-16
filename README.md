# Robodog_Industry_Project_ICT342


# LIDAR Simulator (sim_lidar.py)

When you don’t have access to the robot, you can still test the mapping stack end-to-end.
Main/Lidar/sim_lidar.py fakes the data-channel by generating synthetic point clouds (walls + a moving pillar) and pushing them through the exact same function your real frames use: Lidar_Handler.process_lidar(...).

Why use it

No hardware required; develop anywhere.

Exercises decoding → occupancy grid → visualisation without code changes.

Deterministic scene with mild noise to resemble real scans.

# How to run
Let Main pick real vs sim

Update Main/Lidar/Main.py to accept a --sim flag:
# Main/Lidar/Main.py
import sys
import asyncio
import threading
import argparse

from Main.Lidar.Lidar_Handler import start_lidar_stream
from Main.Lidar.sim_lidar import start_sim_lidar_stream
from Main.Lidar.visualisation import start_visualisation_loop

def _run_loop(loop: asyncio.AbstractEventLoop):
    asyncio.set_event_loop(loop)
    loop.run_forever()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", action="store_true", help="Run with simulated LIDAR instead of robot")
    args = parser.parse_args()

    print("[🟢] Starting autonomous LIDAR mapper..." + (" (SIM)" if args.sim else ""))
    try:
        loop = asyncio.new_event_loop()
        print("[🔁] Creating stream task...")
        bg = threading.Thread(target=_run_loop, args=(loop,), daemon=True)
        bg.start()

        # Choose real robot or simulator
        coro = start_sim_lidar_stream() if args.sim else start_lidar_stream()
        asyncio.run_coroutine_threadsafe(coro, loop)

        start_visualisation_loop()
    except KeyboardInterrupt:
        print("\n[🟥] Program stopped by user.")


activate your venv, then from repo root:
python -m Main.Lidar.Main --sim

You should see the same OpenCV window, but this time Occupied px will tick up and you’ll see “walls” paint in—even with no dog connected.


# How it works

Emits messages shaped like the Go2 voxel topic:

data.positions: Nx3 float32 points (meters)

plus origin, resolution, width fields to keep the pipeline happy

Calls process_lidar(message) at ~10 Hz.

# Customize

Edit _make_room_pts(t) to change geometry (rooms, corridors, moving objects).

Tweak noise level, frame rate (sleep), or map scale.

# Troubleshooting

Only a green square? Make sure you ran with --sim and the console prints SIM MODE.

If the window opens but stays empty, confirm process_lidar handles data["positions"] (it does in this repo).
