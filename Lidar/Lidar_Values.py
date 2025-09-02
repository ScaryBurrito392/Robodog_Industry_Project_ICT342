""" @MrRobotoW at The RoboVerse Discord """
import socketio

""" robert.wagoner@gmail.com """
""" 01/30/2025 """
""" Inspired from lidar_stream.py by @legion1581 at The RoboVerse Discord """

VERSION = "1.0.18"

import asyncio
import logging
import numpy as np
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
import argparse
from datetime import datetime
import os
import sys
import ast



# Constants to enable/disable features
ENABLE_POINT_CLOUD = True
SAVE_LIDAR_DATA = True

# File paths
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
LIDAR_CSV_FILE = f"lidar_data_{timestamp}.csv"

# Global variables
lidar_csv_file = None
lidar_csv_writer = None

lidar_buffer = []
message_count = 0  # Counter for processed LIDAR messages
reconnect_interval = 5  # Time (seconds) before retrying connection

# Constants
MAX_RETRY_ATTEMPTS = 100

ROTATE_X_ANGLE = np.pi / 2  # 90 degrees
ROTATE_Z_ANGLE = np.pi  # 90 degrees

minYValue = 0
maxYValue = 100

# Parse command-line arguments
parser = argparse.ArgumentParser(description=f"LIDAR Viz v{VERSION}")
parser.add_argument("--version", action="version", version=f"LIDAR Viz v{VERSION}")
parser.add_argument("--cam-center", action="store_true", help="Put Camera at the Center")
parser.add_argument("--type-voxel", action="store_true", help="Voxel View")
parser.add_argument("--csv-read", type=str, help="Read from CSV files instead of WebRTC")
parser.add_argument("--csv-write", action="store_true", help="Write CSV data file")
parser.add_argument("--skip-mod", type=int, default=1, help="Skip messages using modulus (default: 1, no skipping)")
parser.add_argument('--minYValue', type=int, default=0, help='Minimum Y value for the plot')
parser.add_argument('--maxYValue', type=int, default=100, help='Maximum Y value for the plot')
args = parser.parse_args()

# minYValue = args.minYValue
# maxYValue = args.maxYValue
# SAVE_LIDAR_DATA = args.csv_write


# @socketio.on('check_args')
# def handle_check_args():
#     global ack_received
#     typeFlag = 0b0101  # default iso cam & point cloud
#     if args.cam_center:
#         typeFlag |= 0b0010
#     if args.type_voxel:
#         typeFlag &= ~0b1000  # disable point cloud
#         typeFlag |= 0b1000  # Set voxel flag
#     typeFlagBinary = format(typeFlag, "04b")
#     socketio.emit("check_args_ack", {"type": typeFlagBinary})



def filter_points(points, percentage):
    """Filter points to skip plotting points within a certain percentage of distance to each other."""
    return points  # No filtering


def rotate_points(points, x_angle, z_angle):
    """Rotate points around the x and z axes by given angles."""
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(x_angle), -np.sin(x_angle)],
        [0, np.sin(x_angle), np.cos(x_angle)]
    ])

    rotation_matrix_z = np.array([
        [np.cos(z_angle), -np.sin(z_angle), 0],
        [np.sin(z_angle), np.cos(z_angle), 0],
        [0, 0, 1]
    ])

    points = points @ rotation_matrix_x.T
    points = points @ rotation_matrix_z.T
    return points


async def lidar_webrtc_connection():
    """Connect to WebRTC and process LIDAR data."""
    global lidar_buffer, message_count
    retry_attempts = 0

    # checkArgs()

    while retry_attempts < MAX_RETRY_ATTEMPTS:
        try:
            conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.12.1")  # WebRTC IP
            # _webrtc_connection = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
            # _webrtc_connection = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

            # Connect to WebRTC
            logging.info("Connecting to WebRTC...")
            await conn.connect()
            logging.info("Connected to WebRTC.")
            retry_attempts = 0  # Reset retry attempts on successful connection

            # Disable traffic saving mode
            await conn.datachannel.disableTrafficSaving(True)

            # Turn LIDAR sensor on
            conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")


            async def lidar_callback_task(message):
                """Task to process incoming LIDAR data."""
                if not ENABLE_POINT_CLOUD:
                    return

                try:
                    global message_count
                    if message_count % args.skip_mod != 0:
                        message_count += 1
                        return

                    positions = message["data"]["data"].get("positions", [])
                    origin = message["data"].get("origin", [])
                    points = np.array([positions[i:i + 3] for i in range(0, len(positions), 3)], dtype=np.float32)
                    total_points = len(points)
                    unique_points = np.unique(points, axis=0)

                    print(unique_points)


                except Exception as e:
                    logging.error(f"Error in LIDAR callback: {e}")

            # Subscribe to LIDAR voxel map messages
            conn.datachannel.pub_sub.subscribe(
                "rt/utlidar/voxel_map_compressed",
                lambda message: asyncio.create_task(lidar_callback_task(message))
            )

            # Keep the connection active
            while True:
                await asyncio.sleep(1)

        except Exception as e:
            logging.error(f"An error occurred: {e}")
            logging.info(
                f"Reconnecting in {reconnect_interval} seconds... (Attempt {retry_attempts + 1}/{MAX_RETRY_ATTEMPTS})")
            try:
                print("peace out")
                await conn.disconnect()
            except Exception as e:
                logging.error(f"Error during disconnect: {e}")
            await asyncio.sleep(reconnect_interval)
            retry_attempts += 1

    logging.error("Max retry attempts reached. Exiting.")



def start_webrtc():
    """Run WebRTC connection in a separate asyncio loop."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(lidar_webrtc_connection())


async def main():
    import threading

    webrtc_thread = threading.Thread(target=start_webrtc, daemon=True)
    webrtc_thread.start()
    await asyncio.sleep(1000)

if __name__ == "__main__":
    asyncio.run(main())