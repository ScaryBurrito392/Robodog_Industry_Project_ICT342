"""
LIDAR Visualization System using Flask, Socket.IO, WebRTC, and Three.js
Version: 1.0.18
Author: @MrRobotoW (Robert Wagoner) from The RoboVerse Discord
Inspired by: lidar_stream.py by @legion1581 from The RoboVerse Discord


This script provides a real-time and playback visualization system for LIDAR data, enabling either live connection
via WebRTC to a Go2 robot or offline replay from previously saved CSV files. The visual output is served via a
webpage that uses Three.js to render either point clouds or voxel representations of the LIDAR scan in 3D space.


It includes features such as:
- Configurable camera position (centered or offset)
- Point cloud or voxel rendering mode
- Live CSV export for post-processing or debugging
- Adjustable Y-axis filtering to eliminate ground/noise
- Basic retry mechanism for failed WebRTC sessions


Back-end: Python (Flask, Socket.IO, asyncio, numpy, argparse)
Front-end: JavaScript (Three.js, OrbitControls, Socket.IO client)


Run with:
python script.py --csv-write --type-voxel --cam-center
python script.py --csv-read lidar_data.csv
"""


VERSION = "1.0.18"


# =============================
# IMPORTS
# =============================
import asyncio # Async event loop for non-blocking I/O
import logging # Logging support
import csv # For reading/writing LIDAR data
import numpy as np # Efficient matrix operations (used for point cloud manipulation)
from flask import Flask, render_template_string # Flask web server to host frontend
from flask_socketio import SocketIO # Real-time WebSocket communication with browser
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod # WebRTC API
import argparse # CLI parsing
from datetime import datetime # Timestamping CSV outputs
import os, sys, ast # System ops, and safe string->list conversions


# =============================
# INITIAL SETUP
# =============================
csv.field_size_limit(sys.maxsize) # Prevent truncation of large CSV fields
app = Flask(__name__) # Create Flask app instance
socketio = SocketIO(app, async_mode='threading') # Enable async communication
logging.basicConfig(level=logging.FATAL) # Silence most logs unless fatal


# =============================
# GLOBAL CONSTANTS AND FLAGS
# =============================
ENABLE_POINT_CLOUD = True # Enable or disable point cloud logic (useful for debug)
SAVE_LIDAR_DATA = True # Controls CSV file output (can be set via --csv-write)


# File paths and data
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
LIDAR_CSV_FILE = f"lidar_data_{timestamp}.csv" # Output file
lidar_csv_file = None # File handle
lidar_csv_writer = None # CSV writer handle


# Buffer and counters
lidar_buffer = []
socketio.run(app, host="127.0.0.1", port=8080, debug=False)