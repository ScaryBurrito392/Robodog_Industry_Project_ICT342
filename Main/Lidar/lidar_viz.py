# lidar_viz.py
from __future__ import annotations
import logging
from voxel_decode import decode_voxel_frame
from voxel_viewer import VoxelViewer

_viewer = None

def _get_viewer():
    global _viewer
    if _viewer is None:
        _viewer = VoxelViewer()
    return _viewer

def handle_voxel_message(message_bytes: bytes):
    """
    Call this from your RTC data-channel 'message' callback when you receive a binary frame.
    It will decode and push to the Open3D window.
    """
    try:
        pts, origin, width, res = decode_voxel_frame(message_bytes)
        logging.info(f"[vox] {len(pts)} pts | origin={origin} | width={width} | res={res}")
        _get_viewer().update(pts, origin, width, res)
    except Exception as e:
        logging.exception(f"[vox] decode error: {e}")
