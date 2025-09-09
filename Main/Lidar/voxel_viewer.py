# voxel_viewer.py
"""
Lightweight Open3D viewer you can push new point clouds into.
Runs its own render loop so you can call .update(...) from any thread.
"""

from __future__ import annotations
import threading
import time
import numpy as np
import open3d as o3d


class VoxelViewer:
    def __init__(self, title: str = "Voxel Map", w: int = 1280, h: int = 800):
        self._latest = None    # (pts, origin, width, res)
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, args=(title, w, h), daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)

    def update(self, pts: np.ndarray, origin, width, res: float):
        # replace the pending frame
        with self._lock:
            self._latest = (pts, origin, width, res)

    # ---------- internals ----------

    def _fit_once(self, vis, origin, width, res):
        origin = np.array(origin, dtype=float)
        size = np.array(width, dtype=float) * float(res)
        bb = o3d.geometry.AxisAlignedBoundingBox(origin, origin + size)
        ctr = vis.get_view_control()
        ctr.set_lookat(bb.get_center())
        ctr.set_front([0.6, -0.7, 0.3])
        ctr.set_up([0.0, 0.0, 1.0])
        ctr.set_zoom(0.6)

    def _loop(self, title, w, h):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=title, width=w, height=h)

        opt = vis.get_render_option()
        opt.background_color = np.array([0.02, 0.04, 0.06])
        opt.point_size = 2.0

        pcd = o3d.geometry.PointCloud()
        pcd.paint_uniform_color([0.2, 0.8, 1.0])
        vis.add_geometry(pcd)

        fit_done = False

        try:
            while self._running:
                frame = None
                with self._lock:
                    if self._latest is not None:
                        frame = self._latest
                        self._latest = None

                if frame is not None:
                    pts, origin, width, res = frame
                    if len(pts):
                        pcd.points = o3d.utility.Vector3dVector(pts)
                        if not fit_done:
                            self._fit_once(vis, origin, width, res)
                            fit_done = True
                    vis.update_geometry(pcd)

                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.02)
        finally:
            vis.destroy_window()
