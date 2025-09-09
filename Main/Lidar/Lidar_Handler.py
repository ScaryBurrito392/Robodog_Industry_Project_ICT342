import asyncio
import logging
import numpy as np
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from Main.Lidar.occupancy_map import update_occupancy_grid

logging.getLogger().setLevel(logging.INFO)

LIDAR_TOPIC = "rt/utlidar/voxel_map_compressed"
ROBOT_IP = "192.168.12.1"  # your Go2 IP

_first_frame_keys_logged = False  # one-time debug


def _bytes_to_xyz(arr_like) -> np.ndarray:
    """
    Accept bytes / bytearray / memoryview / np.uint8 ndarray. Interpret as
    little-endian float32 triplets and reshape to (N,3).
    """
    # Raw Python buffer types
    if isinstance(arr_like, (bytes, bytearray, memoryview)):
        buf = np.frombuffer(arr_like, dtype="<f4")
        if buf.size % 3 != 0:
            return np.empty((0, 3), dtype=np.float32)
        return buf.reshape(-1, 3).astype(np.float32, copy=False)

    # Numpy uint8 array that actually holds packed floats
    if isinstance(arr_like, np.ndarray) and arr_like.dtype == np.uint8:
        view = arr_like.view("<f4")
        if view.size % 3 != 0:
            return np.empty((0, 3), dtype=np.float32)
        return view.reshape(-1, 3).astype(np.float32, copy=False)

    return np.empty((0, 3), dtype=np.float32)


def _decode_positions(data_dict) -> np.ndarray:
    """
    Robustly pull Nx3 float32 points out of a variety of field names/types.
    Returns empty array on failure.
    """
    if not isinstance(data_dict, dict) or not data_dict:
        return np.empty((0, 3), dtype=np.float32)

    # (1) Common field names people use for point clouds
    for key in ("positions", "points", "vertices", "xyz", "centers"):
        if key in data_dict:
            raw = data_dict[key]
            # Fast paths
            if isinstance(raw, np.ndarray):
                try:
                    if raw.dtype == np.uint8:
                        return _bytes_to_xyz(raw)
                    arr = np.asarray(raw, dtype=np.float32)
                    return arr.reshape(-1, 3)
                except Exception:
                    return np.empty((0, 3), dtype=np.float32)
            # bytes-like
            pts = _bytes_to_xyz(raw)
            if pts.size:
                return pts

    # (2) Separate coordinate arrays x,y,z
    if all(k in data_dict for k in ("x", "y", "z")):
        try:
            x = np.asarray(data_dict["x"], dtype=np.float32)
            y = np.asarray(data_dict["y"], dtype=np.float32)
            z = np.asarray(data_dict["z"], dtype=np.float32)
            n = min(x.size, y.size, z.size)
            return np.stack([x[:n], y[:n], z[:n]], axis=1)
        except Exception:
            return np.empty((0, 3), dtype=np.float32)

    # (3) Flat buffer + count
    if "buffer" in data_dict:
        buf = _bytes_to_xyz(data_dict["buffer"])
        if buf.size:
            return buf

    return np.empty((0, 3), dtype=np.float32)


def _to_robot_xy(points_xyz: np.ndarray, origin, resolution, width) -> np.ndarray:
    """
    Convert decoded XYZ to robot-centric XY (meters).
    - If magnitudes look like voxel indices, convert to meters via origin/resolution.
    - Otherwise assume meters.
    - Rotate 90° yaw so +x forward, +y left.
    - Center the cloud by XY centroid (keeps robot at (0,0)).
    """
    if points_xyz.size == 0:
        return np.empty((0, 2), dtype=np.float32)

    pts = points_xyz.astype(np.float32, copy=True)
    origin = np.asarray(origin, dtype=np.float32).reshape(3,)
    width = np.asarray(width, dtype=np.float32).reshape(3,)
    resolution = float(resolution)

    # Heuristic: indices-like if values are within a couple widths
    if np.max(np.abs(pts)) <= (np.max(width) * 2.0 + 1.0):
        pts = pts * resolution + origin  # meters

    # 90° yaw (swap x/y) to make +x forward, +y left
    Rz = np.array([[0, -1, 0],
                   [1,  0, 0],
                   [0,  0, 1]], dtype=np.float32)
    pts = pts @ Rz.T

    xy = pts[:, :2]
    # Center by cloud centroid
    xy -= np.mean(xy, axis=0)

    return xy


async def start_lidar_stream():
    print("[🧭] Entered start_lidar_stream()")
    print(f"[📡] Checking robot reachability at {ROBOT_IP}...")
    conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ROBOT_IP)

    try:
        print("[🔌] Calling conn.connect()...")
        await conn.connect()
        print("[✅] Connected to Go2 robot!")
    except Exception as e:
        print(f"[❌] Failed to connect to Go2 robot at {ROBOT_IP}: {e}")
        return

    try:
        await conn.datachannel.disableTrafficSaving(True)
        conn.datachannel.set_decoder(decoder_type='libvoxel')
        print("[🧩] Decoder set to libvoxel")
        conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
        print("[🔦] LIDAR switch sent: on")

        def on_msg(msg):
            # msg should be a dict like {"data": {...}} from the decoder.
            asyncio.create_task(process_lidar(msg))

        conn.datachannel.pub_sub.subscribe(LIDAR_TOPIC, on_msg)

        while True:
            await asyncio.sleep(0.2)

    except Exception as e:
        print(f"[🔥] LIDAR stream error: {e}")


async def process_lidar(message):
    global _first_frame_keys_logged
    try:
        meta = message.get("data", {})
        inner = meta.get("data", {})

        if not _first_frame_keys_logged:
            _first_frame_keys_logged = True
            # one-shot visibility into what we actually receive
            try:
                print(f"[🔍] payload keys: {list(inner.keys())}")
                if "positions" in inner:
                    print(f"[🔍] positions type: {type(inner['positions']).__name__}")
            except Exception:
                pass

        origin = meta.get("origin", [0.0, 0.0, 0.0])
        resolution = meta.get("resolution", 0.05)
        width = meta.get("width", [128, 128, 38])

        points_xyz = _decode_positions(inner)

        # Print a small summary per frame (no spam)
        pc = len(points_xyz)
        if pc:
            print(f"[📥] LIDAR frame: {pc} pts")

        xy = _to_robot_xy(points_xyz, origin, resolution, width)

        # Update occupancy with a generous radius (no accidental culling)
        update_occupancy_grid(xy, clamp_radius_m=100.0)

    except Exception as e:
        logging.error(f"[LIDAR] Processing error: {e}")
