# voxel_decode.py
"""
Decode bytes from the topic 'rt/utlidar/voxel_map_compressed' into XYZ points.

Wire format (what your logs show):
[2B little-endian JSON length][2B tag][JSON bytes][optional 4B sentinel 1f 00 01 00][payload]

JSON contains:
  resolution: float (meter per voxel)
  origin: [ox, oy, oz] (grid origin, meters)
  width: [Wx, Wy, Wz] (grid dimensions, voxels)
  src_size: int (uncompressed bytes for the occupancy bitfield)

Payload is usually zlib-compressed bitfield of size src_size.
"""

from __future__ import annotations
import json
import zlib
import numpy as np

try:
    import lz4.block as lz4b
except Exception:
    lz4b = None


def _parse_header(frame: bytes):
    mv = memoryview(frame)
    if len(mv) < 8:
        raise ValueError("frame too short")
    json_len = int.from_bytes(mv[0:2], "little")
    # two extra bytes (tag/magic) frequently appear after the length
    json_start = 4
    json_end = json_start + json_len
    meta = json.loads(mv[json_start:json_end].tobytes().decode("utf-8"))
    pos = json_end
    # many frames have a sentinel 1f 00 01 00 between JSON and payload
    if pos + 4 <= len(mv) and mv[pos:pos+4].tobytes() == b"\x1f\x00\x01\x00":
        pos += 4
    return meta, mv[pos:].tobytes()


def _decompress(payload: bytes, expect_len: int) -> bytes:
    attempts = []
    attempts.append(("zlib", lambda b: zlib.decompress(b)))
    attempts.append(("zlib_raw", lambda b: zlib.decompress(b, -zlib.MAX_WBITS)))
    if lz4b:
        attempts.append(("lz4", lambda b: lz4b.decompress(b, uncompressed_size=expect_len)))

    for name, fn in attempts:
        try:
            out = fn(payload)
            if len(out) == expect_len:
                return out
        except Exception:
            pass

    # some send uncompressed already
    if len(payload) == expect_len:
        return payload

    raise ValueError(f"decompress failed (need {expect_len} bytes)")


def _bits_to_points(bitbytes: bytes, width_xyz, resolution: float, origin):
    """
    Treat the uncompressed bytes as a packed occupancy bitfield.
    Assume X-fastest, then Y, then Z (common layout). If your producer differs,
    flip/permute axes where noted below.
    """
    Wx, Wy, Wz = map(int, width_xyz)
    total = Wx * Wy * Wz
    arr = np.frombuffer(bitbytes, dtype=np.uint8)

    # Try both bit orders (producers vary)
    for bitorder in ("little", "big"):
        bits = np.unpackbits(arr, bitorder=bitorder)
        if bits.size >= total:
            occ = bits[:total].reshape((Wz, Wy, Wx))  # z, y, x
            z, y, x = np.nonzero(occ)
            if x.size:  # got some points
                pts = np.stack([x, y, z], axis=1).astype(np.float32)
                o = np.asarray(origin, np.float32)
                return pts * resolution + (o + resolution * 0.5)

    # no occupied voxels
    return np.empty((0, 3), dtype=np.float32)


def decode_voxel_frame(frame: bytes):
    """
    Input: raw bytes payload from the RTC data channel (binary message)
    Output: (points Nx3 float32, origin, width_xyz, resolution)
    """
    meta, payload = _parse_header(frame)

    data = meta["data"]
    resolution = float(data["resolution"])
    origin = data["origin"]
    width = data["width"]          # [Wx, Wy, Wz]
    src_size = int(data["src_size"])

    raw = _decompress(payload, src_size)
    pts = _bits_to_points(raw, width, resolution, origin)
    return pts, origin, width, resolution
