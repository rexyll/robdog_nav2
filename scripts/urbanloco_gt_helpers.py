"""Pure helpers for the fixed UrbanLoco HK-Data20190117 INSPVAX CDR layout."""
from __future__ import annotations

import math
import struct
from typing import Sequence

# These offsets are specific to UrbanLoco HK-Data20190117 INSPVAX CDR.
# Authoritative cross-checks: NovAtel novatel_oem7_driver commit
# 0c0e135b71c5920b845982358f6c7ec71a952b5e (INSPVAX.msg) and Intel
# edge-ai-suites commit decbb06fc1b746b229957b66b92ab972e1f05609
# (point-lio-demo/scripts/extract_ulhk_gt.py). Revalidate any other schema.
LAT_OFFSET = 60
LON_OFFSET = 68
HEIGHT_OFFSET = 76
MIN_CDR_SIZE = HEIGHT_OFFSET + 8

# WGS84 ellipsoid.
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


def parse_inspvax_position(payload: bytes) -> tuple[float, float, float]:
    """Read latitude[deg], longitude[deg], height[m] from known CDR offsets."""
    if len(payload) < MIN_CDR_SIZE:
        raise ValueError(
            f"INSPVAX CDR payload too short: {len(payload)} bytes; "
            f"need at least {MIN_CDR_SIZE}"
        )
    lat, lon, height = struct.unpack_from("<ddd", payload, LAT_OFFSET)
    if not all(math.isfinite(value) for value in (lat, lon, height)):
        raise ValueError("INSPVAX contains a non-finite geodetic position")
    if not -90.0 <= lat <= 90.0:
        raise ValueError(f"INSPVAX latitude out of range: {lat}")
    if not -180.0 <= lon <= 180.0:
        raise ValueError(f"INSPVAX longitude out of range: {lon}")
    if not -1000.0 <= height <= 10000.0:
        raise ValueError(f"INSPVAX height out of expected range: {height}")
    return lat, lon, height


def geodetic_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    prime_vertical = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (prime_vertical + height_m) * cos_lat * math.cos(lon)
    y = (prime_vertical + height_m) * cos_lat * math.sin(lon)
    z = (prime_vertical * (1.0 - WGS84_E2) + height_m) * sin_lat
    return x, y, z


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    height_m: float,
    origin: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Convert geodetic coordinates to local ENU using an ECEF tangent plane."""
    lat0_deg, lon0_deg, height0_m = origin
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, height_m)
    x0, y0, z0 = geodetic_to_ecef(lat0_deg, lon0_deg, height0_m)
    dx, dy, dz = x - x0, y - y0, z - z0

    lat0 = math.radians(lat0_deg)
    lon0 = math.radians(lon0_deg)
    sin_lat0, cos_lat0 = math.sin(lat0), math.cos(lat0)
    sin_lon0, cos_lon0 = math.sin(lon0), math.cos(lon0)

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = (
        -sin_lat0 * cos_lon0 * dx
        - sin_lat0 * sin_lon0 * dy
        + cos_lat0 * dz
    )
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return east, north, up


def order_and_deduplicate(
    samples: Sequence[tuple[int, float, float, float]],
) -> tuple[list[tuple[int, float, float, float]], int]:
    """Sort by bag timestamp and keep the first record for duplicate timestamps."""
    ordered = sorted(samples, key=lambda sample: sample[0])
    unique: list[tuple[int, float, float, float]] = []
    duplicates = 0
    previous_timestamp: int | None = None
    for sample in ordered:
        timestamp = sample[0]
        if previous_timestamp == timestamp:
            duplicates += 1
            continue
        unique.append(sample)
        previous_timestamp = timestamp
    return unique, duplicates
