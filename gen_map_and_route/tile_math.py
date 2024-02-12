#!/usr/bin/env python
# coding: UTF-8


import numpy as np


def lonlat2idx(lon: float, lat: float, zoom: int,
               y_method: str = "default"):
    """
    Return
    ======

    tile_coods: based on the Mercator method

    Memo
    ====

    - [Mercator projection - Wikipedia](https://en.wikipedia.org/wiki/Mercator_projection#Mathematics)
    - [タイル座標確認ページ](https://maps.gsi.go.jp/development/tileCoordCheck.html#5/35.362/138.731)
    """
    zoom = int(zoom)

    # radius for circumference of 1
    r = 1.0 / (2 * np.pi)

    x_raw = 2 * np.pi * r * (lon + 180) / 360

    if y_method == "default":
        y_raw = r * np.log(np.tan(np.pi / 180 * (90 + lat) / 2))
    elif y_method == "sin":
        phi = lat / 180 * np.pi
        y_raw = r / 2.0 * np.log((1 + np.sin(phi)) / (1 - np.sin(phi)))
    elif y_method == "sin_cos":
        phi = lat / 180 * np.pi
        y_raw = r * np.log((1 + np.sin(phi)) / np.cos(phi))
    elif y_method == "cos_tan":
        phi = lat / 180 * np.pi
        y_raw = r * np.log(1.0 / np.cos(phi) + np.tan(phi))
    else:
        raise NotImplementedError("Invalid y_method:", y_method)

    idx_x = int(np.floor(x_raw * (1 << zoom)))
    idx_y = int(np.floor((0.5 - y_raw) * (1 << zoom)))

    return (zoom, idx_x, idx_y)


def max_y2lat():
    max_lat_rad = 2 * np.arctan(np.exp(np.pi)) - np.pi / 2
    max_lat = max_lat_rad / np.pi * 180
    return max_lat


def lonlat2idx_on_tile(lon: float, lat: float, zoom: int, tile_size: int = 256):
    zoom2 = zoom + np.log2(tile_size)
    zoom2 = int(zoom2)
    _, x_tile2, y_tile2 = lonlat2tile_coords(lon, lat, zoom2)

    idx_x = x_tile2 % tile_size
    idx_y = y_tile2 % tile_size

    return idx_x, idx_y


def dist_2points(lon1: float, lat1: float, lon2: float, lat2: float, r: float = 6371.0*1000):
    """
    Return
    ======

    dist: distance between two points in meters using Grat-Circular distance method
    """
    lon1_rad = np.deg2rad(lon1)
    lat1_rad = np.deg2rad(lat1)
    lon2_rad = np.deg2rad(lon2)
    lat2_rad = np.deg2rad(lat2)

    delta_theta = np.arccos(np.sin(lat1_rad) * np.sin(lat2_rad)
                            + np.cos(lat1_rad) * np.cos(lat2_rad) * np.cos(lon1_rad - lon2_rad))

    dist = r * delta_theta

    return dist
