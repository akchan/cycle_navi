#!/usr/bin/env python
# coding: UTF-8

# Description
# ===========
#
# Library to get altitude using geographic information authority of Japan
# altitude tile system.
#
# References
# ==========
#
# - https://maps.gsi.go.jp/development/ichiran.html
#

import os
from pathlib import Path
import shutil
import tempfile
import time

import numpy as np
from PIL import Image
import requests


class ImageTileCache:
    def __init__(self,
                 request_interval_sec=1.0,
                 tile_size=256,  # should be powers of 2
                 cache_dir="./cache",
                 file_ext="png",
                 verbose=False):
        self.request_interval_sec = max([0.0, float(request_interval_sec)])
        self.last_request_time_stamp = 0.0
        self.tile_size = int(tile_size)
        self.cache_dir = str(cache_dir)
        self.file_ext = str(file_ext)
        self.verbose = verbose

    def clear_cache(self):
        if os.path.isdir(self.cache_dir):
            shutil.rmtree(self.cache_dir)

    def build_tile_path(self, zoom, x_tile, y_tile):
        return '{:d}/{:d}/{:d}.{}'.format(zoom, x_tile, y_tile, self.file_ext)

    def build_cache_file_path(self, zoom, x_tile, y_tile):
        tile_path = self.build_tile_path(zoom, x_tile, y_tile)
        file_path = os.path.join(self.cache_dir, tile_path)
        return file_path

    def fetch_file(self, zoom, x_tile, y_tile):
        file_path = self.fetch_file_path(zoom, x_tile, y_tile)

        if file_path is None:
            img = None
        else:
            img = Image.open(file_path)

        return img

    def fetch_file_path(self, zoom, x_tile, y_tile):
        tile_coords = (zoom, x_tile, y_tile)
        file_path = self.build_cache_file_path(*tile_coords)

        if not os.path.isfile(file_path):
            if self.verbose:
                print('Cache miss on file (zoom:{}, x_tile:{}, y_tile:{})'.format(
                    *tile_coords))

            self.wait_interval()

            if not self.request_image(*tile_coords):
                Path(file_path).touch()

        if os.path.isfile(file_path) and os.path.getsize(file_path) > 0:
            ret = file_path
        else:
            ret = None

        return ret

    def gen_url(self, zoom, x_tile, y_tile):
        raise NotImplementedError()

    def request_image(self, zoom: int, x_tile: float, y_tile: float) -> bool:
        ret = False

        zoom = int(zoom)
        x_tile = int(x_tile)
        y_tile = int(y_tile)

        val_max = 1 << zoom
        assert x_tile < val_max, "Given tile coordinate exceeded the limit."
        assert y_tile < val_max, "Given tile coordinate exceeded the limit."

        file_path = self.build_cache_file_path(zoom, x_tile, y_tile)

        url = self.gen_url(zoom, x_tile, y_tile)

        res = requests.get(url)
        if res.status_code == 200:
            # Add downloaded file to cache dir
            with tempfile.SpooledTemporaryFile(self.tile_size**2*3*1.5) as fp:
                fp.write(res.content)
                fp.seek(0)
                os.makedirs(os.path.dirname(file_path), exist_ok=True)
                Image.open(fp, formats=("PNG",)).convert(
                    "RGB").save(file_path)
            ret = True
        elif res.status_code == 404:
            if self.verbose:
                print("Tile not found: z={}, x={}, y={}".format(
                    zoom, x_tile, y_tile))
        else:
            raise AssertionError(
                'unexpected status code from server: {}'.format(res.status_code))

        return ret

    def wait_interval(self):
        t = time.time()
        if t - self.last_request_time_stamp < self.request_interval_sec:
            time.sleep(self.last_request_time_stamp +
                       self.request_interval_sec - t)
            self.last_request_time_stamp += self.request_interval_sec

    @classmethod
    def calc_lonlat2tile_coords(cls,
                                lon: float, lat: float,
                                zoom: int,
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

        x_tile = int(np.floor(x_raw * (1 << zoom)))
        y_tile = int(np.floor((0.5 - y_raw) * (1 << zoom)))

        return (zoom, x_tile, y_tile)

    @classmethod
    def calc_max_lat(cls):
        max_lat_rad = 2 * np.arctan(np.exp(np.pi)) - np.pi / 2
        max_lat = max_lat_rad / np.pi * 180
        return max_lat

    @classmethod
    def calc_idx_on_tile(cls, lon: float, lat: float, zoom: int, tile_size: int = 256):
        zoom2 = zoom + np.log2(tile_size)
        zoom2 = int(zoom2)
        _, x_tile2, y_tile2 = cls.calc_lonlat2tile_coords(lon, lat, zoom2)

        idx_x = x_tile2 % tile_size
        idx_y = y_tile2 % tile_size

        return idx_x, idx_y

    @classmethod
    def dist_2points(cls, lon1: float, lat1: float, lon2: float, lat2: float, r: float = 6371.0*1000):
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


class GsiStandardTile(ImageTileCache):
    def __init__(self,
                 request_interval_sec=1.0,
                 cache_dir="./gsi_standard",
                 verbose=False):
        super.__init__(request_interval_sec=request_interval_sec,
                       tile_size=256,
                       cache_dir=cache_dir,
                       file_ext="png",
                       verbose=verbose)

    def gen_url(self, zoom, x_tile, y_tile):
        url = "https://cyberjapandata.gsi.go.jp/xyz/std/{:d}/{:d}/{:d}.png".format(
            zoom, x_tile, y_tile)
        return url


class GsiAltitudeTile(ImageTileCache):
    def __init__(self,
                 request_interval_sec=1.0,
                 cache_dir="./gsi_altitude",
                 verbose=False):
        super.__init__(request_interval_sec=request_interval_sec,
                       tile_size=256,
                       cache_dir=cache_dir,
                       file_ext="png",
                       verbose=verbose)

    def gen_url(self, zoom, x_tile, y_tile):
        url = "https://cyberjapandata.gsi.go.jp/xyz/dem5a_png/{:d}/{:d}/{:d}.png".format(
            zoom, x_tile, y_tile)
        return url


def main():
    pass


if __name__ == "__main__":
    main()
