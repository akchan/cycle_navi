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

from collections import OrderedDict
import os
import shutil
import tempfile
import time

import numpy as np
from PIL import Image
import requests


class Singleton(object):
    def __new__(cls, *args, **kargs):
        if not hasattr(cls, "_instance"):
            cls._instance = super().__new__(cls)
        return cls._instance


class LRU(OrderedDict):
    def __init__(self, maxsize=128, /, *args, **kwds):
        self.maxsize = maxsize
        super().__init__(*args, **kwds)

    def __getitem__(self, key):
        value = super().__getitem__(key)
        self.move_to_end(key)
        return value

    def __setitem__(self, key, value):
        if key in self:
            self.move_to_end(key)
        super().__setitem__(key, value)
        if len(self) > self.maxsize:
            oldest = next(iter(self))
            del self[oldest]


class GsijAltTile(Singleton):
    def __init__(self, zoom=None,
                 png_request_interval_sec=None,
                 cache_method=None,
                 cache_memory_n=None,
                 cache_dir=None,
                 verbose=None):
        '''
        Parameters
        ==========

        cache_method: 'memory_only', 'memory_and_file'(default)
        '''
        self.verbose = verbose

        # Default values
        if zoom is None:
            zoom = 15  # maximum zoom level for DEM5A data

        if png_request_interval_sec is None:
            png_request_interval_sec = 1.0

        if cache_method is None:
            cache_method = 'memory_and_file'

        if cache_memory_n is None:
            cache_memory_n = 5 * 200  # approximately 200MB
        
        if verbose is None:
            verbose = False

        self.zoom = int(zoom)
        self.png_request_interval_sec = float(png_request_interval_sec)
        self.cache_method = cache_method
        self.cache_memory_n = int(cache_memory_n)
        self.last_png_request_time_stamp = 0.0

        if cache_dir is None:
            self.cache_dir = os.path.join(os.getcwd(), 'gsij_alt_tile_cache')
        else:
            self.cache_dir = cache_dir

        # self.tile_size should be powers of 2 for this implementatin
        self.tile_size = 256

        if not hasattr(self, "tile_cache"):
            self.clear_tile_memory_cache()

        assert self.cache_method in ('memory_only', 'memory_and_file')

    def clear_tile_memory_cache(self):
        self.tile_cache = LRU(self.cache_memory_n)
        if self.verbose:
            print('memory cache has been cleared.')
    
    def clear_tile_file_cache(self):
        if os.path.isdir(self.cache_dir):
            shutil.rmtree(self.cache_dir)
            if self.verbose:
                print('cache dir was removed.', self.cache_dir)

    def fetch_memory_cache(self, zoom, x_tile, y_tile):
        tile_coords = (zoom, x_tile, y_tile)

        if tile_coords in self.tile_cache:
            ret = self.tile_cache[tile_coords]
            if self.verbose:
                print('Cache hit on memory (zoom:{}, x_tile:{}, y_tile:{})'.format(*tile_coords))
        else:
            ret = None
            if self.verbose:
                print('Cache miss on memory (zoom:{}, x_tile:{}, y_tile:{})'.format(*tile_coords))

        return ret

    def add_memory_cache(self, zoom, x_tile, y_tile, tile):
        tile_coords = (zoom, x_tile, y_tile)
        self.tile_cache[tile_coords] = tile

    def build_cache_file_path(self, zoom, x_tile, y_tile):
        max_digit = len(str(1 << zoom))
        file_name = 'z{:02d}_x{:0{:d}d}_y{:0{:d}d}.npy'.format(zoom, x_tile, max_digit, y_tile, max_digit)
        file_path = os.path.join(self.cache_dir, file_name)

        return file_path

    def fetch_file_cache(self, zoom, x_tile, y_tile):
        tile_coords = (zoom, x_tile, y_tile)
        file_path = self.build_cache_file_path(*tile_coords)
        if os.path.isfile(file_path):
            ret = np.load(file_path)
            if self.verbose:
                print('Cache hit on file (zoom:{}, x_tile:{}, y_tile:{})'.format(*tile_coords))
        else:
            ret = None
            if self.verbose:
                print('Cache miss on file (zoom:{}, x_tile:{}, y_tile:{})'.format(*tile_coords))

        return ret

    def add_file_cache(self, zoom, x_tile, y_tile, tile):
        if not os.path.isdir(self.cache_dir):
            os.makedirs(self.cache_dir)

        file_path = self.build_cache_file_path(zoom, x_tile, y_tile)

        if not os.path.isfile(file_path):
            np.save(file_path, tile)
            if self.verbose:
                print('tile was saved to', file_path)

    def get_alt(self, lon, lat):
        lon = float(lon)
        lat = float(lat)

        tile_coords = self.calc_coords2tile_coords(self.zoom, lon, lat)

        if self.verbose:
            print('zoom:{}, lon:{}, lat:{}'.format(self.zoom, lon, lat))
            print('-> tile coordinates (zoom:{}, x_tile:{}, y_tile:{})'.format(*tile_coords))

        tile = self.fetch_memory_cache(*tile_coords)

        if self.cache_method == 'memory_and_file' and tile is None:
            tile = self.fetch_file_cache(*tile_coords)

        if tile is None:
            tile_png_url = self.gen_url(*tile_coords)
            tile = self.get_gia_alt_tile_with_png(tile_png_url)

        self.add_memory_cache(*tile_coords, tile)
        if self.cache_method == 'memory_and_file':
            self.add_file_cache(*tile_coords, tile)

        idx_x, idx_y = self.calc_tile_idx(self.zoom, lon, lat, self.tile_size)

        alt = tile[idx_y, idx_x]

        return alt

    @classmethod
    def calc_coords2tile_coords(cls, zoom, lon, lat, y_method="default"):
        """
        Return
        ======

        tile_coods: based on the Mercator method

        Memo
        ====

        - [Mercator projection - Wikipedia](https://en.wikipedia.org/wiki/Mercator_projection#Mathematics)
        - [タイル座標確認ページ](https://maps.gsi.go.jp/development/tileCoordCheck.html#5/35.362/138.731)
        """
        if not type(zoom) == int:
            print('[Alert] given zoom value ({}) is not integer but {}. It is converted to integer.'.format(zoom, type(zoom)))
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
    def calc_tile_coords2corner_coords(cls, zoom, x_tile, y_tile):
        left_upper_lon = x_tile / (1 << zoom) * 360 - 180

        r = 1.0 / (2 * np.pi)
        y = 0.5 - y_tile / (1 << zoom)
        left_upper_lat_rad = 2 * np.arctan(np.exp(y / r)) - np.pi / 2
        left_upper_lat = left_upper_lat_rad / np.pi * 180

        return (left_upper_lon, left_upper_lat)

    @classmethod
    def calc_max_lon(cls):
        max_lat_rad = 2 * np.arctan(np.exp(np.pi)) - np.pi / 2
        max_lat = max_lat_rad / np.pi * 180
        return max_lat

    @classmethod
    def calc_tile_idx(cls, zoom, lon, lat, tile_size):
        zoom, x_tile, y_tile = cls.calc_coords2tile_coords(zoom, lon, lat)

        zoom2 = zoom + np.log2(tile_size)
        zoom2 = int(zoom2)
        _, x_tile2, y_tile2 = cls.calc_coords2tile_coords(zoom2, lon, lat)

        idx_x = x_tile2 - x_tile * tile_size
        idx_y = y_tile2 - y_tile * tile_size

        return idx_x, idx_y

    @classmethod
    def gen_url(cls, zoom, x_tile, y_tile):
        zoom = int(zoom)
        x_tile = int(x_tile)
        y_tile = int(y_tile)

        val_max = 1 << zoom
        assert x_tile < val_max, "Given tile coordinate exceeded the limit."
        assert y_tile < val_max, "Given tile coordinate exceeded the limit."

        url = (
            "https://cyberjapandata.gsi.go.jp/xyz/dem5a_png/{:d}/{:d}/{:d}.png".format(
                zoom, x_tile, y_tile
            )
        )
        return url

    def get_gia_alt_tile_with_png(self, url, alt_res_m=0.01, request_interval=True):
        res = requests.get(url)
        img = None

        if request_interval:
            t_delay = time.time() - self.last_png_request_time_stamp
            t_delay = np.clip(t_delay, 0.0, self.png_request_interval_sec)
            t_delay = float(t_delay)
            time.sleep(t_delay)

        with tempfile.SpooledTemporaryFile(256**2*3*1.5) as fp:
            fp.write(res.content)
            fp.seek(0)
            img = np.array(Image.open(fp, formats=("PNG",)), dtype=np.int32)

        alt_mat_raw = img[:, :, 0] * 2 ** 16 + img[:, :, 1] * 2 ** 8 + img[:, :, 2]
        alt_mat = np.zeros(alt_mat_raw.shape[0:2], dtype=np.float32)
        val_th = 1 << 23

        mask_tmp = alt_mat_raw < val_th
        alt_mat[mask_tmp] = alt_mat_raw[mask_tmp] * alt_res_m

        mask_tmp = alt_mat_raw > val_th
        alt_mat[mask_tmp] = (alt_mat_raw[mask_tmp] - (1 << 24)) * alt_res_m

        return alt_mat

    @classmethod
    def dist_2points(cls, lon1, lat1, lon2, lat2, r=6371*1000):
        """
        Return
        ======

        dist: distance between two points in meters using Grat-Circular distance method
        """
        lon1 = np.deg2rad(lon1)
        lat1 = np.deg2rad(lat1)
        lon2 = np.deg2rad(lon2)
        lat2 = np.deg2rad(lat2)

        delta_theta = np.arccos(np.sin(lat1) * np.sin(lat2) + np.cos(lat1) * np.cos(lat2) * np.cos(lon1 - lon2))

        dist = r * delta_theta

        return dist


def main():
    pass


if __name__ == "__main__":
    main()
