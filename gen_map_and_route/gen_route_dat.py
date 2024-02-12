#!/usr/bin/env python
# coding: UTF-8

# Generate route dat for the M5stack cycle navigation system
#
# Created on
#

import datetime
import itertools
import json
import urllib.parse
import math
import os
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

from skimage import draw

import numpy as np
import requests
from PIL import Image
from tqdm import tqdm

from gsij_alt_tile import GsiStandardTile

from tile_math import lonlat2idx


TILE_SIZE = 256
REQUEST_INTERVAL = 1  # in sec

t_request_prev = time.time()  # for get_map_tile()


def gen_dir_with_timestamp(base_path):
    base_path = str(base_path).rstrip('/')

    while True:
        path_with_timestamp = datetime.datetime.now().strftime(
            f"{base_path}_%Y%m%d_%H%M%S")

        if not os.path.exists(path_with_timestamp):
            os.makedirs(path_with_timestamp)
            break

        time.sleep(1)

    return path_with_timestamp


def parse_route_coords_from_xml(path):
    xml_tree = ET.parse(path)
    file_type = xml_tree.getroot().tag[-3:]

    if file_type == 'gpx':
        coords = parse_route_coords_from_gpx(path)
    elif file_type == 'kml':
        coords = parse_route_coords_from_kml(path)
    else:
        raise NotImplementedError('invalid file type. {}'.format(file_type))

    return coords


def parse_route_coords_from_gpx(path):
    xml_tree = ET.parse(path)
    coords = []

    root = xml_tree.getroot()
    namespace = root.tag[:-3]
    xpath_str = './/{}trk'.format(namespace)

    def key_func(elm):
        return len(elm.findall('.//{}trkpt'.format(namespace)))

    # Take only the longest route
    elm_trk = sorted(root.findall(xpath_str), key=key_func)[-1]
    xpath_str = './/{}trkpt'.format(namespace)

    for elm in elm_trk.findall(xpath_str):
        lon = elm.get('lon')
        lat = elm.get('lat')

        if lon is None or lat is None:
            continue

        coords.append([float(lon), float(lat)])

    return coords


def parse_route_coords_from_kml(path):
    xml_tree = ET.parse(path)
    coords = []

    root = xml_tree.getroot()
    namespace = root.tag[:-3]

    # Take one whose length is the most longest
    xpath_str = './/{}Placemark/{}LineString/{}coordinates'.format(
        namespace, namespace, namespace)

    def key_func(elm):
        return len(elm.text.split('\n'))

    # Take all routes
    elm_coords = sorted(root.findall(xpath_str), key=key_func)

    # Convert from string to float
    for elm_coords_tmp in elm_coords:
        lines = elm_coords_tmp.text.split('\n')

        # Pass when it seems to be a point
        if len(lines) < 2:
            continue

        for line in lines:
            line = line.split(',')
            if len(line) < 2:
                continue
            lon, lat = line[:2]
            coords.append([float(lon), float(lat)])

    return coords


def parse_point_coords_from_xml(path):
    xml_tree = ET.parse(path)
    file_type = xml_tree.getroot().tag[-3:]

    if file_type == 'gpx':
        coords = []
    elif file_type == 'kml':
        coords = parse_point_coords_from_kml(path)
    else:
        raise NotImplementedError('invalid file type. {}'.format(file_type))

    return coords


def parse_point_coords_from_kml(path):
    xml_tree = ET.parse(path)
    coords = []

    root = xml_tree.getroot()
    namespace = root.tag[:-3]

    # Take one whose length is the most longest
    xpath_str = './/{}Placemark/{}Point/{}coordinates'.format(
        namespace, namespace, namespace)

    # Convert from string to float
    for elm_coords in root.findall(xpath_str):
        for line in elm_coords.text.split('\n'):
            line = line.split(',')
            if len(line) < 2:
                continue
            lon, lat = line[:2]
            coords.append([float(lon), float(lat)])

    return coords


class TileIndex:
    def __init__(self, idx_x, idx_y, tile_size=256):
        self.x = idx_x
        self.y = idx_y
        self.tile_size = tile_size

    def xy(self):
        return (self.x, self.y)

    def tile_x(self):
        return self.x // self.tile_size

    def tile_y(self):
        return self.y // self.tile_size

    def tile_xy(self):
        return (self.tile_x(), self.tile_y())

    def idx_x_on_tile(self):
        return self.x % self.tile_size

    def idx_y_on_tile(self):
        return self.y % self.tile_size

    def idx_on_tile(self, relative_from=None):
        if relative_from is None:
            ret = (self.idx_x_on_tile(), self.idx_y_on_tile())
        else:
            rel_x = self.x - relative_from.tile_x() * relative_from.tile_size
            rel_y = self.y - relative_from.tile_y() * relative_from.tile_size
            ret = (rel_x, rel_y)
        return ret

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        return not self.__eq__(other)

    def is_neighbor(self, other):
        condition_a = np.abs(self.tile_x() - other.tile_x()
                             ) == 1 and self.tile_y() == other.tile_y()
        condition_b = self.tile_x() == other.tile_x(
        ) and np.abs(self.tile_y() - other.tile_y())
        return condition_a or condition_b

    def is_same_tile(self, other):
        return self.tile_x() == other.tile_x() and self.tile_y() == other.tile_y()


def lonlat_list2idx_list(lonlat_list: list[list[float, float]],
                         zoom: int,
                         tile_size=256) -> list[TileIndex]:
    idx_list = []

    for lon, lat in lonlat_list:
        zoom, idx_x, idx_y = lonlat2idx(lon, lat, zoom)
        idx_list.append(TileIndex(idx_x, idx_y, tile_size))

    return idx_list


def remove_idx_duplication(idx_list: list[list[int, int]]):
    if len(idx_list) <= 1:
        return idx_list

    tile_idx_prev = idx_list[0]
    ret = [tile_idx_prev]

    for tile_idx in idx_list[1:]:
        if tile_idx != tile_idx_prev:
            ret.append(tile_idx)
        tile_idx_prev = tile_idx

    return ret


def create_relay_points(tile_idx_list: list[TileIndex]):
    if len(tile_idx_list) <= 1:
        return tile_idx_list

    ret = []

    for point1, point2 in zip(tile_idx_list[:-1], tile_idx_list[1:]):
        relay_points = create_relay_between(point1, point2)

        ret.extend(relay_points[:-1])

    ret.append(tile_idx_list[-1])

    return ret


def create_relay_between(point1: TileIndex, point2: TileIndex) -> list[TileIndex]:
    point1_x, point1_y = point1.xy()
    point2_x, point2_y = point2.xy()
    tile_size = point1.tile_size

    xx, yy = draw.line(point1_x, point1_y, point2_x, point2_y)

    xx_tile = xx // tile_size
    yy_tile = yy // tile_size

    idx_xx_diff = np.where(xx_tile[:-1] != xx_tile[1:])[0]
    idx_yy_diff = np.where(yy_tile[:-1] != yy_tile[1:])[0]

    idx_relay = [
        [0, len(xx)-1],
        idx_xx_diff,
        idx_xx_diff + 1,
        idx_yy_diff,
        idx_yy_diff + 1,
    ]
    idx_relay = np.concatenate(idx_relay)
    idx_relay = np.unique(idx_relay)
    idx_relay = np.sort(idx_relay)

    coords = []
    for x, y in np.array([xx[idx_relay], yy[idx_relay]]).transpose():
        coords.append(TileIndex(x, y, tile_size))

    return coords


def build_route_list_to_write(zoom, idx_list: list[TileIndex], tile_size=256):
    tile_idx_list = []

    tile_idx_prev = idx_list[0]
    tile_idx_list.append([
        zoom,
        tile_idx_prev.tile_x(),
        tile_idx_prev.tile_y(),
        tile_idx_prev.idx_x_on_tile(),
        tile_idx_prev.idx_y_on_tile(),
    ])

    for tile_idx in idx_list[1:]:
        if not tile_idx.is_same_tile(tile_idx_prev):
            # coordinates 1
            # create coordinates outside the previous tile
            rel_x, rel_y = tile_idx.idx_on_tile(relative_from=tile_idx_prev)

            tile_idx_list.append([
                zoom,
                tile_idx_prev.tile_x(),
                tile_idx_prev.tile_y(),
                rel_x,
                rel_y,
            ])

            # coordinates 2
            # create coordinates outside the this tile
            rel_x, rel_y = tile_idx_prev.idx_on_tile(relative_from=tile_idx)

            tile_idx_list.append([
                zoom,
                tile_idx.tile_x(),
                tile_idx.tile_y(),
                rel_x,
                rel_y,
            ])

        tile_idx_list.append([
            zoom,
            tile_idx.tile_x(),
            tile_idx.tile_y(),
            tile_idx.idx_x_on_tile(),
            tile_idx.idx_y_on_tile(),
        ])

        tile_idx_prev = tile_idx

    return tile_idx_list


def write_route_dat(route_dat_dir_path, tile_idx_list):
    for zoom, x_tile, y_tile, x_idx, y_idx in tile_idx_list:
        file_path = os.path.join(
            route_dat_dir_path, '{}/{}/{}.dat'.format(zoom, x_tile, y_tile))

        dir_path = os.path.dirname(file_path)
        os.makedirs(dir_path, exist_ok=True)

        with open(file_path, 'ab') as fp:
            # i: signed integer (4 bytes = 32 bits)
            # <: little endian
            buf = struct.Struct('<ii').pack(x_idx, y_idx)
            fp.write(buf)


def build_point_list_to_write(zoom, idx_list: list[TileIndex]):
    tile_idx_list = []

    for tile_idx in idx_list:
        tile_size = tile_idx.tile_size

        for i in range(-1, 2):
            for j in range(-1, 2):
                x_tmp = (tile_idx.tile_x() + i) * tile_size
                y_tmp = (tile_idx.tile_y() + j) * tile_size
                tile_idx_tmp = TileIndex(x_tmp, y_tmp, tile_idx.tile_size)

                rel_x, rel_y = tile_idx.idx_on_tile(relative_from=tile_idx_tmp)

                tile_idx_list.append([
                    zoom,
                    tile_idx_tmp.tile_x(),
                    tile_idx_tmp.tile_y(),
                    rel_x,
                    rel_y,
                ])

    return tile_idx_list


def write_point_dat(point_dat_dir_path, point_tile_idx):
    for z, x, y, x_idx, y_idx in point_tile_idx:
        file_path = os.path.join(
            point_dat_dir_path, '{}/{}/{}.dat'.format(z, x, y))

        dir_path = os.path.dirname(file_path)
        os.makedirs(dir_path, exist_ok=True)

        with open(file_path, 'ab') as fp:
            # i: signed integer (4 bytes = 32 bits)
            # <: little endian
            buf = struct.Struct('<ii').pack(x_idx, y_idx)
            fp.write(buf)


def main(route_file_path,
         route_dat_dir_path='./route_dat',
         point_dat_dir_path='./point_dat',
         init_point_path="./initPoint",
         use_sound=True,
         target_zoom_level=(8, 12, 14),
         tile_size=256,
         verbose=True):
    base_dir = gen_dir_with_timestamp("route_dat")
    route_dat_dir_path = os.path.join(base_dir, route_dat_dir_path)
    point_dat_dir_path = os.path.join(base_dir, point_dat_dir_path)
    init_point_path = os.path.join(base_dir, init_point_path)
    use_sound_path = os.path.join(base_dir, "useSound")
    
    assert hasattr(target_zoom_level,
                   '__iter__'), 'target_zoom_level should be iterable.'
    target_zoom_level = sorted(map(lambda x: int(x), target_zoom_level))
    zoom_max = max(target_zoom_level)

    # Generate route_dat
    if verbose:
        print("Generating route_dat...")

    route_lonlat_list = parse_route_coords_from_xml(route_file_path)

    for zoom in target_zoom_level:
        tile_idx_list = lonlat_list2idx_list(
            route_lonlat_list, zoom, tile_size)
        tile_idx_list = remove_idx_duplication(tile_idx_list)
        tile_idx_list = create_relay_points(tile_idx_list)

        list_to_write = build_route_list_to_write(
            zoom, tile_idx_list, tile_size)

        write_route_dat(route_dat_dir_path, list_to_write)

        # Genrate initPoint file
        if zoom == zoom_max:
            init_x = tile_idx_list[0].x
            init_y = tile_idx_list[0].y

            if verbose:
                print(f"Initial view point: z={zoom}, x={init_x}, y={init_y}")

            with open(init_point_path, 'wb') as fp:
                # i: signed integer (4 bytes = 32 bits)
                # <: little endian
                buf = struct.Struct('<iii').pack(zoom_max, init_x, init_y)
                fp.write(buf)

    # Generate point_dat
    if verbose:
        print("Generating point_dat...")

    point_lonlat_list = parse_point_coords_from_xml(route_file_path)

    for zoom in target_zoom_level:
        tile_idx_list = lonlat_list2idx_list(
            point_lonlat_list, zoom, tile_size)

        list_to_write = build_point_list_to_write(zoom, tile_idx_list)

        write_point_dat(point_dat_dir_path, list_to_write)

    # Generate use_sound file
    if use_sound:
        open(use_sound_path, 'w').close()


if __name__ == '__main__':
    route_file_path = sys.argv[1]

    main(route_file_path)
