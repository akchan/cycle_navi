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

import numpy as np
import requests
from PIL import Image
from tqdm import tqdm

from gsij_alt_tile import GsijAltTile


TILE_SIZE = 256
REQUEST_INTERVAL = 1  # in sec

t_request_prev = time.time()  # for get_map_tile()


def gen_output_dir(output_dir, basename):
    while True:
        output_basename = datetime.datetime.now().strftime("{}_%Y%m%d_%H%M%S".format(basename))
        output_dir_path = os.path.join(output_dir, output_basename)

        if not os.path.isdir(output_dir_path):
            os.makedirs(output_dir_path)
            break

        time.sleep(1)

    return output_dir_path


def parse_route_coords_from_xml(xml_tree):
    coords = []

    file_type = xml_tree.getroot().tag[-3:]

    if file_type == 'gpx':
        coords = parse_route_coords_from_gpx(xml_tree)
    elif file_type == 'kml':
        coords = parse_route_coords_from_kml(xml_tree)
    else:
        raise NotImplementedError('invalid file type. {}'.format(file_type))

    return coords


def parse_route_coords_from_gpx(xml_tree):
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


def parse_route_coords_from_kml(xml_tree):
    coords = []

    root = xml_tree.getroot()
    namespace = root.tag[:-3]

    # Take one whose length is the most longest
    xpath_str = './/{}Placemark/{}LineString/{}coordinates'.format(namespace, namespace, namespace)

    def key_func(elm):
        return len(elm.text.split('\n'))

    # Take only the longest route
    elm_coords = sorted(root.findall(xpath_str), key=key_func)[-1]

    # Convert from string to float
    for line in elm_coords.text.split('\n'):
        line = line.split(',')
        if len(line) < 2:
            continue
        lon, lat = line[:2]
        coords.append([float(lon), float(lat)])

    return coords


def parse_point_coords_from_xml(xml_tree):
    coords = []

    file_type = xml_tree.getroot().tag[-3:]

    if file_type == 'gpx':
        coords = []
    elif file_type == 'kml':
        coords = parse_point_coords_from_kml(xml_tree)
    else:
        raise NotImplementedError('invalid file type. {}'.format(file_type))

    return coords


def parse_point_coords_from_kml(xml_tree):
    coords = []

    root = xml_tree.getroot()
    namespace = root.tag[:-3]

    # Take one whose length is the most longest
    xpath_str = './/{}Placemark/{}Point/{}coordinates'.format(namespace, namespace, namespace)

    # Convert from string to float
    for elm_coords in root.findall(xpath_str):
        for line in elm_coords.text.split('\n'):
            line = line.split(',')
            if len(line) < 2:
                continue
            lon, lat = line[:2]
            coords.append([float(lon), float(lat)])

    return coords


def conv_route_coords_to_tile_idx_list(zoom, coords, tile_size=256, verbose=False):
    tile_idx_list = []
    raw_tile_idx = []

    for lon, lat in coords:
        _, tile_x, tile_y = GsijAltTile.calc_coords2tile_coords(zoom, lon, lat)
        idx_x, idx_y = GsijAltTile.calc_tile_idx(zoom, lon, lat, tile_size)

        raw_tile_idx.append([tile_x, tile_y, idx_x, idx_y])

    if verbose:
        print(raw_tile_idx[0])

    tile_idx_list.append([zoom, *raw_tile_idx[0]])

    for prev, curr in zip(raw_tile_idx[:-1], raw_tile_idx[1:]):
        if prev == curr:
            continue

        tile_x_prev, tile_y_prev, idx_x_prev, idx_y_prev = prev
        tile_x_curr, tile_y_curr, idx_x_curr, idx_y_curr = curr

        if [tile_x_prev, tile_y_prev] != [tile_x_curr, tile_y_curr]:
            idx_tmp = [
                zoom, tile_x_prev, tile_y_prev,
                idx_x_curr + tile_size * (tile_x_curr - tile_x_prev),
                idx_y_curr + tile_size * (tile_y_curr - tile_y_prev)
            ]

            if verbose:
                print(idx_tmp)

            tile_idx_list.append(idx_tmp)

            idx_tmp = [
                zoom, tile_x_curr, tile_y_curr,
                idx_x_prev + tile_size * (tile_x_prev - tile_x_curr),
                idx_y_prev + tile_size * (tile_y_prev - tile_y_curr)
            ]

            if verbose:
                print(idx_tmp)

            tile_idx_list.append(idx_tmp)

        if verbose:
            print([zoom, *curr])

        tile_idx_list.append([zoom, *curr])

    if verbose:
        print("len(tile_idx_list):", len(tile_idx_list))

    return tile_idx_list


def write_route_tile_idx_to_route_dat(route_dat_dir_path, tile_idx_list):
    for zoom, x_tile, y_tile, x_idx, y_idx in tile_idx_list:
        file_path = os.path.join(route_dat_dir_path, '{}/{}/{}.dat'.format(zoom, x_tile, y_tile))
        dir_path = os.path.dirname(file_path)

        if not os.path.isdir(dir_path):
            os.makedirs(dir_path)

        with open(file_path, 'ab') as fp:
            # i: signed integer (4 bytes = 32 bits)
            # <: little endian
            buf = struct.Struct('<ii').pack(x_idx, y_idx)
            fp.write(buf)


def conv_point_coords_to_tile_idx_list(zoom, point_coords, tile_size, point_dat_tile_margin=20):
    tile_idx_list = []

    for lon, lat in point_coords:

        _, x_tile, y_tile = GsijAltTile.calc_coords2tile_coords(zoom, lon, lat)
        x_idx, y_idx = GsijAltTile.calc_tile_idx(zoom, lon, lat, tile_size)

        tile_idx_list.append([zoom, x_tile, y_tile, x_idx, y_idx])

        # Add tile idx to neighbor tile to avoid chipping point circle on the screen
        if x_idx < point_dat_tile_margin:
            tile_idx_list.append([zoom, x_tile-1, y_tile, x_idx+tile_size, y_idx])
        if tile_size - point_dat_tile_margin < x_idx:
            tile_idx_list.append([zoom, x_tile+1, y_tile, x_idx-tile_size, y_idx])
        if y_idx < point_dat_tile_margin:
            tile_idx_list.append([zoom, x_tile, y_tile-1, x_idx, y_idx+tile_size])
        if tile_size - point_dat_tile_margin < y_idx:
            tile_idx_list.append([zoom, x_tile, y_tile+1, x_idx, y_idx-tile_size])

    return tile_idx_list


def write_point_tile_idx_to_point_dat(point_dat_dir_path, point_tile_idx):
    for z, x, y, x_idx, y_idx in point_tile_idx:
        file_path = os.path.join(point_dat_dir_path, '{}/{}/{}.dat'.format(z, x, y))
        dir_path = os.path.dirname(file_path)

        if not os.path.isdir(dir_path):
            os.makedirs(dir_path)

        with open(file_path, 'ab') as fp:
            # i: signed integer (4 bytes = 32 bits)
            # <: little endian
            buf = struct.Struct('<ii').pack(x_idx, y_idx)
            fp.write(buf)


def conv_tile_coords(from_z, from_tile_x, from_tile_y, to_z):
    to_tile_x = math.floor(from_tile_x * math.pow(2, to_z - from_z))
    to_tile_y = math.floor(from_tile_y * math.pow(2, to_z - from_z))

    return (to_z, to_tile_x, to_tile_y)


def list_map_tile_to_collect(target_zoom_level,
                             route_tile_idx_list,
                             point_tile_idx_list,
                             r_th_neighbor_map_collection=1.5,
                             collection_area_zoom=8,  # same value to the value in japan_tile_list_path json fjile
                             japan_tile_list_path="japan_tile_list.json",):
    map_tile_having_something = set()

    for z, tile_x, tile_y, idx_x, idx_y in route_tile_idx_list:
        _, map_tile_x, map_tile_y = conv_tile_coords(z, tile_x, tile_y, collection_area_zoom)
        map_tile_having_something.add((collection_area_zoom, map_tile_x, map_tile_y))

    for z, tile_x, tile_y, idx_x, idx_y in point_tile_idx_list:
        _, map_tile_x, map_tile_y = conv_tile_coords(z, tile_x, tile_y, collection_area_zoom)
        map_tile_having_something.add((collection_area_zoom, map_tile_x, map_tile_y))

    map_tile_area_to_collect = set()
    with open(japan_tile_list_path, "r") as f:
        japan_tile_list = json.load(f)

    # Pickup neighbor tiles
    for japan_tile, map_tile_area in itertools.product(japan_tile_list, map_tile_having_something):
        z, j_tile_x, j_tile_y = japan_tile
        _, tile_x, tile_y = map_tile_area

        if (j_tile_x - tile_x) ** 2 + (j_tile_y - tile_y) ** 2 < r_th_neighbor_map_collection ** 2:
            map_tile_area_to_collect.add((z, j_tile_x, j_tile_y))

    map_tile_to_collect = []
    # Collect map tile of given zoom level within the area
    for target_zoom in target_zoom_level:
        for z, tile_x, tile_y in map_tile_area_to_collect:
            assert z <= target_zoom, "target zoom level should be equal or less than 8."

            ul_x = (1 << (target_zoom - z)) * tile_x
            ul_y = (1 << (target_zoom - z)) * tile_y
            for i_x in range((1 << (target_zoom - z))):
                for i_y in range((1 << (target_zoom - z))):
                    x_tile_tmp = ul_x + i_x
                    y_tile_tmp = ul_y + i_y
                    map_tile_to_collect.append([target_zoom, x_tile_tmp, y_tile_tmp])

    return map_tile_to_collect


def get_map_tile(zoom, tile_x, tile_y,
                 gsij_obj=None,
                 base_url="https://cyberjapandata.gsi.go.jp/xyz/std/",
                 request_interval=REQUEST_INTERVAL,
                 cache_dir="gsij_map_tile_cache",
                 verbose=False):
    global t_request_prev

    cache_path = os.path.join(cache_dir, "{}/{}/{}.jpg".format(zoom, tile_x, tile_y))
    ret = cache_path

    if not os.path.isfile(cache_path):
        # cache miss
        if verbose:
            print("({},{},{}) -> No cache. Requesting to gsi server.".format(zoom, tile_x, tile_y))

        t_now = time.time()
        if t_request_prev + request_interval > t_now:
            time.sleep(t_request_prev + request_interval - t_now)
        t_request_prev = t_now

        url = urllib.parse.urljoin(base_url, "{}/{}/{}.png".format(zoom, tile_x, tile_y))

        res = requests.get(url)
        if res.status_code == 200:
            # Add downloaded file to cache dir
            with tempfile.SpooledTemporaryFile(TILE_SIZE**2*3*1.5) as fp:
                fp.write(res.content)
                fp.seek(0)
                os.makedirs(cache_path, exist_ok=True)
                Image.open(fp, formats=("PNG",)).convert("RGB").save(cache_path)
        elif res.status_code == 404:
            Path(cache_path).touch()
        else:
            raise AssertionError('unexpected status code from server: {}'.format(res.status_code))

    elif os.path.getsize(cache_path) == 0:
        # cached 404 not found case
        ret = None
        if verbose:
            print("({},{},{}) -> No tile was found.".format(zoom, tile_x, tile_y))
    else:
        # cache hit
        if verbose:
            print("({},{},{}) -> cache hit".format(zoom, tile_x, tile_y))

    return ret


def collect_map_tile(map_dir_path, map_tile_to_collect, jpg_quality=75, verbose=False):
    gsij_obj = GsijAltTile(cache_dir="gsij_map_tile_cache")

    if verbose:
        map_tile_to_collect = tqdm(map_tile_to_collect)

    for zoom, tile_x, tile_y in map_tile_to_collect:
        file_path = os.path.join(map_dir_path, "{}/{}/{}.jpg".format(zoom, tile_x, tile_y))

        jpg_path = get_map_tile(zoom, tile_x, tile_y, gsij_obj)

        if jpg_path is None:
            continue
        
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        shutil.copy(jpg_path, file_path)


def main(route_file_path,
         route_dat_dir_name='route_dat',
         point_dat_dir_name='point_dat',
         point_dat_tile_margin=20,
         map_dir_name="map",
         r_th_neighbor_map_collection=1.5,
         target_zoom_level=(12, 14),
         verbose=True):
    assert hasattr(target_zoom_level, '__iter__'), 'target_zoom_level should be iterable.'

    output_dir_path = gen_output_dir("./", "map_route")

    # Generate route_dat
    if verbose:
        print("Generating route_dat...")
    route_dat_dir_path = os.path.join(output_dir_path, route_dat_dir_name)
    tree = ET.parse(route_file_path)
    route_coords = parse_route_coords_from_xml(tree)

    for zoom in target_zoom_level:
        route_tile_idx_list = conv_route_coords_to_tile_idx_list(zoom, route_coords, TILE_SIZE)
        write_route_tile_idx_to_route_dat(route_dat_dir_path, route_tile_idx_list)

    # Generate point_dat
    if verbose:
        print("Generating point_dat...")
    point_dat_dir_path = os.path.join(output_dir_path, point_dat_dir_name)
    point_coords = parse_point_coords_from_xml(tree)

    for zoom in target_zoom_level:
        point_tile_idx_list = conv_point_coords_to_tile_idx_list(zoom, point_coords, TILE_SIZE, point_dat_tile_margin)
        write_point_tile_idx_to_point_dat(point_dat_dir_path, point_tile_idx_list)

    # Collect map tile
    if verbose:
        print("Collecting map tile...")
    map_tile_to_collect = list_map_tile_to_collect(
        target_zoom_level,
        route_tile_idx_list,
        point_tile_idx_list,
        r_th_neighbor_map_collection=r_th_neighbor_map_collection
    )
    map_dir_path = os.path.join(output_dir_path, map_dir_name)
    collect_map_tile(map_dir_path, map_tile_to_collect, verbose=verbose)

    print('Copy directories inside the directory below to the root direcotry of SD card.')
    print('  {}'.format(output_dir_path))


if __name__ == '__main__':
    route_file_path = sys.argv[1]

    main(route_file_path)
