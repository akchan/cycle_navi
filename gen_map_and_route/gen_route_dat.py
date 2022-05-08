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


def is_neighbor_tile(l_tile_x, l_tile_y, r_tile_x, r_tile_y):
    if (l_tile_x == r_tile_x and (np.abs(l_tile_y - r_tile_y) == 1)) \
       or (l_tile_y == r_tile_y and (np.abs(l_tile_x - r_tile_x) == 1)):
        return True

    return False


def is_same_tile(l_tile_x, l_tile_y, r_tile_x, r_tile_y):
    if l_tile_x == r_tile_x and l_tile_y == r_tile_y:
        return True

    return False


def force_neighbor_tile(tile_idx_list,
                        method='linear'):
    '''
    tile_idx_list: list of [tile_x, tile_y, idx_x, idx_y]
    method: 'linear' or 'bsearch'

    ToDL
    ====

    - Need refactoring for non recursive algorithm
    '''
    # Check arguments
    valid_methods = ['linear', 'bsearch']
    assert method in valid_methods, 'Invalid method {} was given. Use one of these. {}'.format(method, valid_methods)
    
    # Prepare variables
    ret = []
    ret.append(tile_idx_list[0])

    for i in range(len(tile_idx_list) - 1):
        l_tile_x, l_tile_y, l_idx_x, l_idx_y = tile_idx_list[i]
        r_tile_x, r_tile_y, r_idx_x, r_idx_y = tile_idx_list[i+1]

        if 'linear' == method:
            tile_idx_tmp = force_neighbor_tile_linear(l_tile_x, l_tile_y, l_idx_x, l_idx_y,
                                                      r_tile_x, r_tile_y, r_idx_x, r_idx_y)
        elif 'bsearch' == method:
            tile_idx_tmp = force_neighbor_tile_bsearch(l_tile_x, l_tile_y, l_idx_x, l_idx_y,
                                                       r_tile_x, r_tile_y, r_idx_x, r_idx_y)
        
        ret.extend(tile_idx_tmp[1:])
    
    return ret


def force_neighbor_tile_linear(l_tile_x, l_tile_y, l_idx_x, l_idx_y,
                               r_tile_x, r_tile_y, r_idx_x, r_idx_y):
    '''
    Make given two points being neighbor adding intermediate points.
    The points are determined by linear search method.

    Return
    ======

    - [
        [l_tile_x, l_tile_y, l_idx_x, l_idx_y],
        # some or no points
        [r_tile_x, r_tile_y, r_idx_x, r_idx_y]
        ]
    '''
    ret = []
    swap_flag = False  # default is False

    point_left = [l_tile_x, l_tile_y, l_idx_x, l_idx_y]
    point_right = [r_tile_x, r_tile_y, r_idx_x, r_idx_y]

    if is_same_tile(l_tile_x, l_tile_y, r_tile_x, r_tile_y) \
       or is_neighbor_tile(l_tile_x, l_tile_y, r_tile_x, r_tile_y):
        # Nothing to do for given two points
        # This case includes that given two points are same.
        ret.append(point_left)
        ret.append(point_right)
        return ret

    # Convert coordinates from tile and index to index only
    l_coords_idx_x = l_tile_x * TILE_SIZE + l_idx_x
    l_coords_idx_y = l_tile_y * TILE_SIZE + l_idx_y
    r_coords_idx_x = r_tile_x * TILE_SIZE + r_idx_x
    r_coords_idx_y = r_tile_y * TILE_SIZE + r_idx_y

    diff_x = r_coords_idx_x - l_coords_idx_x
    diff_y = r_coords_idx_y - l_coords_idx_y

    if np.abs(diff_x) < np.abs(diff_y):
        swap_flag = True
        l_coords_idx_x, l_coords_idx_y = l_coords_idx_y, l_coords_idx_x
        r_coords_idx_x, r_coords_idx_y = r_coords_idx_y, r_coords_idx_x
        diff_x, diff_y = diff_y, diff_x
    
    def line_formula(x):
        return (diff_y / diff_x) * (x - l_coords_idx_x) + l_coords_idx_y
    
    ret.append(point_left)
    for i_x in np.linspace(0, diff_x, np.abs(diff_x) + 1)[1:]:
        x_tmp = l_coords_idx_x + i_x
        y_tmp = int(np.round(line_formula(x_tmp)))

        if swap_flag:
            x_tmp, y_tmp = y_tmp, x_tmp
        
        tile_x_tmp = x_tmp // TILE_SIZE
        tile_y_tmp = y_tmp // TILE_SIZE
        idx_x_tmp = x_tmp % TILE_SIZE
        idx_y_tmp = y_tmp % TILE_SIZE

        tile_x_last, tile_y_last, idx_x_last, idx_y_last = ret[-1]
        
        if is_neighbor_tile(tile_x_last, tile_y_last, tile_x_tmp, tile_y_tmp):
            ret.append([tile_x_tmp, tile_y_tmp, idx_x_tmp, idx_y_tmp])

            if is_neighbor_tile(tile_x_tmp, tile_y_tmp, r_tile_x, r_tile_y):
                break

    ret.append(point_right)

    return ret


def force_neighbor_tile_bsearch(l_tile_x, l_tile_y, l_idx_x, l_idx_y,
                                 r_tile_x, r_tile_y, r_idx_x, r_idx_y):
    ret = []

    # Calculate middle point
    l_coords_idx_x = l_tile_x * TILE_SIZE + l_idx_x
    l_coords_idx_y = l_tile_y * TILE_SIZE + l_idx_y
    r_coords_idx_x = r_tile_x * TILE_SIZE + r_idx_x
    r_coords_idx_y = r_tile_y * TILE_SIZE + r_idx_y

    m_coords_idx_x = (l_coords_idx_x + r_coords_idx_x) // 2
    m_coords_idx_y = (l_coords_idx_y + r_coords_idx_y) // 2
    m_tile_x = m_coords_idx_x // TILE_SIZE
    m_tile_y = m_coords_idx_y // TILE_SIZE
    m_idx_x = m_coords_idx_x % TILE_SIZE
    m_idx_y = m_coords_idx_y % TILE_SIZE

    # Judge whether the middle point is neighbor of the left anf right point
    cond_neighbor_left = is_neighbor_tile(l_tile_x, l_tile_y, m_tile_x, m_tile_y)
    cond_neighbor_right = is_neighbor_tile(r_tile_x, r_tile_y, m_tile_x, m_tile_y)
    cond_same_left = is_same_tile(l_tile_x, l_tile_y, m_tile_x, m_tile_y)
    cond_same_right = is_same_tile(r_tile_x, r_tile_y, m_tile_x, m_tile_y)

    if is_neighbor_tile(l_tile_x, l_tile_y, r_tile_x, r_tile_y) \
       or (cond_same_left and cond_same_right):
        ret.append([l_tile_x, l_tile_y, l_idx_x, l_idx_y])
        ret.append([r_tile_x, r_tile_y, r_idx_x, r_idx_y])
    elif cond_same_left:
        ret.append([l_tile_x, l_tile_y, l_idx_x, l_idx_y])
        list_tmp = force_neighbor_tile_bsearch(m_tile_x, m_tile_y, m_idx_x, m_idx_y,
                                                r_tile_x, r_tile_y, r_idx_x, r_idx_y)
        ret.extend(list_tmp[1:])
    elif cond_same_right:
        list_tmp = force_neighbor_tile_bsearch(l_tile_x, l_tile_y, l_idx_x, l_idx_y,
                                                m_tile_x, m_tile_y, m_idx_x, m_idx_y)
        ret.extend(list_tmp[:-1])
        ret.append([r_tile_x, r_tile_y, r_idx_x, r_idx_y])
    else:
        if cond_neighbor_left and cond_neighbor_right:
            ret.append([l_tile_x, l_tile_y, l_idx_x, l_idx_y])
            ret.append([m_tile_x, m_tile_y, m_idx_x, m_idx_y])
            ret.append([r_tile_x, r_tile_y, r_idx_x, r_idx_y])
        elif cond_neighbor_left:
            ret.append([l_tile_x, l_tile_y, l_idx_x, l_idx_y])
            ret.extend(force_neighbor_tile_bsearch(m_tile_x, m_tile_y, m_idx_x, m_idx_y,
                                                    r_tile_x, r_tile_y, r_idx_x, r_idx_y))
        elif cond_neighbor_right:
            ret.extend(force_neighbor_tile_bsearch(l_tile_x, l_tile_y, l_idx_x, l_idx_y,
                                                    m_tile_x, m_tile_y, m_idx_x, m_idx_y))
            ret.append([r_tile_x, r_tile_y, r_idx_x, r_idx_y])
        else:
            ret.extend(force_neighbor_tile_bsearch(l_tile_x, l_tile_y, l_idx_x, l_idx_y,
                                                    m_tile_x, m_tile_y, m_idx_x, m_idx_y))
            ret.extend(force_neighbor_tile_bsearch(m_tile_x, m_tile_y, m_idx_x, m_idx_y,
                                                    r_tile_x, r_tile_y, r_idx_x, r_idx_y)[1:])

    return ret


def conv_route_coords_to_tile_idx_list(zoom, coords,
                                       tile_size=256,
                                       force_neighbor=False,
                                       force_neighbor_method='linear',
                                       verbose=False):
    
    tile_idx_list = []
    raw_tile_idx = []

    # Convert (lon, lat) to (tile_x, tile_y, idx_x, idx_y)
    for lon, lat in coords:
        _, tile_x, tile_y = GsijAltTile.calc_coords2tile_coords(zoom, lon, lat)
        idx_x, idx_y = GsijAltTile.calc_tile_idx(zoom, lon, lat, tile_size)

        tile_idx_tmp = [tile_x, tile_y, idx_x, idx_y]

        # Avoid duplication
        if len(raw_tile_idx) == 0 or raw_tile_idx[-1] != tile_idx_tmp:
            raw_tile_idx.append(tile_idx_tmp)

    if force_neighbor:
        raw_tile_idx = force_neighbor_tile(raw_tile_idx,
                                           method=force_neighbor_method)

    # Add some points to handle the tile edge problem
    if verbose:
        print(raw_tile_idx[0])

    tile_idx_list.append([zoom, *raw_tile_idx[0]])

    for prev, curr in zip(raw_tile_idx[:-1], raw_tile_idx[1:]):
        tile_x_prev, tile_y_prev, idx_x_prev, idx_y_prev = prev
        tile_x_curr, tile_y_curr, idx_x_curr, idx_y_curr = curr

        if [tile_x_prev, tile_y_prev] != [tile_x_curr, tile_y_curr]:
            # Add the last point on the previous tile
            tile_idx_tmp = [
                zoom, tile_x_prev, tile_y_prev,
                idx_x_curr + tile_size * (tile_x_curr - tile_x_prev),
                idx_y_curr + tile_size * (tile_y_curr - tile_y_prev)
            ]

            if verbose:
                print(tile_idx_tmp)

            tile_idx_list.append(tile_idx_tmp)

            # Add the first point on the next tile
            tile_idx_tmp = [
                zoom, tile_x_curr, tile_y_curr,
                idx_x_prev + tile_size * (tile_x_prev - tile_x_curr),
                idx_y_prev + tile_size * (tile_y_prev - tile_y_curr)
            ]

            if verbose:
                print(tile_idx_tmp)

            tile_idx_list.append(tile_idx_tmp)

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
                             collection_area_zoom=8,  # same value to the value in japan_tile_list_path json file
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
                os.makedirs(os.path.dirname(cache_path), exist_ok=True)
                Image.open(fp, formats=("PNG",)).convert("RGB").save(cache_path)
        elif res.status_code == 404:
            os.makedirs(os.path.dirname(cache_path), exist_ok=True)
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

    for zoom in sorted(target_zoom_level):
        route_tile_idx_list = conv_route_coords_to_tile_idx_list(zoom, route_coords, TILE_SIZE)
        write_route_tile_idx_to_route_dat(route_dat_dir_path, route_tile_idx_list)
    
        # Genrate initPoint file
        if zoom == max(target_zoom_level):
            coords_idx_x_init = route_tile_idx_list[0][1] * TILE_SIZE + route_tile_idx_list[0][3]
            coords_idx_y_init = route_tile_idx_list[0][2] * TILE_SIZE + route_tile_idx_list[0][4]

            print(zoom)
            print(coords_idx_x_init, coords_idx_y_init)

            init_point_file_path = os.path.join(output_dir_path, 'initPoint')
            with open(init_point_file_path, 'wb') as fp:
                # i: signed integer (4 bytes = 32 bits)
                # <: little endian
                buf = struct.Struct('<iii').pack(max(target_zoom_level),
                                                 coords_idx_x_init, coords_idx_y_init)
                fp.write(buf)

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

    # Generate use_sound file
    use_sound_file_path = os.path.join(output_dir_path, 'useSound')
    open(use_sound_file_path, 'w').close()


if __name__ == '__main__':
    route_file_path = sys.argv[1]

    main(route_file_path)
    