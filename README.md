# cycle_navi

M5Stack Core2を使ったサイクリング用ナビのコードです。

Simple navigation system using M5Stack Core2

![](gen_map_and_route/overview.JPG)

## Main features

- Display current location based on GPS
- Scroll the map using the touch screen of M5Stack Core2
- Show planned routes using scripts in `gen_map_and_route`

# Hardware

### Required

- M5Stack Core2
- M5Stack GPS module v2
- M5Go for core2 bottom mobule
    - Use M3 screw on the bottom to mount M5Stack to the handle of your bicycle.
- SD card (TF card)
    - formatted as exFAT with small allocation unit size (ex. 2048 byte/block)
    - If you use mac, use `newfs_exfat -b 2048 {device}` command instead of `Disk utility.app` which can't specify the allocation unit size.

### Recommended

- External GPS antena (for better signal)
    - Active type of antenna is preferred.
- USB battery (for extended operating time)

# Sofrware requirements

- M5Unified (0.1.12)
- TinyGPSPlus (1.0.3)
- SdFat (2.2.2)

# Usage

```bash
# 1. Copy this repository to local
git clone https://github.com/akchan/cycle_navi

# 2. Install python dependencies
pip install -r gen_map_and_route/requirements.txt

# 3. Prepare cycling route file. Your own file or included sample files (in gen_map_and_route/sample_gpx) can be used.

# 4. Generate directories for M5Stack (route_dat, point_dat, map)
cd gen_map_and_route
python gen_route_dat.py sample_gpx/tokyo_sample_route.kml

# 5. Copy generated directories to root directory of the SD card.
cp -r map_route /mnt/sd
umount /mnt/sd

# 6. Insert the SD card to M5Stack Core2.
# 7. Write M5Stack Core2 program via arduino IDE.
```

# Licence

MIT licence (see LICENSE_MIT.txt)
