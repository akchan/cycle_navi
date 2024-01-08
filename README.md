# cycle_navi

M5Stack Core2を使ったサイクリング用GPSナビのコードです。

Simple GPS navigation system using M5Stack Core2

- Version 1 uses serialized NMEA GPS data from GPS module (e.g. M5Stack GPS module v2)
- Version 2 uses external bluetooth SSP (serial port profile) GPS receiver

![](gen_map_and_route/overview.JPG)

## Main features

- Display current location based on GPS
- Scroll the map using the touch screen of M5Stack Core2
- Receive GPS data from an external GPS receiver via bluetooth (SSP, serial port profile)
- Show planned routes using scripts in `gen_map_and_route`

# Hardware

### Required

- M5Stack Core2
- M5Go for core2 bottom mobule
    - Use M3 screw on the bottom to mount M5Stack to the handle of your bicycle.
- SD card (TF card)
    - formatted as exFAT with small allocation unit size (ex. 2048 byte/block)
    - If you use mac, use `newfs_exfat -b 2048 {device}` command instead of `Disk utility.app` which can't specify the allocation unit size.
- GPS receiver
    - sould be compatible with bluetooth SSP
    - You can use your android phones via this App (GNSS2bluetooth) developed for this project.
        - [https://github.com/akchan/cycle_navi/releases/tag/gnss2bluetooth](https://github.com/akchan/cycle_navi/releases/tag/gnss2bluetooth)

### Recommended

- USB battery (for extended operating time)

### How to use internal GPS module

If you want to use a GPS module of M5Stack series, you should use version 1 and modules like below.

- M5Stack GPS module v2
- External GPS antena (for better signal)

# Sofrware requirements

Install the required packages below via Arduino library manager before compile and upload this project to your device.

- M5Stack Core2 board library (2.0.8)
- TinyGPSPlus (1.0.3)
- SdFat (2.2.2)
- LovyanGFX (1.1.12)

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
