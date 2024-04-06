/* GPS tracker using M5stack
 *
 * Created on 2022/01/08 Sat.
 *
 * Requirements
 * ============
 *
 * - M5stack core2
 * - M5stack GPS module v2
 * - Enabled PSRAM
 *
 * Memo
 * ====
 *
 * sizeof(int) -> 4 on M5stack
 *
 * GPU部分を整理。BLE SPP対応する。
 * LovyanGFVの作り直し
 * 描画プロセスの再検討。指離してから読み込みするようにする。
 */

#include <SdFat.h> // Define SdFat.h before LovyanGFX.hpp
#include <M5Unified.h>
#include <TinyGPSPlus.h> // Installed through arduino IDE library manager

#include <math.h>

#include <driver/i2s.h>
#include "sound_boot.h"
#include "sound_gps_active.h"
#include "sound_gps_inactive.h"
#include "icon_satellite.h"
#include "icon_bluetooth.h"

// ===============================================================================
// Variables declaration
// ================================================================================
// Common
#define CYCLE_NAVI_VERSION 300
#define CYCLE_NAVI_VERSION_STRING "3.0"
#define SUCCESS 1
#define ERROR 0
#define VERBOSE 1 // Set non-zero for verbose log mode

// Settings
const unsigned long interval_sec = 1;
const unsigned long smart_loading_delay_ms = 350;
const float is_moved_cutoff_m = 0.7;
const char map_dir_path[] = "/map";
const char route_dir_path[] = "/route_dat";
const char point_dir_path[] = "/point_dat";
const char init_point_path[] = "/initPoint";
#define TIMEZONE_HOUR 9
#define SERIAL_BAUDRATE_GPS 19200

// Variables [Timers]
unsigned long interval_ms = interval_sec * 1000;
unsigned long t_prev, t_curr;
unsigned long t_last_touch_move = 0;

// Variables [GPS]
TinyGPSPlus gps;
bool is_gps_active = false;
const int gps_count_th = 3;
int gps_active_counter = 0;
bool isUpdatedPrev = false;

// Variables [Coordinates]
struct st_tile_coords
{
    int zoom;
    int tile_x;
    int tile_y;
};
struct st_idx_coords
{
    int zoom;
    int idx_x;
    int idx_y;
};
// Initial point (Tokyo station)
const int z_init = 14;
const int idx_coords_x_init = 14552 * 256 + 218;
const int idx_coords_y_init = 6451 * 256 + 165;
st_idx_coords curr_gps_idx_coords = {
    -1,
    idx_coords_x_init,
    idx_coords_y_init};
st_idx_coords display_center_idx_coords = {
    z_init,
    idx_coords_x_init,
    idx_coords_y_init};
int i_smart_loading = 0;
bool centering_mode = true;

// Variables [map & route]
#define LEN_FILE_PATH 35
const int tile_size = 256;
const int tile_size_power = (int)round(log2(tile_size));
const int n_sprite_x = 3;
const int n_sprite_y = 3;
const int n_sprite = n_sprite_x * n_sprite_y;
static M5Canvas canvas(&M5.Display); // screen buffer
M5Canvas dir_icon(&canvas);
M5Canvas gps_icon(&canvas);
struct sprite_struct
{
    st_tile_coords tile_coords;
    M5Canvas *sprite;
    bool is_update_required;
};
sprite_struct tile_cache_buf[n_sprite];
sprite_struct *tile_cache[n_sprite];
bool is_tile_cache_initialized = false;

// Variables [Zoom]
#define LEN_ZOOM_LIST 20
struct st_zoom
{
    int i;
    int n;
    int list[LEN_ZOOM_LIST];
};
st_zoom zoom;

// Variables [Colors]
// Color name:
//   TFT_BLACK, TFT_NAVY, TFT_DARKGREEN, TFT_MAROON, TFT_PURPLE,
//   TFT_OLIVE, TFT_LIGHTGREY, TFT_DARKGREY, TFT_BLUE, TFT_GREENYELLOW,
//   TFT_GREEN, TFT_YELLOW, TFT_ORANGE, TFT_PINK, TFT_CYAN, TFT_DARKCYAN,
//   TFT_RED, TFT_MAGENTA, TFT_WHITE
#define NO_CACHE_COLOR TFT_WHITE
#define NO_IMG_COLOR TFT_DARKGREY
#define ROUTE_COLOR TFT_BLUE
#define POINT_COLOR TFT_DARKGREEN
#define ROUTE_WIDTH 7
#define POINT_R 10

#define DIR_ICON_COLOR_ACTIVE TFT_BLUE
#define DIR_ICON_COLOR_INACTIVE TFT_DARKGREY
#define DIR_ICON_BG_COLOR TFT_WHITE
#define DIR_ICON_TRANS_COLOR TFT_BLACK
#define DIR_ICON_R 14
#define DIR_ICON_EDGE_WIDTH 3
#define DIR_ICON_ANGLE 45.0 / 180.0 * M_PI
const int dir_icon_palette_id_trans = 0;
const int dir_icon_palette_id_bg = 1;
const int dir_icon_palette_id_fg = 2;

// Variables [Brightness]
const int brightness_list[] = {255, 128, 64, 32};
const int n_brightness = 4;
int i_brightness = 0;

// Variables [Sound]
bool use_sound;

// ================================================================================
// SdFat settings
// ================================================================================
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3 // Use SD_FAT_TYPE 3 for LovyanGFX
SdFs sd;
FsFile file;
#define SDFAT_FSFILE_TYPE FsFile

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
// The max SPI clock is around 25 MHz for M5Stack Core2.
#define SPI_CLOCK SD_SCK_MHZ(25)

// Try to select the best SD card configuration.
// M5Core2 shares SPI but between SD card and the LCD display
#define TFCARD_CS_PIN 4
#define SD_CONFIG SdSpiConfig(TFCARD_CS_PIN, SHARED_SPI, SPI_CLOCK)

/* ================================================================================
 * GPS
 * ================================================================================
 * LonLat: Longitude and latitude system
 * IdxCoords: Pixel index of web Mercator system
 * TileCoords: Tile index of web Mercator system
 */

int calcLonLat2TileCoords(st_tile_coords &tile_coords,
                          int zoom, double lon, double lat,
                          int method = 2)
{
    // Methods:
    //   0: tan
    //   1: sin
    //   2: sin_cos
    //   3: cos_tan

    if (lon < -180.0 || 180.0 < lon)
    {
        return ERROR;
    }

    if (-90.0 <= lat && lat <= 90.0)
    {
        // valid latitude case
        double r = 1.0 / (2 * M_PI);
        double x_raw = (lon + 180.0) / 360.0;

        double y_raw, rad;
        if (method == 0)
        {
            rad = M_PI * (90.0 + lat) / 360.0;
            y_raw = r * log(tan(rad));
        }
        else if (method == 1)
        {
            rad = lat / 180.0 * M_PI;
            y_raw = r / 2.0 * log((1.0 + sin(rad)) / (1.0 - sin(rad)));
        }
        else if (method == 2)
        {
            rad = lat / 180.0 * M_PI;
            y_raw = r * log((1.0 + sin(rad)) / cos(rad));
        }
        else if (method == 3)
        {
            rad = lat / 180.0 * M_PI;
            y_raw = r * log(1.0 / cos(rad) + tan(rad));
        }
        else
        {
            return ERROR;
        }

        tile_coords.zoom = zoom;
        tile_coords.tile_x = (int)floor(x_raw * (1 << zoom));
        tile_coords.tile_y = (int)floor((0.5 - y_raw) * (1 << zoom));

        return SUCCESS;
    }
    return ERROR;
}

int calcLonLat2IdxCoords(st_idx_coords &idx_coords, double lon, double lat,
                         int zoom, int tile_size_power)
{
    /*
     * tile_size_power: log2(tile_size)
     */
    st_tile_coords tile_coords_tmp;
    int ret;

    int zoom_for_idx_coords = zoom + tile_size_power;

    ret = calcLonLat2TileCoords(tile_coords_tmp, zoom_for_idx_coords, lon, lat);
    if (ret == ERROR)
    {
        return ERROR;
    }

    idx_coords.zoom = zoom;
    idx_coords.idx_x = tile_coords_tmp.tile_x;
    idx_coords.idx_y = tile_coords_tmp.tile_y;

    return SUCCESS;
}

void convIdxCoords2TileCoords(st_idx_coords &idx_coords, st_tile_coords &tile_coords,
                              int tile_size)
{
    tile_coords.zoom = idx_coords.zoom;
    tile_coords.tile_x = idx_coords.idx_x / tile_size;
    tile_coords.tile_y = idx_coords.idx_y / tile_size;
}

void convIdxCoordsForZoom(st_idx_coords &src_idx_coords, st_idx_coords &dst_idx_coords,
                          bool correct_tile_center = true)
{
    int zoom_diff = dst_idx_coords.zoom - src_idx_coords.zoom;
    double idx_x = src_idx_coords.idx_x;
    double idx_y = src_idx_coords.idx_y;

    if (correct_tile_center)
    {
        idx_x += 0.5;
        idx_y += 0.5;
    }

    dst_idx_coords.idx_x = (int)floor(idx_x * pow(2, zoom_diff));
    dst_idx_coords.idx_y = (int)floor(idx_y * pow(2, zoom_diff));
}

void checkGPS()
{
    if (gps.location.isValid())
    {
        // gps.location status is updated on calling gps.location.lng() or gps.location.lat()
        bool isUpdated = gps.location.isUpdated();

        if (isUpdated != isUpdatedPrev)
        {
            gps_active_counter = 0;
        }

        isUpdatedPrev = isUpdated;

        if (isUpdated)
        {
            if (VERBOSE)
            {
                Serial.println("checkGPS(): GPS is available.");
            }

            if ((!is_gps_active) && gps_active_counter == gps_count_th)
            {
                is_gps_active = true;

                if (use_sound)
                {
                    playGPSActive();
                    if (VERBOSE)
                    {
                        Serial.println("checkGPS(): playGPSActive() was invoked.");
                    }
                }
            }
        }
        else
        {
            if (VERBOSE)
            {
                Serial.println("checkGPS(): GPS is unavailable.");
            }

            if (is_gps_active && gps_active_counter == gps_count_th)
            {
                is_gps_active = false;

                if (use_sound)
                {
                    playGPSInactive();
                    if (VERBOSE)
                    {
                        Serial.println("checkGPS(): playGPSInactive() was invoked.");
                    }
                }
            }
        }
        gps_active_counter++;

        // Update curr_gps_idx_coords with gps data
        calcLonLat2IdxCoords(curr_gps_idx_coords, gps.location.lng(), gps.location.lat(),
                             zoom_level(), tile_size_power);

        if (centering_mode)
        {
            display_center_idx_coords.zoom = zoom_level();
            display_center_idx_coords.idx_x = curr_gps_idx_coords.idx_x;
            display_center_idx_coords.idx_y = curr_gps_idx_coords.idx_y;
        }

        // Update RTC
        m5::rtc_datetime_t datetime;
        datetime.date.year = gps.date.year();
        datetime.date.month = gps.date.month();
        datetime.date.date = gps.date.day();
        datetime.time.hours = gps.time.hour();
        datetime.time.minutes = gps.time.minute();
        datetime.time.seconds = gps.time.second();
        M5.Rtc.setDateTime(&datetime);

        if (VERBOSE)
        {
            Serial.printf("checkGPS()\n");
            Serial.printf("  gps=(%f, %f), satellites=%d\n",
                          gps.location.lng(),
                          gps.location.lat(),
                          gps.satellites.value());
            Serial.printf("  curr_gps_idx_coords=(z=%d,x=%d,y=%d)\n",
                          curr_gps_idx_coords.zoom,
                          curr_gps_idx_coords.idx_x,
                          curr_gps_idx_coords.idx_y);
            Serial.printf("  display_center_idx_coords=(z=%d,x=%d,y=%d)\n",
                          display_center_idx_coords.zoom,
                          display_center_idx_coords.idx_x,
                          display_center_idx_coords.idx_y);
            Serial.printf("  RTC updated: %04d/%02d/%02d %02d:%02d:%02d",
                          datetime.date.year,
                          datetime.date.month,
                          datetime.date.date,
                          datetime.time.hours,
                          datetime.time.minutes,
                          datetime.time.seconds);
        }
    }
    else
    {
        if (VERBOSE)
        {
            Serial.printf("checkGPS(): gps.location.isValid() = false\n");
        }
    }
}
// ================================================================================
// Tile cache
// ================================================================================
void genMapPath(char *file_path,
                int z, int tile_x, int tile_y)
{
    snprintf(file_path, LEN_FILE_PATH, "%s/%d/%d/%d.jpg", map_dir_path, z, tile_x,
             tile_y);
}

void initTileCache()
{
    int cap_ram, cap_spiram;
    int cap_ram_after, cap_spiram_after;
    if (VERBOSE)
    {
        cap_ram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        cap_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

        Serial.printf("initTileCache():\n");
        Serial.printf("    heap_caps_get_free_size(MALLOC_CAP_INTERNAL):%8d byte\n", cap_ram);
        Serial.printf("    heap_caps_get_free_size(MALLOC_CAP_SPIRAM):  %8d byte\n", cap_spiram);
    }

    for (int i = 0; i < n_sprite; i++)
    {
        tile_cache[i] = &tile_cache_buf[i];

        tile_cache[i]->sprite = new M5Canvas(&canvas);
        tile_cache[i]->sprite->setPsram(true);
        tile_cache[i]->sprite->createSprite(tile_size, tile_size);
        tile_cache[i]->sprite->fillSprite(NO_CACHE_COLOR);

        tile_cache[i]->tile_coords.zoom = -1;
        tile_cache[i]->tile_coords.tile_x = -1;
        tile_cache[i]->tile_coords.tile_y = -1;

        tile_cache[i]->is_update_required = false;
    }

    if (VERBOSE)
    {
        cap_ram_after = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        cap_spiram_after = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

        Serial.printf("  tile_cache[%d] allocated.\n", n_sprite);
        Serial.printf("    heap_caps_get_free_size(MALLOC_CAP_INTERNAL):%8d byte (%d)\n",
                      cap_ram_after, cap_ram_after - cap_ram);
        Serial.printf("    heap_caps_get_free_size(MALLOC_CAP_SPIRAM):  %8d byte (%d)\n",
                      cap_spiram_after, cap_spiram_after - cap_spiram);
    }

    is_tile_cache_initialized = true;
}

void loadTile(M5Canvas *sprite, int zoom, int tile_x, int tile_y)
{
    char file_path[LEN_FILE_PATH];
    uint32_t t;

    genMapPath(file_path, zoom, tile_x, tile_y);

    if (file.open(file_path, O_RDONLY))
    {
        if (VERBOSE)
        {
            t = millis();
        }

        sprite->drawJpgFile(sd, file_path);

        if (VERBOSE)
        {
            t = millis() - t;
            Serial.printf("loadTile(): Updated (%d ms) %s\n", t, file_path);
        }
    }
    else
    {
        sprite->fillSprite(NO_IMG_COLOR);
        if (VERBOSE)
        {
            Serial.printf("loadTile(): Not found %s\n", file_path);
        }
    }
    file.close();
}

int read4BytesAsInt(SDFAT_FSFILE_TYPE &file)
{
    int ret;

    file.read((uint8_t *)&ret, sizeof(int));

    return ret;
}

int mod(const int a, const int b)
{
    /*
     * Return: Positive modulo evenif the variable a is negative.
     */
    int mod = a % b;
    if (mod < 0)
    {
        mod += b;
    }
    return mod;
}

bool isStTileCoordsEqual(st_tile_coords &a, st_tile_coords &b)
{
    return a.zoom == b.zoom && a.tile_x == b.tile_x && a.tile_y == b.tile_y;
}

void shiftTileCache(int shift_x, int shift_y)
{
    int idx_shifted_x;
    int idx_shifted_y;
    int k;
    sprite_struct *tile_cache_tmp[n_sprite];

    for (int i = 0; i < n_sprite_x; i++)
    {
        for (int j = 0; j < n_sprite_y; j++)
        {
            idx_shifted_x = mod(i + shift_x, n_sprite_x);
            idx_shifted_y = mod(j + shift_y, n_sprite_y);
            k = idx_shifted_x + idx_shifted_y * n_sprite_x;

            tile_cache_tmp[k] = tile_cache[i + j * n_sprite_x];
        }
    }

    for (int i = 0; i < n_sprite; i++)
    {
        tile_cache[i] = tile_cache_tmp[i];
    }
}

void checkSpriteUpdateRequired()
{
    int k;
    const int i_center = n_sprite_x / 2;
    const int j_center = n_sprite_y / 2;
    st_tile_coords center_tile_coords;
    st_tile_coords tile_coords_curr;
    st_tile_coords tile_coords_tgt;

    if (VERBOSE)
    {
        Serial.printf("checkSpriteUpdateRequired():\n");
    }

    convIdxCoords2TileCoords(display_center_idx_coords, center_tile_coords, tile_size);

    for (int i = 0; i < n_sprite_x; i++)
    {
        for (int j = 0; j < n_sprite_y; j++)
        {
            k = i + j * n_sprite_x;

            if (VERBOSE)
            {
                Serial.printf("  i=%d,j=%d,k=%d ", i, j, k);
            }

            tile_coords_tgt.zoom = center_tile_coords.zoom;
            tile_coords_tgt.tile_x = center_tile_coords.tile_x + (i - i_center);
            tile_coords_tgt.tile_y = center_tile_coords.tile_y + (j - j_center);

            if (isStTileCoordsEqual(tile_cache[k]->tile_coords, tile_coords_tgt))
            {
                if (VERBOSE)
                {
                    Serial.printf("up-to-date");
                }
            }
            else
            {
                if (VERBOSE)
                {
                    Serial.printf(" update required. ");
                    Serial.printf("curr=(z=%d,x=%d,y=%d) ",
                                  tile_cache[k]->tile_coords.zoom,
                                  tile_cache[k]->tile_coords.tile_x,
                                  tile_cache[k]->tile_coords.tile_y);
                    Serial.printf("tgt=(z=%d,x=%d,y=%d) ",
                                  tile_coords_tgt.zoom,
                                  tile_coords_tgt.tile_x,
                                  tile_coords_tgt.tile_y);
                }

                tile_cache[k]->tile_coords = tile_coords_tgt;
                tile_cache[k]->is_update_required = true;
                tile_cache[k]->sprite->fillSprite(NO_CACHE_COLOR);
            }

            Serial.printf("\n");
        }
    }
}

void updateTileCache()
{
    /*
     * Update tile_cache based on display_center_idx_coords
     *
     * Return:
     *
     * If any tiles updated (or shifted), true will be returned.
     */
    const int i_center_sprite = n_sprite / 2;
    int tile_shift_x = 0;
    int tile_shift_y = 0;
    st_tile_coords center_tile_coords;
    st_tile_coords center_tile_coords_tgt;

    if (VERBOSE)
    {
        Serial.printf("updateTileCache():\n");
    }

    if (!is_tile_cache_initialized)
    {
        if (VERBOSE)
        {
            Serial.printf("  Passed. tile_cache is not initialized.\n");
        }
        return;
    }

    center_tile_coords = tile_cache[i_center_sprite]->tile_coords;
    convIdxCoords2TileCoords(display_center_idx_coords, center_tile_coords_tgt, tile_size);

    if (VERBOSE)
    {
        Serial.printf("  display_center_idx_coords (z=%d,x=%d,y=%d)\n",
                      display_center_idx_coords.zoom,
                      display_center_idx_coords.idx_x,
                      display_center_idx_coords.idx_y);
        Serial.printf("  center_tile_coords        (z=%d,x=%d,y=%d)\n",
                      center_tile_coords.zoom,
                      center_tile_coords.tile_x,
                      center_tile_coords.tile_y);
        Serial.printf("  center_tile_coords_tgt    (z=%d,x=%d,y=%d)\n",
                      center_tile_coords_tgt.zoom,
                      center_tile_coords_tgt.tile_x,
                      center_tile_coords_tgt.tile_y);
    }

    if (!isStTileCoordsEqual(center_tile_coords, center_tile_coords_tgt))
    {
        tile_shift_x = center_tile_coords.tile_x - center_tile_coords_tgt.tile_x;
        tile_shift_y = center_tile_coords.tile_y - center_tile_coords_tgt.tile_y;

        if (VERBOSE)
        {
            Serial.printf("  tile_shift (x=%d,y=%d)\n", tile_shift_x, tile_shift_y);
        }

        shiftTileCache(tile_shift_x, tile_shift_y);

        checkSpriteUpdateRequired();
    }
}

void pushTileCache()
{
    int is_update_required;
    int x, y;

    if (VERBOSE)
    {
        Serial.printf("pushTileCache():\n");
    }

    for (int i = 0; i < n_sprite; i++)
    {
        x = tile_cache[i]->tile_coords.tile_x * tile_size - display_center_idx_coords.idx_x + M5.Display.width() / 2;
        y = tile_cache[i]->tile_coords.tile_y * tile_size - display_center_idx_coords.idx_y + M5.Display.height() / 2;

        tile_cache[i]->sprite->pushSprite(x, y);

        if (VERBOSE)
        {
            Serial.printf("  i:%i, tile=(%d,%d), offset=(%d,%d)\n", i, tile_cache[i]->tile_coords.tile_x, tile_cache[i]->tile_coords.tile_y, x, y);
        }
    }
}

// ================================================================================
// Smart tile loading
// ================================================================================
void smartTileLoading(const bool draw_canvas = true)
{
    int j;
    int tile_zoom, tile_x, tile_y;
    M5Canvas *p_tile_sprite;

    if (VERBOSE)
    {
        Serial.printf("smartTileLoading():\n");
    }

    if (!is_tile_cache_initialized)
    {
        if (VERBOSE)
        {
            Serial.printf("  tile_cache is not initialized.\n");
        }
        return;
    }

    if (t_last_touch_move + smart_loading_delay_ms > millis())
    {
        return;
    }

    // Prepare array of indices sorted by proximity to the current point
    float curr_center_x = float(display_center_idx_coords.idx_x) / tile_size;
    float curr_center_y = float(display_center_idx_coords.idx_y) / tile_size;
    std::array<int, n_sprite> idx_ary;
    std::array<float, n_sprite> dist_ary;
    for (int i = 0; i < n_sprite; i++)
    {
        idx_ary[i] = i;
        dist_ary[i] = fabs(tile_cache[i]->tile_coords.tile_x + 0.5 - curr_center_x) + fabs(tile_cache[i]->tile_coords.tile_y + 0.5 - curr_center_y);
    }
    std::sort(idx_ary.begin(), idx_ary.end(), [&](int idx_a, int idx_b)
              { return dist_ary[idx_a] < dist_ary[idx_b]; });

    if (VERBOSE)
    {
        Serial.printf("  Current center: %f %f\n",
                      curr_center_x,
                      curr_center_y);
    }

    // Check each sprite
    for (int i = 0; i < n_sprite; i++)
    {
        j = idx_ary[i];

        if (tile_cache[j]->is_update_required)
        {
            M5.update();
            auto t = M5.Touch.getDetail();

            if (t.isPressed())
            {
                if (VERBOSE)
                {
                    Serial.printf("  tile_cache[%d] interrupted\n", j);
                }

                break;
            }

            p_tile_sprite = tile_cache[j]->sprite;
            tile_zoom = tile_cache[j]->tile_coords.zoom;
            tile_x = tile_cache[j]->tile_coords.tile_x;
            tile_y = tile_cache[j]->tile_coords.tile_y;

            if (VERBOSE)
            {
                Serial.printf("  Updating (i=%d,z=%d,tile_x=%d,tile_y=%d)\n    ",
                              i, tile_zoom, tile_x, tile_y);
            }

            loadTile(p_tile_sprite, tile_zoom, tile_x, tile_y);

            if (VERBOSE)
            {
                Serial.printf("    ");
            }
            loadRoute(p_tile_sprite, tile_zoom, tile_x, tile_y);

            if (VERBOSE)
            {
                Serial.printf("    ");
            }
            loadPoint(p_tile_sprite, tile_zoom, tile_x, tile_y);

            tile_cache[j]->is_update_required = false;

            if (draw_canvas)
            {
                drawCanvas();
            }
        }
    }
}

void smartTileLoadingOld(const bool draw_canvas = true)
{
    int j;
    int tile_zoom, tile_x, tile_y;
    M5Canvas *p_tile_sprite;

    if (VERBOSE)
    {
        Serial.printf("smartTileLoading():\n");
    }

    if (!is_tile_cache_initialized)
    {
        if (VERBOSE)
        {
            Serial.printf("  tile_cache is not initialized.\n");
        }
        return;
    }

    for (int i = 0; i < n_sprite; i++)
    {
        j = mod(i_smart_loading + i, n_sprite);

        if (tile_cache[j]->is_update_required)
        {
            M5.update();
            auto t = M5.Touch.getDetail();

            if (t.isPressed())
            {
                i_smart_loading = j;

                if (VERBOSE)
                {
                    Serial.printf("  tile_cache[%d] interrupted\n", j);
                }

                break;
            }

            if (VERBOSE)
            {
                Serial.printf("  Updating (i=%d,z=%d,tile_x=%d,tile_y=%d)\n    ",
                              i, tile_zoom, tile_x, tile_y);
            }

            p_tile_sprite = tile_cache[j]->sprite;
            tile_zoom = tile_cache[j]->tile_coords.zoom;
            tile_x = tile_cache[j]->tile_coords.tile_x;
            tile_y = tile_cache[j]->tile_coords.tile_y;

            loadTile(p_tile_sprite, tile_zoom, tile_x, tile_y);

            if (VERBOSE)
            {
                Serial.printf("    ");
            }
            loadRoute(p_tile_sprite, tile_zoom, tile_x, tile_y);

            if (VERBOSE)
            {
                Serial.printf("    ");
            }
            loadPoint(p_tile_sprite, tile_zoom, tile_x, tile_y);

            tile_cache[j]->is_update_required = false;

            if (draw_canvas)
            {
                drawCanvas();
            }
        }
    }
}
// ================================================================================
// MapTile
// ================================================================================
bool isWithinTile(int x, int y, int tile_size_x, int tile_size_y)
{
    if (0 <= x && x < tile_size_x && 0 <= y && y < tile_size_y)
        return true;
    else
        return false;
}

void initMapVariables()
{
    if (file.open(init_point_path, O_RDONLY))
    {
        if (VERBOSE)
        {
            Serial.println("initMapVariables(): initPoint file was detected.");
        }

        curr_gps_idx_coords.zoom = read4BytesAsInt(file);
        curr_gps_idx_coords.idx_x = read4BytesAsInt(file);
        curr_gps_idx_coords.idx_y = read4BytesAsInt(file);
    }
    else
    {
        if (VERBOSE)
        {
            Serial.println("initMapVariables(): initPoint file was not detected.");
            Serial.println("initMapVariables(): The default point was loaded.");
        }

        curr_gps_idx_coords.zoom = z_init;
        curr_gps_idx_coords.idx_x = idx_coords_x_init;
        curr_gps_idx_coords.idx_y = idx_coords_y_init;
    }
    file.close();

    if (VERBOSE)
    {
        Serial.printf("initMapVariables(): zoom:%d, curr_gps_idx_coords idx_x:%d, idx_y:%d\n", curr_gps_idx_coords.zoom, curr_gps_idx_coords.idx_x, curr_gps_idx_coords.idx_y);
    }

    display_center_idx_coords = curr_gps_idx_coords;
}

void initCanvas()
{
    canvas.setPsram(true);
    canvas.createSprite(M5.Display.width(), M5.Display.height());
    canvas.fillSprite(NO_CACHE_COLOR);
}

void genRoutePath(char *file_path, int z, int tile_x, int tile_y)
{
    snprintf(file_path, LEN_FILE_PATH, "%s/%d/%d/%d.dat", route_dir_path, z,
             tile_x, tile_y);
}

void genPointPath(char *file_path, int z, int tile_x, int tile_y)
{
    snprintf(file_path, LEN_FILE_PATH, "%s/%d/%d/%d.dat", point_dir_path, z,
             tile_x, tile_y);
}

void loadRoute(M5Canvas *sprite, int zoom, int tile_x, int tile_y)
{
    int prev_point_x, prev_point_y, point_x, point_y;
    char file_path[LEN_FILE_PATH];

    if (VERBOSE)
    {
        Serial.print("loadRoute(): ");
    }

    genRoutePath(file_path, zoom, tile_x, tile_y);

    if (file.open(file_path, O_RDONLY))
    {
        uint64_t route_dat_size = file.fileSize();
        int size_of_a_point = (sizeof(int) * 2);

        if (route_dat_size % size_of_a_point == 0 &&
            route_dat_size / size_of_a_point > 1)
        {
            if (VERBOSE)
            {
                Serial.printf("Loaded from %s\n", file_path);
            }

            if (file.available())
            {
                prev_point_x = read4BytesAsInt(file);
                prev_point_y = read4BytesAsInt(file);
            }

            while (file.available())
            {
                point_x = read4BytesAsInt(file);
                point_y = read4BytesAsInt(file);

                // Conditions to avoid an inapproriate line occuring when
                // the route comes back to the same tile
                if (isWithinTile(point_x, point_y, tile_size, tile_size) || isWithinTile(prev_point_x, prev_point_y, tile_size, tile_size))
                {
                    drawLineWithStroke(sprite, prev_point_x,
                                       prev_point_y, point_x, point_y,
                                       ROUTE_WIDTH, ROUTE_COLOR);
                }

                prev_point_x = point_x;
                prev_point_y = point_y;
            }
        }
        else
        {
            if (VERBOSE)
            {
                Serial.printf("Invalid or small size %s\n", file_path);
            }
        }
    }
    else
    {
        if (VERBOSE)
        {
            Serial.printf("No route at %s\n", file_path);
        }
    }
    file.close();
}

void loadPoint(M5Canvas *sprite, int zoom, int tile_x, int tile_y)
{
    int point_x, point_y;
    char file_path[LEN_FILE_PATH];

    if (VERBOSE)
    {
        Serial.print("loadPoint():");
    }

    genPointPath(file_path, zoom, tile_x, tile_y);

    if (file.open(file_path, O_RDONLY))
    {
        if (file.fileSize() % (sizeof(int) * 2) == 0 &&
            file.fileSize() / sizeof(int) / 2 > 1)
        {
            if (VERBOSE)
            {
                Serial.printf("Point dat was detected.  %s\n", file_path);
            }

            while (file.available())
            {
                point_x = read4BytesAsInt(file);
                point_y = read4BytesAsInt(file);

                sprite->fillCircle(point_x, point_y, POINT_R, POINT_COLOR);
            }
        }
        else
        {
            if (VERBOSE)
            {
                Serial.printf("Invalid point dat:  %s\n", file_path);
            }
        }
    }
    else
    {
        if (VERBOSE)
        {
            Serial.printf("Point dat was not found.  %s\n", file_path);
        }
    }
    file.close();
}

void drawLineWithStroke(M5Canvas *sprite, int x_1, int y_1, int x_2, int y_2,
                        int stroke, uint16_t color)
{
    int center_i = stroke / 2; // floored automatically
    int r = (stroke - 1) / 2;
    int x_shift, y_shift;
    double angle_rad;

    if (x_1 != x_2)
    {
        angle_rad = atan((double)(y_2 - y_1) / (x_2 - x_1));
    }
    else
    {
        angle_rad = M_PI_2;
    }

    // when line is nearly horizontal
    if (-M_PI_4 <= angle_rad && angle_rad <= M_PI_4)
    {
        for (int i_y = 0; i_y < stroke; i_y++)
        {
            y_shift = i_y - center_i;
            sprite->drawLine(x_1, y_1 + y_shift, x_2, y_2 + y_shift, color);
        }
    }
    else // when line is nearly vertical
    {
        for (int i_x = 0; i_x < stroke; i_x++)
        {
            x_shift = i_x - center_i;
            sprite->drawLine(x_1 + x_shift, y_1, x_2 + x_shift, y_2, color);
        }
    }

    if (r > 0)
    {
        // Make round edges
        sprite->fillCircle(x_1, y_1, r, color);
        sprite->fillCircle(x_2, y_2, r, color);
    }
}

void initDirIcon()
{
    /*
     * dir_icon color palette:
     *   0: DIR_ICON_TRANS_COLOR
     *   1: DIR_ICON_BG_COLOR
     *   2: foreground color (DIR_ICON_COLOD_ACTIVE or DIR_ICON_COLOR_INACTIVE)
     *   3: not used (default is TFT_WHITE)
     */
    if (VERBOSE)
    {
        Serial.printf("initDirIcon(): Initializing direction icon\n");
    }

    // Allocate sprite
    dir_icon.setColorDepth(2);
    gps_icon.setPsram(false);
    dir_icon.createSprite(DIR_ICON_R * 2 + 1, DIR_ICON_R * 2 + 1);

    // Set palette colors
    dir_icon.setPaletteColor(dir_icon_palette_id_trans, DIR_ICON_TRANS_COLOR);
    dir_icon.setPaletteColor(dir_icon_palette_id_bg, DIR_ICON_BG_COLOR);
    dir_icon.setPaletteColor(dir_icon_palette_id_fg, DIR_ICON_COLOR_INACTIVE);

    // Draw icon
    dir_icon.fillSprite(dir_icon_palette_id_trans); // translucent background
    dir_icon.fillCircle(DIR_ICON_R, DIR_ICON_R, DIR_ICON_R, dir_icon_palette_id_fg);
    dir_icon.fillCircle(DIR_ICON_R, DIR_ICON_R, DIR_ICON_R - DIR_ICON_EDGE_WIDTH,
                        dir_icon_palette_id_bg);

    int x0 = DIR_ICON_R;
    int y0 = DIR_ICON_EDGE_WIDTH;
    int x1 = DIR_ICON_R + (DIR_ICON_R - DIR_ICON_EDGE_WIDTH) * cos(-M_PI_2 + DIR_ICON_ANGLE);
    int y1 = DIR_ICON_R - (DIR_ICON_R - DIR_ICON_EDGE_WIDTH) * sin(-M_PI_2 + DIR_ICON_ANGLE);
    int x2 = DIR_ICON_R - (DIR_ICON_R - DIR_ICON_EDGE_WIDTH) * cos(-M_PI_2 + DIR_ICON_ANGLE);
    int y2 = DIR_ICON_R - (DIR_ICON_R - DIR_ICON_EDGE_WIDTH) * sin(-M_PI_2 + DIR_ICON_ANGLE);

    dir_icon.fillTriangle(x0, y0, x1, y1, x2, y2, dir_icon_palette_id_fg);

    x0 = DIR_ICON_R;
    y0 = (int)(DIR_ICON_R * 1.2);
    dir_icon.fillTriangle(x0, y0, x1, y1, x2, y2, dir_icon_palette_id_bg);

    // set center of rotation
    dir_icon.setPivot(DIR_ICON_R, DIR_ICON_R);
}

void pushDirIcon()
{
    double dir_degree = gps.course.deg();
    int offset_x = curr_gps_idx_coords.idx_x - display_center_idx_coords.idx_x + M5.Display.width() / 2;
    int offset_y = curr_gps_idx_coords.idx_y - display_center_idx_coords.idx_y + M5.Display.height() / 2;

    // When dir icon is out of canvas
    if (!((-DIR_ICON_R < offset_x && offset_x < M5.Display.width() + DIR_ICON_R) &&
          (-DIR_ICON_R < offset_y && offset_y < M5.Display.height() + DIR_ICON_R)))
    {
        if (VERBOSE)
        {
            Serial.printf("pushDirIcon(): out of canvas offset=(%d,%d)\n", offset_x, offset_y);
        }

        return;
    }

    if (is_gps_active)
    {
        dir_icon.setPaletteColor(dir_icon_palette_id_fg, DIR_ICON_COLOR_ACTIVE);
    }
    else
    {
        dir_icon.setPaletteColor(dir_icon_palette_id_fg, DIR_ICON_COLOR_INACTIVE);
    }

    dir_icon.pushRotateZoomWithAA(offset_x, offset_y, dir_degree, 1, 1,
                                  dir_icon_palette_id_trans);
}

void pushButtonLabels()
{
    const int h = 8;
    const int pad = 1;

    canvas.setCursor(0, canvas.height() - h + pad);
    canvas.setTextSize(1);
    canvas.setTextColor(WHITE, BLACK);

    // BtnA
    canvas.setCursor(15, canvas.height() - h + pad);
    canvas.print(" Brightness ");

    // BtnB
    canvas.setCursor(142, canvas.height() - h + pad);
    canvas.print(" Zoom ");

    // BtnC
    if (!centering_mode)
    {
        canvas.setCursor(241, canvas.height() - h + pad);
        canvas.print(" Center ");
    }
}

void initGPSIcon()
{
    Serial.printf("initGPSIcon(): Initializing GPS icon\n");
    gps_icon.setPsram(false);
    gps_icon.createSprite(satellite_icon_png_width, satellite_icon_png_height);
    gps_icon.fillSprite(TFT_WHITE);
    gps_icon.drawPng((std::uint8_t *)satellite_icon_png, satellite_icon_png_len, 0, 0);
}

void pushInfoTopRight()
{
    // [Memo] 12 x 18 = Character size in case of `canvas.setTextSize(2);`.
    // The variable w is calculated from right to left.
    const int pad = 1;
    const int clock_width = 12 * 5; // 5 characters
    const int gps_icon_width = satellite_icon_png_width;
    const int w = pad + gps_icon_width + clock_width + pad;
    const int h = pad + 16 + pad;
    char colon = ':';

    canvas.fillRect(M5.Display.width() - w, 0, w, h, TFT_BLACK);

    // Show clock
    canvas.setCursor(M5.Display.width() - (pad + clock_width), pad);
    canvas.setTextSize(2);
    canvas.setTextColor(WHITE, BLACK);

    auto time_utc = M5.Rtc.getTime();
    colon = (time_utc.seconds % 2) ? ':' : ' ';

    canvas.printf("%02d%c%02d",
                  mod(time_utc.hours + TIMEZONE_HOUR, 24),
                  colon,
                  time_utc.minutes);

    // GPS status
    if (is_gps_active)
    {
        gps_icon.pushSprite(M5.Display.width() - (gps_icon_width + pad + clock_width + pad), pad, TFT_BLACK);
    }
}

void pushInfoTopLeft()
{
    // Character size
    // size 1: w6,  h8
    // size 2: w12, h16
    const int pad = 1;
    const int w = pad + 12 * 5 + pad + 6 * 4 + pad;
    const int h = pad + 16 + pad;

    canvas.fillRect(0, 0, w, h, TFT_BLACK);

    double speed = is_gps_active ? gps.speed.kmph() : 0.0;

    canvas.setCursor(pad, pad);
    canvas.setTextSize(2);
    canvas.setTextColor(WHITE, BLACK);
    canvas.printf("%5.1f", speed);

    canvas.setCursor(pad + 12 * 5 + pad, pad + 12 - 6);
    canvas.setTextSize(1);
    canvas.print("km/h");
}

void drawCanvas()
{
    canvas.fillSprite(NO_CACHE_COLOR);
    pushTileCache();
    pushDirIcon();
    pushInfoTopRight();
    pushInfoTopLeft();
    pushButtonLabels();
    canvas.pushSprite(0, 0);
}

void incrementBrightness()
{
    int i_brightness_new = mod(i_brightness + 1, n_brightness);

    M5.Display.setBrightness(brightness_list[i_brightness_new]);

    if (VERBOSE)
    {
        Serial.printf("incrementBrightness(): brightness %d -> %d",
                      brightness_list[i_brightness],
                      brightness_list[i_brightness_new]);
    }

    i_brightness = i_brightness_new;
}

// ================================================================================
// Zoom level
// ================================================================================
int compIntR(const void *a, const void *b) { return *(int *)a < *(int *)b; }

void initZoomList()
{
    if (VERBOSE)
    {
        Serial.println("initZoomList():");
    }

    for (int j = 0; j < LEN_ZOOM_LIST; j++)
    {
        zoom.list[j] = -1;
    }

    int i = 0;
    int num;

    // List zoom level directories
    SDFAT_FSFILE_TYPE map_dir;
    SDFAT_FSFILE_TYPE entry;
    char filename[20];
    if (map_dir.open(map_dir_path, O_RDONLY))
    {
        while (i < LEN_ZOOM_LIST)
        {
            if (!entry.openNext(&map_dir, O_RDONLY))
            {
                entry.close();
                break;
            }
            entry.getName(filename, 20);
            num = String(filename).toInt();
            if (String(filename) == String(num))
            {
                zoom.list[i] = num;
                zoom.n = i + 1;
                i++;
            }

            if (VERBOSE)
            {

                Serial.printf("  %s\n", filename);
            }

            entry.close();
        }
    }
    map_dir.close();

    qsort(zoom.list, zoom.n, sizeof(int), compIntR);

    if (VERBOSE)
    {
        Serial.printf("  zoom.n: %d\n", zoom.n);
        Serial.printf("  zoom.list: ");

        for (int j = 0; j < LEN_ZOOM_LIST; j++)
        {
            Serial.printf("%d ", zoom.list[j]);
        }
        Serial.println("");
    }

    int initial_zoom = zoom.list[LEN_ZOOM_LIST - 1];
    display_center_idx_coords.zoom = initial_zoom;
    curr_gps_idx_coords.zoom = initial_zoom;
}

int zoom_level()
{
    return zoom.list[zoom.i];
}

void increment_zoom_level()
{
    zoom.i = (zoom.i + 1) % zoom.n;
}

void changeZoomLevel()
{
    // Change zoom level index
    int zoom_old = zoom.list[zoom.i];
    increment_zoom_level();

    if (VERBOSE)
    {
        Serial.printf("changeZoomLevel(): %d->%d, ", zoom_old, zoom_level());
    }

    // Convert coordinates for the new zoom level
    st_idx_coords idx_coords_new;
    idx_coords_new.zoom = zoom_level();

    convIdxCoordsForZoom(display_center_idx_coords, idx_coords_new);
    display_center_idx_coords = idx_coords_new;

    if (VERBOSE)
    {
        Serial.printf("display_center_idx_coords (%d,%d)->",
                      display_center_idx_coords.idx_x,
                      display_center_idx_coords.idx_y);
        Serial.printf("(%d,%d)\n", idx_coords_new.idx_x, idx_coords_new.idx_y);
    }

    // Update gps_idx_coords for the new zoom level immidiately
    if (centering_mode)
    {
        curr_gps_idx_coords = display_center_idx_coords;
    }
    else
    {
        convIdxCoordsForZoom(curr_gps_idx_coords, idx_coords_new);
        curr_gps_idx_coords = idx_coords_new;
    }
}

// ================================================================================
// Sound
// ================================================================================
bool checkIfUseSound()
{
    bool ret = false;

    if (file.open("/useSound", O_RDONLY))
    {
        ret = true;
    }
    file.close();

    if (VERBOSE)
    {
        Serial.printf("checkIfUseSound(): useSound=%d\n", (int)ret);
    }

    return ret;
}

void playBoot()
{
    int sampling_rate = 16000;
    M5.Speaker.playRaw(boot_16k_u8_raw, boot_16k_u8_raw_len, sampling_rate);
}
void playGPSActive()
{
    int sampling_rate = 16000;
    M5.Speaker.playRaw(gps_active_16k_u8_raw, gps_active_16k_u8_raw_len, sampling_rate);
}

void playGPSInactive()
{
    int sampling_rate = 16000;
    M5.Speaker.playRaw(gps_inactive_16k_u8_raw, gps_inactive_16k_u8_raw_len, sampling_rate);
}

// ================================================================================
// Button & Touch
// ================================================================================
void checkTouchMoveEvent()
{
    auto t = M5.Touch.getDetail();

    auto dx = t.deltaX();
    auto dy = t.deltaY();

    if (t.isPressed() && (dx || dy)) // If moving
    {
        centering_mode = false;
        t_last_touch_move = millis();

        display_center_idx_coords.idx_x -= dx;
        display_center_idx_coords.idx_y -= dy;

        drawCanvas();

        if (VERBOSE)
        {
            Serial.printf("checkTouchMoveEvent(): display_center_idx_coords (x=%d,y=%d)", display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
            Serial.printf(" diff (x=%d,y=%d)\n", dx, dy);
        }
    }
}

void enableCentering()
{
    /*
     * Back to centering mode
     */
    if (VERBOSE)
    {
        Serial.printf("enableCentering() is called.\n");
    }

    centering_mode = true;
    display_center_idx_coords.idx_x = curr_gps_idx_coords.idx_x;
    display_center_idx_coords.idx_y = curr_gps_idx_coords.idx_y;

    updateTileCache();
    smartTileLoading();
}

void checkButtonEvents()
{
    if (M5.BtnA.wasClicked())
    {
        incrementBrightness();
    }

    if (M5.BtnB.wasClicked())
    {
        changeZoomLevel();
    }

    if (M5.BtnC.wasClicked())
    {
        if (!centering_mode)
        {
            enableCentering();
        }
    }
}

// ========================================
// M5Stack loop
// ========================================
void setup(void)
{
    // Initialization
    auto cfg = M5.config();
    M5.begin(cfg);

    M5.Display.init();
    M5.Display.setRotation(1);
    M5.Display.setBrightness(brightness_list[i_brightness]);
    M5.Display.setTextSize(2);
    M5.Display.println("Initializing");

    // Serial connection for debug
    Serial.println("Initializing");

    // Serial for gps module
    Serial2.begin(SERIAL_BAUDRATE_GPS, SERIAL_8N1, 13, 14);

    // Initialize the SD card.
    if (!sd.begin(SD_CONFIG))
    {
        sd.initErrorHalt(&Serial);
    }

    // Initializing sound
    use_sound = checkIfUseSound();
    if (use_sound)
    {
        M5.Speaker.begin();
        playBoot();
    }

    // Initialize zoom_list
    initZoomList();

    // Initializing small parts
    initGPSIcon();
    initDirIcon();

    // Set initial map variables (The tokyo station)
    initMapVariables();

    // Initialize sprites for image cache
    initCanvas();
    initTileCache();
    updateTileCache();

    Serial.println("  Initialization finished");
    Serial.println("  Waiting GPS signals...");
    t_prev = millis();
}

void loop()
{
    M5.update();
    auto t = M5.Touch.getDetail();

    if (t.isReleased())
    {
        checkGPS();

        updateTileCache();
        smartTileLoading();
        drawCanvas();
    }

    do // Smart delay
    {
        // Update button & touch screen
        M5.update();
        auto t = M5.Touch.getDetail();

        checkTouchMoveEvent();
        checkButtonEvents();

        if (t.isReleased())
        {
            smartTileLoading();

            // Feed GPS parser
            while (t.isReleased() && Serial2.available() > 0)
            {
                gps.encode(Serial2.read());
            }
        }

        t_curr = millis();
    } while (t_curr - t_prev < interval_ms);

    t_prev = t_curr;
}
