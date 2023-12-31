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
#include <M5Core2.h>
#include <math.h>
#include <TinyGPSPlus.h> // Installed through arduino IDE library manager

#define LGFX_M5STACK_CORE2
#include <LovyanGFX.hpp> // Installed through arduino IDE library manager
#include <LGFX_AUTODETECT.hpp>

#include <driver/i2s.h>
#include "sound_boot.h"
#include "sound_gps_active.h"
#include "sound_gps_inactive.h"
#include "satellite_icon.h"

// ===============================================================================
// Variables declaration
// ================================================================================
// Common
#define CYCLE_NAVI_VERSION 100
#define CYCLE_NAVI_VERSION_STRING "1.0"
#define SUCCESS 1
#define ERROR 0
#define VERBOSE 1 // Set non-zero for verbose log mode

// Settings
const unsigned long interval_sec = 1;
const float is_moved_cutoff_m = 0.7;
const char map_dir_path[] = "/map";
const char route_dir_path[] = "/route_dat";
const char point_dir_path[] = "/point_dat";
const char init_point_path[] = "/initPoint";

// Variables [GPS]
TinyGPSPlus gps;
unsigned long interval_ms = interval_sec * 1000;
unsigned long t_prev, t_curr;
bool is_gps_active = false;
const int gps_count_th = 3;
bool isUpdatedPrev = false;
int gps_active_counter = 0;

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

// Variables [map & route]
#define LEN_FILE_PATH 35
const int tile_size = 256;
const int tile_size_power = (int)round(log2(tile_size));
const int n_sprite_x = 3;
const int n_sprite_y = 3;
const int n_sprite = n_sprite_x * n_sprite_y;
static LGFX lcd;
static LGFX_Sprite canvas(&lcd); // screen buffer
LGFX_Sprite dir_icon(&canvas);
LGFX_Sprite gps_icon(&canvas);
LGFX_Sprite updating_icon(&canvas);
#define UPDATING_ICON_WIDTH 16
struct sprite_struct
{
    LGFX_Sprite *sprite;
    QueueHandle_t tile_coords;
    SemaphoreHandle_t update_required; // BinarySemaphore for sprite
};
sprite_struct *tile_cache[n_sprite];
bool centering_mode = true;
#define ASYNC_TASK_DELAY 50

#define LEN_ZOOM_LIST 20
struct st_zoom
{
    int i;
    int n;
    int list[LEN_ZOOM_LIST];
};
st_zoom zoom;

// Color name:
//   TFT_BLACK, TFT_NAVY, TFT_DARKGREEN, TFT_MAROON, TFT_PURPLE,
//   TFT_OLIVE, TFT_LIGHTGREY, TFT_DARKGREY, TFT_BLUE, TFT_GREENYELLOW,
//   TFT_GREEN, TFT_YELLOW, TFT_ORANGE, TFT_PINK, TFT_CYAN, TFT_DARKCYAN,
//   TFT_RED, TFT_MAGENTA, TFT_WHITE
#define NO_CACHE_COLOR TFT_WHITE
#define NO_IMG_COLOR TFT_LIGHTGREY
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

const int brightness_list[] = {255, 128, 64, 32};
const int n_brightness = 4;
int i_brightness = 0;

// Variables [Sound]
#define CONFIG_I2S_BCK_PIN 12
#define CONFIG_I2S_LRCK_PIN 0
#define CONFIG_I2S_DATA_PIN 2
#define CONFIG_I2S_DATA_IN_PIN 34

#define Speak_I2S_NUMBER I2S_NUM_0
#define SAMPLE_RATE 16000

#define MODE_MIC 0
#define MODE_SPK 1

bool use_sound;

// ================================================================================
// SdFat settings
// ================================================================================
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3 // Use SD_FAT_TYPE 3 for LovyanGFX
SdFs sd;
// FsFile file;
#define SDFAT_FSFILE_TYPE FsFile

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
// The max SPI clock is around 24 MHz for M5Stack Core2.
#define SPI_CLOCK SD_SCK_MHZ(24)

// Try to select the best SD card configuration.
// M5Core2 shares SPI but between SD card and the LCD display
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

int calcLonLat2IdxCoords(st_idx_coords &idx_coords, int zoom, int lon, int lat,
                         int tile_size_power)
{
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
        calcLonLat2IdxCoords(curr_gps_idx_coords, gps.location.lng(),
                             gps.location.lat(), display_center_idx_coords.zoom, tile_size_power);

        if (centering_mode)
        {
            display_center_idx_coords = curr_gps_idx_coords;
        }

        if (VERBOSE)
        {
            Serial.printf("checkGPS()\n");
            Serial.printf("  gps=(%f, %f), satellites=%d\n", gps.location.lng(), gps.location.lat(), gps.satellites.value());
            Serial.printf("  curr_gps_idx_coords=(%d,%d) ", curr_gps_idx_coords.idx_x, curr_gps_idx_coords.idx_y);
            Serial.printf("  display_center_idx_coords=(%d,%d)", display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
            Serial.println();
        }
    }
}
// ================================================================================
// Tile cache
// ================================================================================
void genMapPath(char *file_path, int z, int tile_x, int tile_y)
{
    snprintf(file_path, LEN_FILE_PATH, "%s/%d/%d/%d.jpg", map_dir_path, z, tile_x,
             tile_y);
}

void initTileCache()
{
    if (VERBOSE)
    {
        Serial.println("initTileCache():");
        Serial.printf("  heap_caps_get_free_size(MALLOC_CAP_DMA):   %8d\n",
                      heap_caps_get_free_size(MALLOC_CAP_DMA));
        Serial.printf("  heap_caps_get_free_size(MALLOC_CAP_SPIRAM):%8d\n",
                      heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }

    st_tile_coords tile_coords_tmp = {
        -1,
        -1,
        -1,
    };

    for (int i = 0; i < n_sprite; i++)
    {
        tile_cache[i] = (sprite_struct *)ps_malloc(sizeof(sprite_struct));

        tile_cache[i]->sprite = new LGFX_Sprite(&canvas);
        tile_cache[i]->sprite->setPsram(true);
        tile_cache[i]->sprite->createSprite(tile_size, tile_size);
        tile_cache[i]->sprite->fillSprite(NO_CACHE_COLOR);

        QueueHandle_t xQueue1 = xQueueCreate(1, sizeof(st_tile_coords));
        if (xQueue1 == NULL)
        {
            Serial.println("[Error] xQueueCreate() returned NULL.");
        }
        else
        {
            tile_cache[i]->tile_coords = xQueue1;
            xQueueOverwrite(tile_cache[i]->tile_coords, &tile_coords_tmp); // copied from tile_corods
        }
        tile_cache[i]->update_required = xSemaphoreCreateBinary();
        xSemaphoreGive(tile_cache[i]->update_required);                   // Initialization
        xSemaphoreTake(tile_cache[i]->update_required, pdMS_TO_TICKS(0)); // Make it 0
    }

    if (VERBOSE)
    {
        Serial.println("  tile_cache[] was allocated.");
        Serial.printf("  heap_caps_get_free_size(MALLOC_CAP_DMA):   %8d\n",
                      heap_caps_get_free_size(MALLOC_CAP_DMA));
        Serial.printf("  heap_caps_get_free_size(MALLOC_CAP_SPIRAM):%8d\n",
                      heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
}

void loadTile(LGFX_Sprite *sprite, int zoom, int tile_x, int tile_y)
{
    SDFAT_FSFILE_TYPE file;
    char file_path[LEN_FILE_PATH];
    uint32_t t;

    genMapPath(file_path, zoom, tile_x, tile_y);

    if (file.open(file_path, O_RDONLY))
    {
        if (VERBOSE)
        {
            t = millis();
        }

        sprite->drawJpgFile(sd, &file);

        if (VERBOSE)
        {
            t = millis() - t;
            Serial.printf("loadTile(): MapTile was updated in %d ms. %s\n", t, file_path);
        }
    }
    else
    {
        sprite->fillSprite(NO_IMG_COLOR);
        if (VERBOSE)
        {
            Serial.printf("loadTile(): Map was not found. %s\n", file_path);
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

int mod(int a, int b)
{
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
            idx_shifted_x = (i + shift_x) % n_sprite_x;
            idx_shifted_y = (j + shift_y) % n_sprite_y;
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
    int i_center = n_sprite_x / 2;
    int j_center = n_sprite_y / 2;
    st_tile_coords center_tile_coords;
    st_tile_coords tile_coords_curr;
    st_tile_coords tile_coords_tgt;

    convIdxCoords2TileCoords(display_center_idx_coords, center_tile_coords, tile_size);

  
    for (int i = 0; i < n_sprite; i++)
    {
        xQueuePeek(tile_cache[k]->tile_coords, &center_tile_coords, pdMS_TO_TICKS(0));
    }
    Serial.printf("%x ", tile_cache[k]->tile_coords);


    for (int i = 0; i < n_sprite_x; i++)
    {
        for (int j = 0; j < n_sprite_y; j++)
        {
            k = j + i * n_sprite_x;
            Serial.printf("i=%d,j=%d,k=%d, ", i, j, k);
            tile_coords_tgt.zoom = center_tile_coords.zoom;
            tile_coords_tgt.tile_x = center_tile_coords.tile_x + (i - i_center);
            tile_coords_tgt.tile_y = center_tile_coords.tile_y + (j - j_center);

            Serial.printf("xQueuePeek() ");
            // if (!xQueuePeek(tile_cache[k]->tile_coords, &tile_coords_curr, pdMS_TO_TICKS(0)))
            // {
            //     Serial.printf("if() ");
            //     tile_coords_curr.zoom = -1;
            //     tile_coords_curr.tile_x = -1;
            //     tile_coords_curr.tile_y = -1;
            // }

            Serial.printf("isStTileCoordsEqual() ");
            if (!isStTileCoordsEqual(tile_coords_curr, tile_coords_tgt))
            {
                // Serial.printf("%x ", tile_cache[k]->tile_coords);
                Serial.printf("%x ", &tile_coords_tgt);
                Serial.printf("xQueueOverwrite() ");
                // xQueueOverwrite(tile_cache[k]->tile_coords, &tile_coords_tgt);
                // Serial.printf("xSemaphoreGive()");
                // xSemaphoreGive(tile_cache[k]->update_required);
            }
            Serial.println("");
        }
    }
}

void pushTileCache()
{
    int is_update_required;
    int x, y;
    st_tile_coords tile_coords;

    if (VERBOSE)
    {
        Serial.println("pushTileCache():");
    }

    for (int i = 0; i < n_sprite; i++)
    {
        if (xQueuePeek(tile_cache[i]->tile_coords, &tile_coords, pdMS_TO_TICKS(0)))
        {
            x = tile_coords.tile_x * tile_size - display_center_idx_coords.idx_x + lcd.width() / 2;
            y = tile_coords.tile_y * tile_size - display_center_idx_coords.idx_y + lcd.height() / 2;

            is_update_required = uxSemaphoreGetCount(tile_cache[i]->update_required);

            if (!is_update_required)
            {
                tile_cache[i]->sprite->pushSprite(x, y);

                if (VERBOSE)
                {
                    Serial.printf("  i:%i, tile=(%d,%d), offset=(%d,%d)\n", i, tile_coords.tile_x, tile_coords.tile_y, x, y);
                }
            }
            else
            {
                tile_cache[i]->sprite->getParent()->fillRect(x, y, tile_size, tile_size, NO_CACHE_COLOR);

                if (VERBOSE)
                {
                    Serial.printf("  i:%i, Locked (updated_required=1)\n");
                }
            }
        }
        else
        {
            if (VERBOSE)
            {
                Serial.printf("  i:%i, xQueuePeek() failed.\n");
            }
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
    int i_center_sprite = n_sprite / 2;
    int tile_shift_x = 0;
    int tile_shift_y = 0;
    st_tile_coords center_tile_coords;
    st_tile_coords center_tile_coords_tgt;

    convIdxCoords2TileCoords(display_center_idx_coords, center_tile_coords_tgt, tile_size);

    if (xQueuePeek(tile_cache[i_center_sprite]->tile_coords, &center_tile_coords, pdMS_TO_TICKS(0)))
    {
        center_tile_coords = {
            -1,
            -1,
            -1,
        };
    }

    if (!isStTileCoordsEqual(center_tile_coords, center_tile_coords_tgt))
    {
        tile_shift_x = center_tile_coords.tile_x - center_tile_coords_tgt.tile_x;
        tile_shift_y = center_tile_coords.tile_y - center_tile_coords_tgt.tile_y;

        shiftTileCache(tile_shift_x, tile_shift_y);

        checkSpriteUpdateRequired();
    }

    if (VERBOSE)
    {
        Serial.printf("updateTileCache():\n");
        Serial.printf("  display_center_idx_coords (z=%d,x=%d,y=%d)\n", display_center_idx_coords.zoom, display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
        Serial.printf("  center_tile_coords     (z=%d,x=%d,y=%d)\n", center_tile_coords.zoom, center_tile_coords.tile_x, center_tile_coords.tile_y);
        Serial.printf("  center_tile_coords_tgt (z=%d,x=%d,y=%d)\n", center_tile_coords_tgt.zoom, center_tile_coords_tgt.tile_x, center_tile_coords_tgt.tile_y);
        Serial.printf("  tile_shift (x=%d,y=%d)\n", tile_shift_x, tile_shift_y);
    }
}

// ================================================================================
// Async tile loading
// ================================================================================
void initAsyncTileUpdate()
{
    xTaskCreatePinnedToCore(updateTileTask, "tileUpdateTask", 8192, NULL, 1, NULL, 1);
}

void updateTileTask(void *args)
{
    bool is_update_required;
    bool is_released;
    int zoom, tile_x, tile_y;
    st_tile_coords tile_coords;

    while (1)
    {
        for (int i = 0; i < n_sprite; i++)
        {
            is_update_required = 0 < uxSemaphoreGetCount(tile_cache[i]->update_required);
            is_released = M5.Touch.ispressed();

            if (is_update_required && is_released)
            {
                if (xQueuePeek(tile_cache[i]->tile_coords, &tile_coords, pdMS_TO_TICKS(0)))
                {
                    zoom = tile_coords.zoom;
                    tile_x = tile_coords.tile_x;
                    tile_y = tile_coords.tile_y;

                    loadTile(tile_cache[i]->sprite, zoom, tile_x, tile_y);
                    loadRoute(tile_cache[i]->sprite, zoom, tile_x, tile_y);
                    loadPoint(tile_cache[i]->sprite, zoom, tile_x, tile_y);

                    // 1 -> 0
                    if (!xSemaphoreTake(tile_cache[i]->update_required, pdMS_TO_TICKS(0)))
                    {
                        Serial.println("[ERROR] updateTileTask(): xSemaphoreTake() failed.");
                    }

                    if (VERBOSE)
                    {
                        Serial.printf("updateTileTask(): i=%d,z=%d,x=%d,y=%d", i, zoom, tile_x, tile_y);
                    }
                }
            }
        }
    }

    delay(ASYNC_TASK_DELAY);
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
    SDFAT_FSFILE_TYPE file;

    if (file.open(init_point_path, O_RDONLY))
    {
        if (VERBOSE)
        {
            Serial.println("initMapVariables(): initPoint file was detected.");
        }

        display_center_idx_coords.zoom = read4BytesAsInt(file);
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

        display_center_idx_coords.zoom = z_init;
        curr_gps_idx_coords.idx_x = idx_coords_x_init;
        curr_gps_idx_coords.idx_y = idx_coords_y_init;
    }
    file.close();

    if (VERBOSE)
    {
        Serial.printf("initMapVariables(): zoom:%d, curr_gps_idx_coords idx_x:%d, idx_y:%d\n", display_center_idx_coords.zoom, curr_gps_idx_coords.idx_x, curr_gps_idx_coords.idx_y);
    }

    display_center_idx_coords = curr_gps_idx_coords;
}

void initCanvas()
{
    canvas.setPsram(true);
    canvas.createSprite(lcd.width(), lcd.height());
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

void loadRoute(LGFX_Sprite *sprite, int zoom, int tile_x, int tile_y)
{
    int prev_point_x, prev_point_y, point_x, point_y;
    SDFAT_FSFILE_TYPE file;
    SDFAT_FSFILE_TYPE route_dat;
    char file_path[LEN_FILE_PATH];

    if (VERBOSE)
    {
        Serial.print("loadRoute(): ");
    }

    genRoutePath(file_path, zoom, tile_x, tile_y);

    if (route_dat.open(file_path, O_RDONLY))
    {
        uint64_t route_dat_size = route_dat.fileSize();
        int size_of_a_point = (sizeof(int) * 2);

        if (route_dat_size % size_of_a_point == 0 &&
            route_dat_size / size_of_a_point > 1)
        {
            if (VERBOSE)
            {
                Serial.printf("Loaded from %s\n", file_path);
            }

            if (route_dat.available())
            {
                prev_point_x = read4BytesAsInt(route_dat);
                prev_point_y = read4BytesAsInt(route_dat);
            }

            while (route_dat.available())
            {
                point_x = read4BytesAsInt(route_dat);
                point_y = read4BytesAsInt(route_dat);

                // Conditions to avoid an inapproriate line occuring when comes
                // back to the same tile
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
    route_dat.close();
}

void loadPoint(LGFX_Sprite *sprite, int zoom, int tile_x, int tile_y)
{
    int point_x, point_y;
    SDFAT_FSFILE_TYPE file;
    SDFAT_FSFILE_TYPE point_dat;
    char file_path[LEN_FILE_PATH];

    if (VERBOSE)
    {
        Serial.print("loadPoint():");
    }

    genPointPath(file_path, zoom, tile_x, tile_y);

    if (point_dat.open(file_path, O_RDONLY))
    {
        if (point_dat.fileSize() % (sizeof(int) * 2) == 0 &&
            point_dat.fileSize() / sizeof(int) / 2 > 1)
        {
            if (VERBOSE)
            {
                Serial.printf("Point dat was detected.  %s\n", file_path);
            }

            while (point_dat.available())
            {
                point_x = read4BytesAsInt(point_dat);
                point_y = read4BytesAsInt(point_dat);

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
    point_dat.close();
}

void drawLineWithStroke(LGFX_Sprite *sprite, int x_1, int y_1, int x_2, int y_2,
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

void pushDirIcon()
{
    double dir_degree = gps.course.deg();
    int offset_x = curr_gps_idx_coords.idx_x - display_center_idx_coords.idx_x + lcd.width() / 2;
    int offset_y = curr_gps_idx_coords.idx_y - display_center_idx_coords.idx_y + lcd.height() / 2;

    // When dir icon is out of canvas
    if (!((-DIR_ICON_R < offset_x && offset_x < lcd.width() + DIR_ICON_R) &&
          (-DIR_ICON_R < offset_y && offset_y < lcd.height() + DIR_ICON_R)))
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
        Serial.println("Initializing direction icon");
    }

    // Allocate sprite
    dir_icon.setColorDepth(2);
    dir_icon.createSprite(DIR_ICON_R * 2 + 1, DIR_ICON_R * 2 + 1);

    // Set palette colors
    dir_icon.setPaletteColor(dir_icon_palette_id_trans, DIR_ICON_TRANS_COLOR);
    dir_icon.setPaletteColor(dir_icon_palette_id_bg, DIR_ICON_BG_COLOR);
    dir_icon.setPaletteColor(dir_icon_palette_id_fg, DIR_ICON_COLOR_INACTIVE);

    // Draw icon
    dir_icon.fillSprite(dir_icon_palette_id_fg);
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

void pushButtonLabels()
{
    int h = 8;
    int pad = 1;

    canvas.setCursor(0, canvas.height() - h + pad);
    canvas.setTextSize(1);
    canvas.setTextColor(WHITE, BLACK);

    canvas.setCursor(15, canvas.height() - h + pad);
    canvas.print(" Brightness ");

    canvas.setCursor(142, canvas.height() - h + pad);
    canvas.print(" Zoom ");

    if (!centering_mode)
    {
        canvas.setCursor(241, canvas.height() - h + pad);
        canvas.print(" Center ");
    }
}

void initGPSIcon()
{
    Serial.println("Initializing GPS icon");
    gps_icon.setPsram(true);
    gps_icon.createSprite(satellite_icon_png_width, satellite_icon_png_height);
    gps_icon.fillSprite(TFT_WHITE);
    gps_icon.drawPng((std::uint8_t *)satellite_icon_png, satellite_icon_png_len, 0, 0);
}

void pushInfoTopRight()
{
    // [Memo] 12 x 18 = Character size in case of `canvas.setTextSize(2);`.
    // The variable w is calculated from right to left.
    int pad = 1;
    int clock_width = 12 * 5; // 5 characters
    int gps_icon_width = satellite_icon_png_width;
    int w = pad + gps_icon_width + pad + clock_width + pad;
    int h = pad + 16 + pad;

    canvas.fillRect(lcd.width() - w, 0, w, h, TFT_BLACK);

    // Show clock
    canvas.setCursor(lcd.width() - (pad + clock_width), pad);
    canvas.setTextSize(2);
    canvas.setTextColor(WHITE, BLACK);
    canvas.printf("%02d:%02d", (gps.time.hour() + 9) % 24, gps.time.minute());

    // GPS state
    if (is_gps_active)
    {
        gps_icon.pushSprite(lcd.width() - (gps_icon_width + pad + clock_width + pad), pad, TFT_BLACK);
    }
}

void pushInfoTopLeft()
{
    // Character size
    // size 1: w6,  h8
    // size 2: w12, h16
    int pad = 1;
    int w = pad + 12 * 5 + pad + 6 * 4 + pad;
    int h = pad + 16 + pad;

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
    canvas.fillSprite(NO_IMG_COLOR);
    pushTileCache();
    pushDirIcon();
    pushInfoTopRight();
    pushInfoTopLeft();
    pushButtonLabels();
    canvas.pushSprite(0, 0);
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
        Serial.print("  Zoom levels of stored on the SD: ");

        for (int j = 0; j < LEN_ZOOM_LIST; j++)
        {
            Serial.printf("%d ", zoom.list[j]);
        }
        Serial.println("");
    }

    int initial_zoom = zoom.list[LEN_ZOOM_LIST - 1];
    display_center_idx_coords.zoom = initial_zoom;
}

void changeZoomLevel()
{
    int zoom_old = zoom.list[zoom.i];
    zoom.i = (zoom.i + 1) % zoom.n;

    if (VERBOSE)
    {
        Serial.printf("changeZoomLevel(): %d->%d, ", zoom_old, zoom.list[zoom.i]);
    }

    st_idx_coords idx_coords_new;
    idx_coords_new.zoom = zoom.list[zoom.i];

    convIdxCoordsForZoom(display_center_idx_coords, idx_coords_new);

    if (VERBOSE)
    {
        Serial.printf("display_center_idx_coords (%d,%d)->", display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
        Serial.printf("(%d,%d)\n", idx_coords_new.idx_x, idx_coords_new.idx_y);
    }

    display_center_idx_coords.zoom = idx_coords_new.zoom;
    display_center_idx_coords.idx_x = idx_coords_new.idx_x;
    display_center_idx_coords.idx_y = idx_coords_new.idx_y;
}

// ================================================================================
// Sound
// ================================================================================
bool checkIfUseSound()
{
    bool ret = false;
    SDFAT_FSFILE_TYPE file;

    if (file.open("/useSound", O_RDONLY))
    {
        ret = true;
    }
    file.close();

    if (VERBOSE)
    {
        Serial.printf("checkIfUseSound(): useSound=%d", (int)ret);
    }

    return ret;
}

void InitI2SSpeakOrMic(int mode)
{
    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(Speak_I2S_NUMBER);
    i2s_config_t i2s_config = {.mode = (i2s_mode_t)(I2S_MODE_MASTER),
                               .sample_rate = SAMPLE_RATE,
                               .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
                               .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
                               .communication_format = I2S_COMM_FORMAT_STAND_I2S,
                               .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
                               .dma_buf_count = 6,
                               .dma_buf_len = 60,
                               .use_apll = false,
                               .tx_desc_auto_clear = true,
                               .fixed_mclk = 0};
    if (mode == MODE_MIC)
    {
        i2s_config.mode =
            (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    }
    else
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    }

    err += i2s_driver_install(Speak_I2S_NUMBER, &i2s_config, 0, NULL);

    // Initialization and decrlaration should be done simultaneouly.
    // The sample code from M5Stack core2 which declares and assignment
    // doesn't work.
    i2s_pin_config_t tx_pin_config = {
        .bck_io_num = CONFIG_I2S_BCK_PIN,
        .ws_io_num = CONFIG_I2S_LRCK_PIN,
        .data_out_num = CONFIG_I2S_DATA_PIN,
        .data_in_num = CONFIG_I2S_DATA_IN_PIN,
    };
    err += i2s_set_pin(Speak_I2S_NUMBER, &tx_pin_config);

    if (mode != MODE_MIC)
    {
        err += i2s_set_clk(Speak_I2S_NUMBER, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT,
                           I2S_CHANNEL_MONO);
    }
}

size_t writeSound2Speaker(const unsigned char sound_data[], int data_len)
{
    size_t bytes_written = 0;

    M5.Axp.SetSpkEnable(true);
    InitI2SSpeakOrMic(MODE_SPK);

    i2s_write(Speak_I2S_NUMBER, sound_data, data_len, &bytes_written,
              portMAX_DELAY);

    InitI2SSpeakOrMic(MODE_MIC);
    M5.Axp.SetSpkEnable(false);

    return bytes_written;
}

void playBoot()
{
    writeSound2Speaker(boot_raw, boot_raw_len);
}
void playGPSActive()
{
    writeSound2Speaker(gps_active_raw, gps_active_raw_len);
}

void playGPSInactive()
{
    writeSound2Speaker(gps_inactive_raw, gps_inactive_raw_len);
}

// ================================================================================
// Button
// ================================================================================
void vibrate(int t_ms)
{
    M5.Axp.SetLDOEnable(3, true);
    delay(t_ms);
    M5.Axp.SetLDOEnable(3, false);
}

void centeringHandler(Event &e)
{
    /*
     * Recover to centering mode
     */
    if (VERBOSE)
    {
        Serial.printf("centeringHandler() is called.\n");
    }

    centering_mode = true;
    display_center_idx_coords = curr_gps_idx_coords;

    updateTileCache();
    drawCanvas();
}

void zoomHandler(Event &e)
{
    /*
     * Change zoom level
     */
    if (VERBOSE)
    {
        Serial.printf("zoomHandler() is called.\n");
    }

    changeZoomLevel();

    updateTileCache();
    drawCanvas();
}

void moveHandler(Event &e)
{
    /*
     * Scroll tile
     */
    centering_mode = false;

    int diff_x = e.to.x - e.from.x;
    int diff_y = e.to.y - e.from.y;

    display_center_idx_coords.idx_x -= diff_x;
    display_center_idx_coords.idx_y -= diff_y;

    drawCanvas();

    if (VERBOSE)
    {
        Serial.printf("moveHandler(): display_center_idx_coords (x=%d,y=%d)", display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
        Serial.printf("  diff (x=%d,y=%d)\n", diff_x, diff_y);
    }
}

void wasReleasedHandler(Event &e)
{
    /*
     * Update tiles
     */
    if (VERBOSE)
    {
        Serial.println("wasReleasedHandler() is called.");
    }
    updateTileCache();
    drawCanvas();
}

void toggleBrightnessHandler(Event &e)
{
    if (VERBOSE)
    {
        Serial.printf("toggleBrightnessHandler(): %d", brightness_list[i_brightness]);
    }

    i_brightness = (i_brightness + 1) % n_brightness;
    lcd.setBrightness(brightness_list[i_brightness]);

    if (VERBOSE)
    {
        Serial.printf("->%d", brightness_list[i_brightness]);
    }
}

void setupButtonHandlers()
{
    M5.background.delHandlers();

    M5.background.addHandler(moveHandler, E_MOVE);
    // M5.background.addHandler(wasReleasedHandler, E_RELEASE);
    M5.BtnA.addHandler(toggleBrightnessHandler, E_RELEASE);
    M5.BtnB.addHandler(zoomHandler, E_RELEASE);
    M5.BtnC.addHandler(centeringHandler, E_RELEASE);
}

// ========================================
// M5Stack loop
// ========================================
void setup(void)
{
    // Initialization
    // M5.begin(bool LCDEnable = true, bool SDEnable = true, bool SerialEnable = true, bool I2CEnable = false, mbus_mode_t mode = kMBusModeOutput);
    // kMBusModeOutput: powered by USB or Battery
    // KMBusModeInput: powered by 5-12V external
    M5.begin(true, false, true, false, kMBusModeOutput);
    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(brightness_list[i_brightness]);

    lcd.setTextSize(2);
    lcd.println("Initializing");

    // Serial connection for debug
    Serial.println("Initializing");

    // Serial connection to GPS module
    Serial2.begin(9600, SERIAL_8N1, 13, 14);

    // Initialize the SD card.
    if (!sd.begin(SD_CONFIG))
    {
        sd.initErrorHalt(&Serial);
    }

    // Initializing sound
    use_sound = checkIfUseSound();
    if (use_sound)
    {
        playBoot();
    }

    // Initialize zoom_list
    initZoomList();

    // Initialize sprites for image cache
    initCanvas();
    initTileCache();

    // Initializing small parts
    initGPSIcon();
    initDirIcon();
    // initUpdatingIcon();

    // Initializing button handlers
    setupButtonHandlers();

    // Initialize async tasks
    // initAsyncTileUpdate();

    // Set initial map variables (The tokyo station)
    initMapVariables();

    updateTileCache();

    Serial.println("Waiting GPS signals...");
    t_prev = millis();
}

void loop()
{
    checkGPS();

    updateTileCache();
    drawCanvas();

    do // Smart delay
    {
        // Update button & touch screen
        M5.update();

        // Feed GPS parser
        while (Serial2.available() > 0)
        {
            gps.encode(Serial2.read());
        }
        t_curr = millis();
    } while (t_curr - t_prev < interval_ms);
    t_prev = t_curr;
}
