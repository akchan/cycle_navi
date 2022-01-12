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
 */

#include <math.h>
#include <M5Core2.h>
#include <LovyanGFX.hpp> // Installed through arduino IDE library manager
#include <TinyGPSPlus.h> // Installed through arduino IDE library manager

#include <driver/i2s.h>
#include "sound_boot.h"
#include "sound_gps_active.h"
#include "sound_gps_inactive.h"
#include "satellite_icon.h"

// Settings
const unsigned long interval_sec = 1;
const float is_moved_cutoff_m = 0.7;
const char map_dir_path[] = "/map";
const char route_dir_path[] = "/route_dat";
const char point_dir_path[] = "/point_dat";

// Common
#define SUCCESS 1
#define ERROR 0

// Variables [GPS]
TinyGPSPlus gps;
unsigned long interval_ms = interval_sec * 1000;
unsigned long t_prev, t_curr;
bool is_gps_active = false;
const int gps_count_th = 3;
int gps_count = 0;

struct st_tile_coords
{
    int zoom;
    int tile_x;
    int tile_y;
};
struct st_idx_on_tile
{
    int idx_x;
    int idx_y;
};
struct st_idx_coords
{
    int idx_x;
    int idx_y;
};
// Initial point (Tokyo station)
int z_init = 14, idx_coords_x_init = 14552 * 256 + 218, idx_coords_y_init = 6451 * 256 + 165;
st_idx_coords curr_gps_idx_coords = {0, 0};

// Variables [map & route]
// Color name:
//   TFT_BLACK, TFT_NAVY, TFT_DARKGREEN, TFT_MAROON, TFT_PURPLE,
//   TFT_OLIVE, TFT_LIGHTGREY, TFT_DARKGREY, TFT_BLUE, TFT_GREENYELLOW,
//   TFT_GREEN, TFT_YELLOW, TFT_ORANGE, TFT_PINK, TFT_CYAN, TFT_DARKCYAN,
//   TFT_RED, TFT_MAGENTA, TFT_WHITE
#define NO_TILE_COLOR TFT_LIGHTGREY
#define LEN_FILE_PATH 35
char file_path[LEN_FILE_PATH];
const int tile_size = 256;
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
    int zoom;
    int tile_x;
    int tile_y;
    LGFX_Sprite *sprite;
    SemaphoreHandle_t mutex;
};
sprite_struct *tile_cache[n_sprite];
st_idx_coords display_center_idx_coords = {0, 0};
bool centering_mode = true;

#define LEN_ZOOM_LIST 20
int zoom_prev = -1, tile_x_prev = -1, tile_y_prev = -1;
int zoom;
int zoom_list[LEN_ZOOM_LIST];
int n_zoom = -1;
int zoom_list_i = 0;

#define ROUTE_COLOR TFT_BLUE
#define POINT_COLOR TFT_DARKGREEN
#define ROUTE_WIDTH 7
#define POINT_R 10

#define DIR_ICON_COLOR_ACTIVE TFT_BLUE
#define DIR_ICON_COLOR_INACTIVE TFT_DARKGREY
#define DIR_ICON_BG_COLOR TFT_WHITE
#define DIR_ICON_TRANS_COLOR TFT_BLACK
#define DIR_ICON_R 14
#define DIR_ICON_WIDTH 3
#define DIR_ICON_ANGLE 45.0 / 180.0 * M_PI

int updating_icon_n_line = 8;
int i_shift_updating_icon = 0;

const int brightness_list[] = {255, 128, 64, 32};
const int n_brightness = 4;
int i_brightness = 0;

struct st_updateTileQueueData
{
    sprite_struct *p_sprite_struct;
    int zoom;
    int tile_x;
    int tile_y;
};
QueueHandle_t update_tile_queue;

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
// GPS
// ================================================================================
int calcCoords2TileCoords(st_tile_coords &tile_coords, int zoom, double lon,
                          double lat)
{
    int method = 2;
    // 0: tan
    // 1: sin
    // 2: sin_cos
    // 3: cos_tan

    if (lat <= 90.0 && lat >= -90.0)
    {
        // valid latitude case
        double r = 1.0 / (2 * M_PI);
        double x_raw = 2 * M_PI * r * (lon + 180.0) / 360.0;

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
        } else if (method == 3)
        {
            rad = lat / 180.0 * M_PI;
            y_raw = r * log(1.0 / cos(rad) + tan(rad));
        }
        
        tile_coords.zoom = zoom;
        tile_coords.tile_x = (int)floor(x_raw * (1 << zoom));
        tile_coords.tile_y = (int)floor((0.5 - y_raw) * (1 << zoom));

        return SUCCESS;
    }
    else
    {
        // Invalid latitude case
        tile_coords.zoom = zoom;
        tile_coords.tile_x = -1;
        tile_coords.tile_y = -1;

        return ERROR;
    }
}

int calcIdxOnTile(st_idx_on_tile &idx_on_tile, int zoom, int lon, int lat,
                  int tile_size)
{
    st_tile_coords tile_coords1, tile_coords2;
    int ret;

    ret = calcCoords2TileCoords(tile_coords1, zoom, lon, lat);
    if (ret == ERROR)
        return ERROR;

    if (tile_size <= 0)
        return ERROR;
    int zoom2 = zoom + (int)log2(tile_size);

    ret = calcCoords2TileCoords(tile_coords2, zoom2, lon, lat);
    if (ret == ERROR)
        return ERROR;

    idx_on_tile.idx_x = tile_coords2.tile_x - tile_coords1.tile_x * tile_size;
    idx_on_tile.idx_y = tile_coords2.tile_y - tile_coords1.tile_y * tile_size;

    return SUCCESS;
}

void calcCoordsIdx2Tile(st_tile_coords &tile_coords, st_idx_coords &idx_coords,
                        int tile_size)
{
    tile_coords.tile_x = idx_coords.idx_x / tile_size;
    tile_coords.tile_y = idx_coords.idx_y / tile_size;
}

void calcCoordsIdx2IdxOnTile(st_idx_on_tile &idx_on_tile,
                             st_idx_coords &idx_coords, int tile_size)
{
    idx_on_tile.idx_x = idx_coords.idx_x % tile_size;
    idx_on_tile.idx_y = idx_coords.idx_y % tile_size;
}

int calcCoords2CoordsIdx(st_idx_coords &idx_coords, double lon, double lat,
                         int zoom, int tile_size)
{
    st_tile_coords tile_coords;

    if (tile_size <= 0)
        return ERROR;
    int zoom2 = zoom + (int)log2(tile_size);

    int ret = calcCoords2TileCoords(tile_coords, zoom2, lon, lat);
    if (ret == ERROR)
        return ERROR;

    idx_coords.idx_x = tile_coords.tile_x;
    idx_coords.idx_y = tile_coords.tile_y;

    return SUCCESS;
}

// ================================================================================
// Tile cache
// ================================================================================
void genMapPath(char *file_path, int z, int tile_x, int tile_y)
{
    snprintf(file_path, LEN_FILE_PATH, "%s/%d/%d/%d.jpg", map_dir_path, z, tile_x,
             tile_y);
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

void loadTile(LGFX_Sprite *sprite, int zoom, int tile_x, int tile_y)
{
    genMapPath(file_path, zoom, tile_x, tile_y);
    File fp = SD.open(file_path, FILE_READ);
    if (fp)
    {
        uint32_t t = millis();

        sprite->drawJpgFile(SD, file_path);

        t = millis() - t;
        Serial.printf("MapTile was updated. (%d ms) %s\n", t, file_path);
    }
    else
    {
        sprite->fillSprite(NO_TILE_COLOR);
        Serial.print("Map was not found.\n");
        Serial.println(file_path);
    }
    fp.close();
}

void drawLineWithStroke(LGFX_Sprite *sprite, int x_1, int y_1, int x_2, int y_2,
                        int stroke, uint16_t color)
{
    int center_i = stroke / 2;
    int r = (stroke - 1) / 2;
    int x_shift, y_shift;

    double angle_rad = atan((double)(y_2 - y_1) / (x_2 - x_1));

    // sprite->setColor(color);

    // when line is nearly vertical
    if (angle_rad >= M_PI / 4.0 || angle_rad <= -M_PI / 4.0)
    {
        for (int i_x = 0; i_x < stroke; i_x++)
        {
            x_shift = i_x - center_i;
            sprite->drawLine(x_1 + x_shift, y_1, x_2 + x_shift, y_2, color);
        }
    }
    // when line is nearly horizontal
    else
    {
        for (int i_y = 0; i_y < stroke; i_y++)
        {
            y_shift = i_y - center_i;
            sprite->drawLine(x_1, y_1 + y_shift, x_2, y_2 + y_shift, color);
        }
    }

    sprite->fillCircle(x_1, y_1, r, color);
    sprite->fillCircle(x_2, y_2, r, color);
}

int read4BitsAsInt(File f)
{
    int ret;

    f.read((uint8_t *)&ret, sizeof(int));

    return ret;
}

void loadRoute(LGFX_Sprite *sprite, int zoom, int tile_x, int tile_y)
{
    int prev_point_x, prev_point_y, point_x, point_y;

    genRoutePath(file_path, zoom, tile_x, tile_y);
    File route_dat = SD.open(file_path, FILE_READ);
    if (route_dat)
    {
        if (route_dat.size() % (sizeof(int) * 2) == 0 &&
            route_dat.size() / sizeof(int) / 2 > 1)
        {
            Serial.printf("Route dat was detected.  %s\n", file_path);

            if (route_dat.available())
            {
                prev_point_x = read4BitsAsInt(route_dat);
                prev_point_y = read4BitsAsInt(route_dat);
            }

            while (route_dat.available())
            {
                point_x = read4BitsAsInt(route_dat);
                point_y = read4BitsAsInt(route_dat);

                drawLineWithStroke(sprite, prev_point_x, prev_point_y, point_x, point_y,
                                   ROUTE_WIDTH, ROUTE_COLOR);

                prev_point_x = point_x;
                prev_point_y = point_y;
            }
        }
        else
        {
            Serial.printf("Invalid route dat:  %s\n", file_path);
        }
    }
    else
    {
        Serial.printf("Route dat was not found.  %s\n", file_path);
    }
    route_dat.close();
}

void loadPoint(LGFX_Sprite *sprite, int zoom, int tile_x, int tile_y)
{
    int point_x, point_y;

    genPointPath(file_path, zoom, tile_x, tile_y);
    File point_dat = SD.open(file_path, FILE_READ);
    if (point_dat)
    {
        if (point_dat.size() % (sizeof(int) * 2) == 0 &&
            point_dat.size() / sizeof(int) / 2 > 1)
        {
            Serial.printf("Point dat was detected.  %s\n", file_path);

            while (point_dat.available())
            {
                point_x = read4BitsAsInt(point_dat);
                point_y = read4BitsAsInt(point_dat);

                sprite->fillCircle(point_x, point_y, POINT_R, POINT_COLOR);
            }
        }
        else
        {
            Serial.printf("Invalid point dat:  %s\n", file_path);
        }
    }
    else
    {
        Serial.printf("Point dat was not found.  %s\n", file_path);
    }
    point_dat.close();
}

void shiftTileCacheLeft(sprite_struct *tile_cache[], int n_sprite_x,
                        int n_sprite_y)
{
    sprite_struct *ptr_tmp;

    xQueueReset(update_tile_queue);

    for (int i_y = 0; i_y < n_sprite_y; i_y++)
    {
        ptr_tmp = tile_cache[n_sprite_x * i_y];

        for (int i_x = 0; i_x < n_sprite_x - 1; i_x++)
        {
            tile_cache[n_sprite_x * i_y + i_x] =
                tile_cache[n_sprite_x * i_y + i_x + 1];
        }

        tile_cache[n_sprite_x * i_y + n_sprite_x - 1] = ptr_tmp;

        if (xSemaphoreTake(ptr_tmp->mutex, pdMS_TO_TICKS(0)))
        {
            ptr_tmp->sprite->fillSprite(NO_TILE_COLOR);

            // unlock the tile
            if (xSemaphoreGive(ptr_tmp->mutex) != pdTRUE)
            {
                Serial.printf("shiftTileCacheLeft(): Error in xSemaphoreGive() i=%d, ptr_tmp=%d\n", n_sprite_x * i_y + n_sprite_x - 1, ptr_tmp);
            }
        }
        else
        {
            Serial.println("shiftTileCacheLeft(): passed because the tile was locked.");
        }
    }
}

void shiftTileCacheRight(sprite_struct *tile_cache[], int n_sprite_x,
                         int n_sprite_y)
{
    sprite_struct *ptr_tmp;

    xQueueReset(update_tile_queue);

    for (int i_y = 0; i_y < n_sprite_y; i_y++)
    {
        ptr_tmp = tile_cache[n_sprite_x * i_y + n_sprite_x - 1];

        for (int i_x = n_sprite_x - 1; i_x > 0; i_x--)
        {
            tile_cache[n_sprite_x * i_y + i_x] =
                tile_cache[n_sprite_x * i_y + i_x - 1];
        }

        tile_cache[n_sprite_x * i_y] = ptr_tmp;

        if (xSemaphoreTake(ptr_tmp->mutex, pdMS_TO_TICKS(0)))
        {
            ptr_tmp->sprite->fillSprite(NO_TILE_COLOR);

            // unlock the tile
            if (xSemaphoreGive(ptr_tmp->mutex) != pdTRUE)
            {
                Serial.printf("shiftTileCacheRight(): Error in xSemaphoreGive() i=%d, ptr_tmp=%d\n", n_sprite_x * i_y, ptr_tmp);
            }
        }
        else
        {
            Serial.println("shiftTileCacheRight(): passed because the tile was locked.");
        }
    }
}

void shiftTileCacheX(sprite_struct *tile_cache[], int n_shift_x, int n_sprite_x,
                     int n_sprite_y)
{
    if (n_shift_x > 0)
    {
        for (int i = 0; i < n_shift_x; i++)
        {
            shiftTileCacheRight(tile_cache, n_sprite_x, n_sprite_y);
        }
    }
    else if (n_shift_x < 0)
    {
        for (int i = 0; i < abs(n_shift_x); i++)
        {
            shiftTileCacheLeft(tile_cache, n_sprite_x, n_sprite_y);
        }
    }
}

void shiftTileCacheUp(sprite_struct *tile_cache[], int n_sprite_x,
                      int n_sprite_y)
{
    sprite_struct *ptr_tmp;

    xQueueReset(update_tile_queue);

    for (int i_x = 0; i_x < n_sprite_x; i_x++)
    {
        ptr_tmp = tile_cache[i_x];

        for (int i_y = 0; i_y < n_sprite_y - 1; i_y++)
        {
            tile_cache[n_sprite_x * i_y + i_x] =
                tile_cache[n_sprite_x * (i_y + 1) + i_x];
        }

        tile_cache[n_sprite_x * (n_sprite_y - 1) + i_x] = ptr_tmp;

        if (xSemaphoreTake(ptr_tmp->mutex, pdMS_TO_TICKS(0)))
        {
            ptr_tmp->sprite->fillSprite(NO_TILE_COLOR);

            // unlock the tile
            if (xSemaphoreGive(ptr_tmp->mutex) != pdTRUE)
            {
                Serial.printf("shiftTileCacheUp(): Error in xSemaphoreGive() i=%d, ptr_tmp=%d\n", n_sprite_x * (n_sprite_y - 1) + i_x, ptr_tmp);
            }
        }
        else
        {
            Serial.println("shiftTileCacheUp(): passed because the tile was locked.");
        }
    }
}

void shiftTileCacheDown(sprite_struct *tile_cache[], int n_sprite_x,
                        int n_sprite_y)
{
    sprite_struct *ptr_tmp;

    xQueueReset(update_tile_queue);

    for (int i_x = 0; i_x < n_sprite_x; i_x++)
    {
        ptr_tmp = tile_cache[n_sprite_x * (n_sprite_y - 1) + i_x];

        for (int i_y = n_sprite_y - 1; i_y > 0; i_y--)
        {
            tile_cache[n_sprite_x * i_y + i_x] =
                tile_cache[n_sprite_x * (i_y - 1) + i_x];
        }

        tile_cache[i_x] = ptr_tmp;

        if (xSemaphoreTake(ptr_tmp->mutex, pdMS_TO_TICKS(0)))
        {
            ptr_tmp->sprite->fillSprite(NO_TILE_COLOR);

            // unlock the tile
            if (xSemaphoreGive(ptr_tmp->mutex) != pdTRUE)
            {
                Serial.printf("shiftTileCacheDown(): Error in xSemaphoreGive() i=%d, ptr_tmp=%d\n", i_x, ptr_tmp);
            }
        }
        else
        {
            Serial.println("shiftTileCacheDown(): passed because the tile was locked.");
        }
    }
}

void shiftTileCacheY(sprite_struct *tile_cache[], int n_shift_y, int n_sprite_x,
                     int n_sprite_y)
{
    if (n_shift_y > 0)
    {
        for (int i = 0; i < n_shift_y; i++)
        {
            shiftTileCacheDown(tile_cache, n_sprite_x, n_sprite_y);
        }
    }
    else if (n_shift_y < 0)
    {
        for (int i = 0; i < abs(n_shift_y); i++)
        {
            shiftTileCacheUp(tile_cache, n_sprite_x, n_sprite_y);
        }
    }
}

void updateTileCache(sprite_struct *tile_cache[],
                     int zoom, st_idx_coords &idx_coords,
                     int n_sprite_x, int n_sprite_y,
                     bool reset_gps_state_cursor = true)
{
    int i, tile_x, tile_y;
    bool is_first_tile_loading = true;
    st_updateTileQueueData queueData, queueDataTmp;

    st_tile_coords tile_coords;
    calcCoordsIdx2Tile(tile_coords, idx_coords, tile_size);

    int center_tile_x = tile_coords.tile_x;
    int center_tile_y = tile_coords.tile_y;

    int i_center = n_sprite_x * n_sprite_y / 2;
    int center_cache_tile_x = tile_cache[i_center]->tile_x;
    int center_cache_tile_y = tile_cache[i_center]->tile_y;

    int tile_shift_x = center_cache_tile_x - center_tile_x;
    int tile_shift_y = center_cache_tile_y - center_tile_y;

    if (abs(tile_shift_x) < n_sprite_x)
    {
        shiftTileCacheX(tile_cache, tile_shift_x, n_sprite_x,
                        n_sprite_y);
    }
    if (abs(tile_shift_y) < n_sprite_y)
    {
        shiftTileCacheY(tile_cache, tile_shift_y, n_sprite_x,
                        n_sprite_y);
    }

    Serial.printf("updateTileCache()\n");
    Serial.printf("  idx_coords=(%d,%d), center_tile=(%d,%d), tile_shift=(%d,%d)\n", idx_coords.idx_x, idx_coords.idx_y, center_tile_x, center_tile_y, tile_shift_x, tile_shift_y);

    Serial.printf("  i,zoom,tile_x[target],tile_y[target] = \n");
    for (int i_x = 0; i_x < n_sprite_x; i_x++)
    {
        for (int i_y = 0; i_y < n_sprite_y; i_y++)
        {
            i = n_sprite_x * i_y + i_x;
            tile_x = center_tile_x + i_x - n_sprite_x / 2;
            tile_y = center_tile_y + i_y - n_sprite_y / 2;

            Serial.printf("    tile (%d,%d,%d[%d],%d[%d]) -> ", i, zoom, tile_cache[i]->tile_x, tile_x, tile_cache[i]->tile_y, tile_y);

            if (!(tile_cache[i]->zoom == zoom && tile_cache[i]->tile_x == tile_x &&
                  tile_cache[i]->tile_y == tile_y))
            {
                Serial.print("changed, update required. ");

                queueData.p_sprite_struct = tile_cache[i];
                queueData.zoom = zoom;
                queueData.tile_x = tile_x;
                queueData.tile_y = tile_y;

                if (uxQueueSpacesAvailable(update_tile_queue))
                {
                    if (xQueueSend(update_tile_queue, &queueData, 0) == errQUEUE_FULL)
                    {
                        Serial.print("faild to send a queue! The queue is full.\n");
                    }
                    else
                    {
                        Serial.print("sent a queue\n");
                    }
                }
            }
            else
            {
                Serial.print("no change, nothing to do.\n");
            }
        }
    }
}

void pushTileCache(sprite_struct *tile_cache[], st_idx_coords &idx_coords)
{
    int i, offset_x, offset_y;

    Serial.printf("pushTileCache() idx_coords=(%d,%d) which is tile=(%d,%d), idx_on_tile=(%d,%d)\n", idx_coords.idx_x, idx_coords.idx_y, idx_coords.idx_x / tile_size, idx_coords.idx_y / tile_size, idx_coords.idx_x % tile_size, idx_coords.idx_y % tile_size);

    for (int i_y = 0; i_y < n_sprite_y; i_y++)
    {
        for (int i_x = 0; i_x < n_sprite_x; i_x++)
        {
            i = n_sprite_x * i_y + i_x;

            offset_x = tile_cache[i]->tile_x * tile_size - idx_coords.idx_x + lcd.width() / 2;
            offset_y = tile_cache[i]->tile_y * tile_size - idx_coords.idx_y + lcd.height() / 2;

            Serial.printf("  i:%i, tile=(%d,%d), offset=(%d,%d)\n", i, tile_cache[i]->tile_x, tile_cache[i]->tile_y, offset_x, offset_y);

            if (xSemaphoreTake(tile_cache[i]->mutex, pdMS_TO_TICKS(0)))
            {
                
                tile_cache[i]->sprite->pushSprite(offset_x, offset_y);

                // unlock the tile
                if (xSemaphoreGive(tile_cache[i]->mutex) != pdTRUE)
                {
                    Serial.printf("pushTileCache(): Error in xSemaphoreGive() i=%d, zoom=%d, tile=(%d,%d)\n", i, zoom, tile_cache[i]->tile_x, tile_cache[i]->tile_y);
                }
            }
            else
            {
                Serial.printf("pushTileCache(): passed because the tile was locked. i=%d, zoom=%d, tile=(%d,%d)\n", i, zoom, tile_cache[i]->tile_x, tile_cache[i]->tile_y);
            }
        }
    }
}

void initTileCache()
{
    Serial.printf("heap_caps_get_free_size(MALLOC_CAP_DMA):   %8d\n",
                  heap_caps_get_free_size(MALLOC_CAP_DMA));
    Serial.printf("heap_caps_get_free_size(MALLOC_CAP_SPIRAM):%8d\n",
                  heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    canvas.setPsram(true);
    canvas.createSprite(lcd.width(), lcd.height());
    canvas.fillSprite(NO_TILE_COLOR);

    for (int i = 0; i < n_sprite; i++)
    {
        tile_cache[i] = (sprite_struct *)ps_malloc(sizeof(sprite_struct));

        tile_cache[i]->zoom = 0;
        tile_cache[i]->tile_x = 0;
        tile_cache[i]->tile_y = 0;

        tile_cache[i]->sprite = new LGFX_Sprite(&canvas);
        tile_cache[i]->sprite->setPsram(true);
        tile_cache[i]->sprite->createSprite(tile_size, tile_size);
        tile_cache[i]->sprite->fillSprite(NO_TILE_COLOR);

        tile_cache[i]->mutex = xSemaphoreCreateMutex();

        // release mutex
        xSemaphoreGive(tile_cache[i]->mutex);
    }

    Serial.println("tile_cache was allocated.");
    Serial.printf("heap_caps_get_free_size(MALLOC_CAP_DMA):   %8d\n",
                  heap_caps_get_free_size(MALLOC_CAP_DMA));
    Serial.printf("heap_caps_get_free_size(MALLOC_CAP_SPIRAM):%8d\n",
                  heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void pushDirIcon(double dir_degree,
                 st_idx_coords &dir_icon_coords,
                 st_idx_coords &display_center_idx_coords,
                 bool is_active)
{
    int offset_x = dir_icon_coords.idx_x - display_center_idx_coords.idx_x + lcd.width() / 2;
    int offset_y = dir_icon_coords.idx_y - display_center_idx_coords.idx_y + lcd.height() / 2;

    // When dir icon is out of canvas
    if (!((-DIR_ICON_R < offset_x && offset_x < lcd.width() + DIR_ICON_R) &&
          (-DIR_ICON_R < offset_y && offset_y < lcd.height() + DIR_ICON_R)))
    {
        Serial.printf(
            "Direction icon is out of canvas. offset=(%d,%d)\n", offset_x, offset_y);
        return;
    }

    uint16_t dir_icon_color = is_active ? DIR_ICON_COLOR_ACTIVE : DIR_ICON_COLOR_INACTIVE;

    dir_icon.fillSprite(DIR_ICON_TRANS_COLOR);
    dir_icon.fillCircle(DIR_ICON_R, DIR_ICON_R, DIR_ICON_R, dir_icon_color);
    dir_icon.fillCircle(DIR_ICON_R, DIR_ICON_R, DIR_ICON_R - DIR_ICON_WIDTH,
                        DIR_ICON_BG_COLOR);

    int x0 = DIR_ICON_R;
    int y0 = DIR_ICON_WIDTH;
    int x1 = DIR_ICON_R + (DIR_ICON_R - DIR_ICON_WIDTH) * cos(-M_PI_2 + DIR_ICON_ANGLE);
    int y1 = DIR_ICON_R - (DIR_ICON_R - DIR_ICON_WIDTH) * sin(-M_PI_2 + DIR_ICON_ANGLE);
    int x2 = DIR_ICON_R - (DIR_ICON_R - DIR_ICON_WIDTH) * cos(-M_PI_2 + DIR_ICON_ANGLE);
    int y2 = DIR_ICON_R - (DIR_ICON_R - DIR_ICON_WIDTH) * sin(-M_PI_2 + DIR_ICON_ANGLE);

    dir_icon.fillTriangle(x0, y0, x1, y1, x2, y2, dir_icon_color);

    x0 = DIR_ICON_R;
    y0 = (int)(DIR_ICON_R * 1.2);
    dir_icon.fillTriangle(x0, y0, x1, y1, x2, y2, DIR_ICON_BG_COLOR);

    dir_icon.setPivot(DIR_ICON_R, DIR_ICON_R); // set center of rotation
    dir_icon.pushRotateZoomWithAA(offset_x, offset_y, dir_degree, 1, 1,
                                  DIR_ICON_TRANS_COLOR);
}

void initDirIcon()
{
    Serial.println("Initializing direction icon");
    dir_icon.createSprite(DIR_ICON_R * 2 + 1, DIR_ICON_R * 2 + 1);
}

void pushButtonLabels()
{
    int h = 8;

    canvas.setCursor(0, canvas.height() - h + 1);
    canvas.setTextSize(1);
    canvas.setTextColor(WHITE, BLACK);

    canvas.setCursor(15, canvas.height() - h + 1);
    canvas.print(" Brightness ");

    canvas.setCursor(142, canvas.height() - h + 1);
    canvas.print(" Zoom ");

    if (!centering_mode)
    {
        canvas.setCursor(241, canvas.height() - h + 1);
        canvas.print(" Center ");
    }
}

void initUpdatingIcon()
{
    Serial.println("Initializing updating icon");

    double r1 = UPDATING_ICON_WIDTH * 0.5 * 0.9;
    double r2 = UPDATING_ICON_WIDTH * 0.5 * 0.4;
    int line_stroke = 2;
    uint8_t color, color1 = 32, color2 = 255;
    int x1, y1, x2, y2;

    updating_icon.createSprite(UPDATING_ICON_WIDTH, UPDATING_ICON_WIDTH);
    updating_icon.fillSprite(TFT_BLACK);

    for (int i = 0; i < updating_icon_n_line; i++)
    {
        x1 = UPDATING_ICON_WIDTH / 2 + r1 * cos((double)i / updating_icon_n_line * 2 * M_PI);
        y1 = UPDATING_ICON_WIDTH / 2 + r1 * sin((double)i / updating_icon_n_line * 2 * M_PI);
        x2 = UPDATING_ICON_WIDTH / 2 + r2 * cos((double)i / updating_icon_n_line * 2 * M_PI);
        y2 = UPDATING_ICON_WIDTH / 2 + r2 * sin((double)i / updating_icon_n_line * 2 * M_PI);
        color = color1 + i * (double)(color2 - color1) / updating_icon_n_line;

        drawLineWithStroke(&updating_icon, x1, y1, x2, y2, line_stroke, lcd.color565(color, color, color));
    }
}

void initGPSIcon()
{
    Serial.println("Initializing GPS icon");
    gps_icon.setPsram(true);
    gps_icon.createSprite(satellite_icon_png_width, satellite_icon_png_height);
    gps_icon.fillSprite(WHITE);
    gps_icon.drawPng((std::uint8_t *)satellite_icon_png, satellite_icon_png_len, 0, 0);
}

void pushInfoTopRight()
{
    // [Memo] 12 x 18 = Character size in case of `canvas.setTextSize(2);`.
    // The variable w is calculated from right to left.
    int pad = 1;
    int w = pad + 12 * 5 + pad + 16 + pad + 16 + pad;
    int h = pad + 16 + pad;

    canvas.fillRect(lcd.width() - w, 0, w, h, TFT_BLACK);

    // Show clock
    canvas.setCursor(lcd.width() - (pad + 12 * 5), pad);
    canvas.setTextSize(2);
    canvas.setTextColor(WHITE, BLACK);
    canvas.printf("%02d:%02d", (gps.time.hour() + 9) % 24, gps.time.minute());

    // GPS state
    if (is_gps_active)
    {
        gps_icon.pushSprite(lcd.width() - (pad + 12 * 5 + pad + 16), pad, TFT_BLACK);
    }

    // Tile cache updating state
    if ((int)uxQueueSpacesAvailable(update_tile_queue) < n_sprite)
    {
        // if queue is not empty
        int offset_x = lcd.width() - (pad + 12 * 5 + pad + 16 + pad + UPDATING_ICON_WIDTH / 2);
        int offset_y = pad + UPDATING_ICON_WIDTH / 2;

        // set center of rotation
        updating_icon.setPivot(UPDATING_ICON_WIDTH / 2, UPDATING_ICON_WIDTH / 2);
        updating_icon.pushRotateZoomWithAA(offset_x, offset_y, i_shift_updating_icon * 360.0 / updating_icon_n_line, 1, 1,
                                           DIR_ICON_TRANS_COLOR);
        i_shift_updating_icon++;
    }
    else
    {
        i_shift_updating_icon = 0;
    }
}

void drawCanvas(sprite_struct *tile_cache[], st_idx_coords &display_center_idx_coords)
{
    canvas.fillSprite(NO_TILE_COLOR);
    pushTileCache(tile_cache, display_center_idx_coords);
    pushDirIcon(gps.course.deg(), curr_gps_idx_coords, display_center_idx_coords, is_gps_active);
    pushInfoTopRight();
    pushButtonLabels();
    canvas.pushSprite(0, 0);
}

// ================================================================================
// Async tile loading
// ================================================================================
void initUpdateTileQueue()
{
    update_tile_queue = xQueueCreate(n_sprite, sizeof(st_updateTileQueueData));

    if (update_tile_queue == NULL)
    {
        Serial.println("Failed in initializing update_tile_queue!");
    }
    else
    {
        Serial.println("update_tile_queue was initialized.");
    }
}

void updateTileTask(void *arg)
{
    QueueHandle_t xQueue;
    BaseType_t xStatus;
    st_updateTileQueueData queueData;
    sprite_struct *p_sprite_struct;
    int zoom, tile_x, tile_y;
    const TickType_t xTicksToWait = 50U; // about 100ms

    while (1)
    {
        // loop in xTicksToWait and
        // when data is received, this func is triggerred
        xStatus = xQueueReceive(update_tile_queue, &queueData, xTicksToWait);

        if (xStatus == pdPASS) // successful receiving
        {
            p_sprite_struct = queueData.p_sprite_struct;
            zoom = queueData.zoom;
            tile_x = queueData.tile_x;
            tile_y = queueData.tile_y;

            // lock the tile
            if (xSemaphoreTake(p_sprite_struct->mutex, (TickType_t)0))
            {
                // We now have the semaphore and can access the shared resource.
                loadTile(p_sprite_struct->sprite, zoom, tile_x, tile_y);
                loadRoute(p_sprite_struct->sprite, zoom, tile_x, tile_y);
                loadPoint(p_sprite_struct->sprite, zoom, tile_x, tile_y);

                p_sprite_struct->zoom = zoom;
                p_sprite_struct->tile_x = tile_x;
                p_sprite_struct->tile_y = tile_y;

                // unlock the tile
                if (xSemaphoreGive(p_sprite_struct->mutex) != pdTRUE)
                {
                    Serial.printf("updateTileTask(): Error in xSemaphoreGive() p_sprite_struct=%d, z=%d, tile_x=%d, tile_y=%d\n", p_sprite_struct, zoom, tile_x, tile_y);
                }
            }
            else
            {
                Serial.printf("updateTileTask(): queue was passed because the tile was locked. p_sprite_struct=%d, z=%d, tile_x=%d, tile_y=%d\n", p_sprite_struct, zoom, tile_x, tile_y);
            }
        }

        delay(50);
    }
}

// ================================================================================
// Zoom level
// ================================================================================
int compIntR(const void *a, const void *b) { return *(int *)a < *(int *)b; }

void initZoomList(const char *map_dir_path)
{
    for (int j = 0; j < LEN_ZOOM_LIST; j++)
    {
        zoom_list[j] = -1;
    }

    int i = 0;
    int num;

    // List zoom level directories
    File map_dir = SD.open(map_dir_path, FILE_READ);
    if (map_dir)
    {
        while (true)
        {
            File entry = map_dir.openNextFile();
            if (!entry)
            {
                entry.close();
                break;
            }

            num = String(entry.name()).toInt();
            if (String(entry.name()) == String(num))
            {
                zoom_list[i] = num;
                n_zoom = i + 1;
                i++;
            }

            entry.close();
        }
    }
    map_dir.close();
    Serial.printf("n_zoom: %d\n", n_zoom);

    qsort(zoom_list, n_zoom, sizeof(int), compIntR);

    Serial.print("Zoom levels of stored map: ");
    for (int j = 0; j < LEN_ZOOM_LIST; j++)
    {
        Serial.printf("%d ", zoom_list[j]);
    }
    Serial.println("");

    zoom = zoom_list[zoom_list_i];
}

void changeZoomLevel()
{
    int zoom_prev = zoom;
    zoom_list_i++;

    if (zoom_list_i >= n_zoom)
    {
        zoom_list_i = 0;
    }

    zoom = zoom_list[zoom_list_i];

    Serial.printf("Zoom level changed from %d to %d. ", zoom_prev, zoom);
    Serial.printf("display_center_idx_coords (%d,%d) -> ", display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);

    // Update curr_gps_idx_coords for the new zoom level
    if (gps.location.isValid())
    {
        calcCoords2CoordsIdx(curr_gps_idx_coords, gps.location.lng(),
                             gps.location.lat(), zoom, tile_size);
    }
    else
    {
        curr_gps_idx_coords.idx_x = (int)(curr_gps_idx_coords.idx_x * pow(2.0, (double)(zoom - zoom_prev)));
        curr_gps_idx_coords.idx_y = (int)(curr_gps_idx_coords.idx_y * pow(2.0, (double)(zoom - zoom_prev)));
    }

    // Update display_center_idx_coords for the new zoom level
    display_center_idx_coords.idx_x = (int)(display_center_idx_coords.idx_x * pow(2.0, (double)(zoom - zoom_prev)));
    display_center_idx_coords.idx_y = (int)(display_center_idx_coords.idx_y * pow(2.0, (double)(zoom - zoom_prev)));

    Serial.printf("(%d,%d)\n", display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
}

// ================================================================================
// Sound
// ================================================================================
bool checkIfUseSound()
{
    bool ret = false;

    File fp = SD.open("/useSound", FILE_READ);

    if (fp)
        ret = true;
    fp.close();

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
                               .communication_format = I2S_COMM_FORMAT_I2S,
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

// ========================================
// Button
// ========================================
void vibrate(int t_ms)
{
    M5.Axp.SetLDOEnable(3, true);
    delay(t_ms);
    M5.Axp.SetLDOEnable(3, false);
}

void centeringHandler(Event &e)
{
    // recover to centering mode
    Serial.printf("centeringHandler()\n");
    centering_mode = true;
    display_center_idx_coords = curr_gps_idx_coords;

    updateTileCache(tile_cache, zoom, display_center_idx_coords, n_sprite_x,
                    n_sprite_y);
    drawCanvas(tile_cache, display_center_idx_coords);
}

void zoomHandler(Event &e)
{
    // change zoom level
    Serial.printf("zoomHandler()\n");
    changeZoomLevel();

    updateTileCache(tile_cache, zoom, display_center_idx_coords, n_sprite_x,
                    n_sprite_y);
    drawCanvas(tile_cache, display_center_idx_coords);
}

void moveHandler(Event &e)
{
    // scroll tile
    centering_mode = false;

    Serial.printf("moveHandler() from (%d,%d) to (%d,%d); display_center_idx_coords:%d,%d\n", e.from.x, e.from.y, e.to.x, e.to.y, display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
    display_center_idx_coords.idx_x -= e.to.x - e.from.x;
    display_center_idx_coords.idx_y -= e.to.y - e.from.y;

    drawCanvas(tile_cache, display_center_idx_coords);
}

void wasReleasedHandler(Event &e)
{
    // update tileCache
    Serial.println("wasReleasedHandler()");
    updateTileCache(tile_cache, zoom, display_center_idx_coords, n_sprite_x,
                    n_sprite_y);
    drawCanvas(tile_cache, display_center_idx_coords);
}

void toggleBrightnessHandler(Event &e)
{
    Serial.println("toggleBrightnessHandler()");
    i_brightness = (i_brightness + 1) % n_brightness;
    lcd.setBrightness(brightness_list[i_brightness]);
    Serial.printf("toggleBrightnessHandler()  set to %d", brightness_list[i_brightness]);
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
    // kMBusModeOutput: powered by USB or Battery
    // KMBusModeInput: powered by 5-12V external
    M5.begin(true, true, true, false, kMBusModeOutput);
    lcd.init();
    lcd.setRotation(1);
    lcd.setBrightness(brightness_list[i_brightness]);

    lcd.setTextSize(2);
    lcd.println("Initializing...");

    // Serial connection for debug
    Serial.println("Initializing...");

    // Serial connection to GPS module
    Serial2.begin(9600, SERIAL_8N1, 13, 14);

    // Initialize zoom_list
    initZoomList(map_dir_path);

    // Initialize sprites for image cache
    initTileCache();

    // Initializing sound
    use_sound = checkIfUseSound();
    if (use_sound)
        playBoot();

    // Initializing button handlers
    setupButtonHandlers();

    // Initializing small parts
    initGPSIcon();
    initDirIcon();
    initUpdatingIcon();

    // Initialize queue and task
    initUpdateTileQueue();
    xTaskCreatePinnedToCore(updateTileTask, "updateTileTask", 8192, &update_tile_queue, 1, NULL, 0);

    // Set initial map variables (The tokyo station)
    zoom = z_init;
    curr_gps_idx_coords.idx_x = idx_coords_x_init;
    curr_gps_idx_coords.idx_y = idx_coords_y_init;
    display_center_idx_coords = curr_gps_idx_coords;
    updateTileCache(tile_cache, zoom, display_center_idx_coords, n_sprite_x,
                    n_sprite_y, false);

    Serial.println("Waiting GPS signals...");
    t_prev = millis();
}

void loop()
{
    if (gps.location.isValid())
    {
        if (gps.location.isUpdated())
        {
            if (is_gps_active == false)
                gps_count = 0;

            if (use_sound && gps_count == gps_count_th)
            {
                playGPSActive();
                Serial.println("loop(): playGPSActive() was invoked.");
            }
            Serial.println("loop(): GPS is available.");
            is_gps_active = true;
            gps_count++;
        }
        else
        {
            if (is_gps_active == true)
                gps_count = 0;

            if (use_sound && gps_count == gps_count_th)
            {
                playGPSInactive();
                Serial.println("loop(): playGPSInactive() was invoked.");
            }
            
            Serial.println("loop()  GPS is unavailable.");
            is_gps_active = false;
            gps_count++;
        }

        // Update curr_gps_idx_coords with gps data
        calcCoords2CoordsIdx(curr_gps_idx_coords, gps.location.lng(),
                             gps.location.lat(), zoom, tile_size);

        if (centering_mode)
            display_center_idx_coords = curr_gps_idx_coords;

        Serial.printf("loop()\n");
        Serial.printf("  gps=(%f, %f), satellites=%d ", gps.location.lng(), gps.location.lat(), gps.satellites.value());
        Serial.printf("curr_gps_idx_coords=(%d,%d) ", curr_gps_idx_coords.idx_x, curr_gps_idx_coords.idx_y);
        Serial.printf("display_center_idx_coords=(%d,%d)", display_center_idx_coords.idx_x, display_center_idx_coords.idx_y);
        Serial.println();
    }

    updateTileCache(tile_cache, zoom, display_center_idx_coords, n_sprite_x,
                    n_sprite_y);
    drawCanvas(tile_cache, display_center_idx_coords);

    do
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
