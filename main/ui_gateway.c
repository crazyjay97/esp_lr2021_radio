#include "ui_gateway.h"
#include "app_config.h"
#include "bsp.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_imgfx_scale.h"
#include "esp_heap_caps.h"
#include "qrcodegen.h"
#include "wifi_manager.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "ui_gw";
static const char *kGwNvs = "ui_gw";

static bool gw_nvs_save_u8(const char *key, uint8_t val)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(kGwNvs, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: key=%s err=%s", key, esp_err_to_name(err));
        return false;
    }

    err = nvs_set_u8(h, key, val);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS save failed: key=%s value=%u err=%s",
                 key, val, esp_err_to_name(err));
        return false;
    }
    return true;
}

static uint8_t gw_nvs_load_u8(const char *key, uint8_t def)
{
    nvs_handle_t h;
    uint8_t val = def;
    if (nvs_open(kGwNvs, NVS_READONLY, &h) == ESP_OK) {
        (void)nvs_get_u8(h, key, &val);
        nvs_close(h);
    }
    return val;
}

/* ─── Colors ─── */
#define COL_STATUS_BG   lv_color_hex(0x263831)
#define COL_TITLE_BG    lv_color_hex(0xDFE9E2)
#define COL_BODY_BG     lv_color_hex(0xEEF3EF)
#define COL_BOTTOM_BG   lv_color_hex(0x263831)
#define COL_TEXT_MAIN   lv_color_hex(0x18231F)
#define COL_TEXT_LIGHT  lv_color_hex(0xEAF6EF)
#define COL_GREEN       lv_color_hex(0x2F7D5B)
#define COL_AMBER       lv_color_hex(0xB76A2C)
#define COL_ORANGE      lv_color_hex(0x008CFF)
#define COL_PANEL_BG    lv_color_hex(0xFFFFFF)
#define COL_PANEL_BORDER lv_color_hex(0xC9D8D0)
#define COL_KV_BORDER   lv_color_hex(0xE2EBE6)
#define COL_MUTED       lv_color_hex(0x6A7D75)

/* Battery voltage level colors (bright, for the dark status bar background):
 *   > 3.5V  green, 3.3~3.5V amber, < 3.3V red. */
#define COL_VBAT_GREEN  lv_color_hex(0x4CD98A)
#define COL_VBAT_AMBER  lv_color_hex(0xF2C14E)
#define COL_VBAT_RED    lv_color_hex(0xF25C54)

/* ─── Layout ─── */
#define STATUS_H    22
#define TITLE_H     28
#define BODY_Y      (STATUS_H + TITLE_H)
#define BODY_H      242
#define BOTTOM_Y    292
#define BOTTOM_H    28
#define SCR_W       240
#define SCR_H       320
#define IMG_W       240
#define IMG_H       320

/* ─── State ─── */
static ui_page_t s_page = UI_PAGE_IMAGE;
static ui_gw_capture_cb_t s_capture_cb = NULL;
static ui_gw_interval_cb_t s_interval_cb = NULL;
static SemaphoreHandle_t s_lock = NULL; // points to bsp_lcd's LVGL lock

/* Interval presets */
static const uint32_t s_interval_presets[] = {0, 10, 30, 60, 300, 600, 900, 1200, 1800, 3600};
static const char *s_interval_labels[] = {"Off", "10s", "30s", "1min", "5min", "10min", "15min", "20min", "30min", "1h"};
#define INTERVAL_PRESET_COUNT 10
static int s_cfg_interval_idx = 4; /* default = 5 min */

/* Latest node (camera) battery voltage in mV, 0 = unknown. Shown in status bar right. */
static uint16_t s_node_vbat_mv = 0;
/* Gateway's own supply voltage in mV, 0 = unknown. Shown in status bar left,
 * refreshed once at boot and then every minute from the bsp_vbat cache. */
static uint16_t s_gw_vbat_mv = 0;
static lv_timer_t *s_gw_vbat_timer = NULL;

/* Shared layout objects */
static lv_obj_t *s_scr = NULL;
static lv_obj_t *s_status_bar = NULL;
static lv_obj_t *s_status_lbl_l = NULL;
static lv_obj_t *s_status_lbl_r = NULL;
static lv_obj_t *s_title_bar = NULL;
static lv_obj_t *s_title_lbl = NULL;
static lv_obj_t *s_title_chip = NULL;
static lv_obj_t *s_body = NULL;
static lv_obj_t *s_bottom_bar = NULL;
static lv_obj_t *s_bottom_lbl_l = NULL;
static lv_obj_t *s_bottom_lbl_m = NULL;
static lv_obj_t *s_bottom_lbl_r = NULL;

/* PAGE_IMAGE objects. The front buffer may be owned by LCD DMA, pending stays
 * ready for the next present, and back/spare are the free-buffer pool. */
static lv_obj_t *s_img_canvas = NULL;
static lv_color_t *s_img_canvas_buf = NULL;
static lv_color_t *s_img_canvas_back_buf = NULL;
static lv_color_t *s_img_canvas_spare_buf = NULL;
static lv_color_t *s_img_canvas_pending_buf = NULL;
static lv_timer_t *s_img_present_timer = NULL;
static portMUX_TYPE s_img_canvas_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_img_canvas_writing = false;
static bool s_img_buffers_initialized = false;
static uint8_t s_img_canvas_buffer_count = 0;

typedef struct {
    uint32_t jpeg_size;
    uint32_t elapsed_ms;
    int16_t rssi;
} img_present_meta_t;

static img_present_meta_t s_img_pending_meta = {0};
static lv_obj_t *s_img_placeholder = NULL;
static lv_obj_t *s_img_time_lbl = NULL;
static lv_obj_t *s_img_info_lbl = NULL;
static lv_obj_t *s_img_link_lbl = NULL;
static lv_obj_t *s_img_status_lbl = NULL;
static bool s_has_image = false;

/* Continuous video stream state. When the user starts a capture we enter stream
 * mode: the FIRST frame shows the RX progress page as before; once it displays,
 * every subsequent frame skips the RX page and just refreshes the image canvas
 * in place, and app_main auto-requests the next frame. Cleared when the user
 * leaves the image page (which also aborts the in-flight RX). */
static bool s_stream_mode = false;
static bool s_stream_first_shown = false;

/* PAGE_RX objects */
static lv_obj_t *s_rx_pct_lbl = NULL;
static lv_obj_t *s_rx_bar = NULL;
static lv_obj_t *s_rx_frag_lbl = NULL;
static lv_obj_t *s_rx_rate_lbl = NULL;
static lv_obj_t *s_rx_retry_lbl = NULL;
static lv_obj_t *s_rx_rssi_lbl = NULL;
static uint16_t s_rx_total = 0;
static uint32_t s_rx_start_ms = 0;
static int16_t s_rx_last_rssi = 0;

/* PAGE_LINK objects */
static lv_obj_t *s_link_labels[5] = {NULL};
static int16_t s_link_rssi = 0;
static uint32_t s_link_rate = 0;
static uint32_t s_link_elapsed_ms = 0;
static uint32_t s_link_jpeg_size = 0;

/* Transfer stats (updated per transfer) */
static uint16_t s_stats_total_frags = 0;
static uint16_t s_stats_first_missing = 0;
static uint16_t s_stats_total_retransmitted = 0;
static bool s_stats_first_eot_seen = false;

/* PAGE_CONFIG objects */
static lv_obj_t *s_cfg_touch_btns[9] = {NULL};
static lv_obj_t *s_cfg_touch_lbls[9] = {NULL};
static int s_volume_level = 13; /* 0~15, default 13 → 130% */
static ui_gw_sound_trigger_cb_t s_sound_trigger_cb = NULL;
static int s_sound_trigger_idx = 0;
static const char *s_trigger_labels[] = {"Off", "Low", "Med", "High"};
static ui_gw_pir_trigger_cb_t s_pir_trigger_cb = NULL;
static bool s_pir_on = false;
static ui_gw_voice_alarm_cb_t s_voice_alarm_cb = NULL;
static bool s_alarm_on = false;
static ui_gw_low_power_cb_t s_low_power_cb = NULL;
static bool s_low_power_on = false;
static ui_gw_intercom_cb_t s_intercom_cb = NULL;
static bool s_intercom_on = false;
static bool s_intercom_active = false;
typedef enum {
    INTERCOM_UI_IDLE = 0,
    INTERCOM_UI_CONNECTING,
    INTERCOM_UI_ACTIVE,
    INTERCOM_UI_FAILED,
} intercom_ui_state_t;
static intercom_ui_state_t s_intercom_ui_state = INTERCOM_UI_IDLE;
static ui_gw_wifi_prov_cb_t s_wifi_prov_cb = NULL;
static ui_gw_wifi_disconnect_cb_t s_wifi_disconnect_cb = NULL;
static ui_gw_rx_abort_cb_t s_rx_abort_cb = NULL;

/* PAGE_CONFIG WiFi status panel */
static lv_obj_t *s_cfg_wifi_btn = NULL;
static lv_obj_t *s_cfg_wifi_lbl = NULL;
static bool s_wifi_connected = false;
static char s_wifi_ssid[33] = {0};
static int8_t s_wifi_rssi = 0;

/* PAGE_QR objects */
static lv_obj_t *s_qr_canvas = NULL;
static lv_color_t *s_qr_canvas_buf = NULL;
static char s_qr_payload[200] = {0};

/* Forward declarations */
static void create_shared_layout(void);
static void show_page(ui_page_t page);
static void create_image_page(void);
static void create_rx_page(void);
static void create_link_page(void);
static void create_config_page(void);
static void create_intercom_page(void);
static void create_qr_page(void);
static void destroy_body_children(void);
static void stream_stop(void);
static void start_capture_action(void);
static void update_title(const char *text, const char *chip, lv_color_t chip_bg);
static lv_color_t vbat_level_color(uint16_t mv);
static void gw_vbat_refresh(void);
static void gw_vbat_timer_cb(lv_timer_t *t);
static void image_present_timer_cb(lv_timer_t *t);

static lv_color_t *alloc_image_canvas_buffer(size_t pixels)
{
    const size_t bytes = pixels * sizeof(lv_color_t);
    lv_color_t *buf = heap_caps_aligned_alloc(
        64U, bytes,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf) {
        buf = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    }
    return buf;
}

/* 调用方必须持有 s_img_canvas_mux。 */
static void release_image_canvas_buffer(lv_color_t *buf)
{
    if (!buf) return;
    if (!s_img_canvas_back_buf) {
        s_img_canvas_back_buf = buf;
    } else if (!s_img_canvas_spare_buf) {
        s_img_canvas_spare_buf = buf;
    }
}

static void rotate_rgb565_to_canvas(const uint16_t *src, lv_color_t *dst)
{
    for (int out_y = 0; out_y < IMG_H; out_y++) {
        for (int out_x = 0; out_x < IMG_W; out_x++) {
            uint16_t px = src[(239 - out_x) * 320 + out_y];
            dst[out_y * IMG_W + out_x].full =
                (uint16_t)(((px & 0x001FU) << 11) |
                           (px & 0x07E0U) |
                           ((px & 0xF800U) >> 11));
        }
    }
}

/* PLACEHOLDER_IMPL */

/* ─── Shared layout ─── */
static void create_shared_layout(void)
{
    s_scr = lv_scr_act();
    lv_obj_clean(s_scr);
    lv_obj_set_style_bg_color(s_scr, COL_BODY_BG, 0);

    /* Status bar */
    s_status_bar = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_status_bar);
    lv_obj_set_size(s_status_bar, SCR_W, STATUS_H);
    lv_obj_set_pos(s_status_bar, 0, 0);
    lv_obj_set_style_bg_color(s_status_bar, COL_STATUS_BG, 0);
    lv_obj_set_style_bg_opa(s_status_bar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_status_lbl_l = lv_label_create(s_status_bar);
    lv_obj_set_style_text_color(s_status_lbl_l, COL_TEXT_LIGHT, 0);
    lv_obj_set_style_text_font(s_status_lbl_l, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_status_lbl_l, "GW --V");
    lv_obj_align(s_status_lbl_l, LV_ALIGN_LEFT_MID, 7, 0);

    s_status_lbl_r = lv_label_create(s_status_bar);
    lv_obj_set_style_text_color(s_status_lbl_r, COL_TEXT_LIGHT, 0);
    lv_obj_set_style_text_font(s_status_lbl_r, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_status_lbl_r, "CAM --V");
    lv_obj_align(s_status_lbl_r, LV_ALIGN_RIGHT_MID, -7, 0);

    /* Title bar */
    s_title_bar = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_title_bar);
    lv_obj_set_size(s_title_bar, SCR_W, TITLE_H);
    lv_obj_set_pos(s_title_bar, 0, STATUS_H);
    lv_obj_set_style_bg_color(s_title_bar, COL_TITLE_BG, 0);
    lv_obj_set_style_bg_opa(s_title_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_title_bar, 1, 0);
    lv_obj_set_style_border_color(s_title_bar, lv_color_hex(0xC3D2CA), 0);
    lv_obj_set_style_border_side(s_title_bar, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_clear_flag(s_title_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_title_lbl = lv_label_create(s_title_bar);
    lv_obj_set_style_text_color(s_title_lbl, COL_TEXT_MAIN, 0);
    lv_obj_set_style_text_font(s_title_lbl, &lv_font_montserrat_14, 0);
    lv_label_set_text(s_title_lbl, "");
    lv_obj_align(s_title_lbl, LV_ALIGN_LEFT_MID, 8, 0);

    s_title_chip = lv_label_create(s_title_bar);
    lv_obj_set_style_text_color(s_title_chip, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_title_chip, &lv_font_montserrat_10, 0);
    lv_obj_set_style_bg_color(s_title_chip, COL_GREEN, 0);
    lv_obj_set_style_bg_opa(s_title_chip, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_title_chip, 8, 0);
    lv_obj_set_style_pad_hor(s_title_chip, 6, 0);
    lv_obj_set_style_pad_ver(s_title_chip, 2, 0);
    lv_label_set_text(s_title_chip, "");
    lv_obj_align(s_title_chip, LV_ALIGN_RIGHT_MID, -8, 0);

/* PLACEHOLDER_BODY_BOTTOM */

    /* Body container */
    s_body = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_body);
    lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
    lv_obj_set_pos(s_body, 0, BODY_Y);
    lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);
    lv_obj_set_style_bg_opa(s_body, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_body, 0, 0);

    /* Bottom bar (hidden — all pages use touch now) */
    s_bottom_bar = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_bottom_bar);
    lv_obj_set_size(s_bottom_bar, SCR_W, BOTTOM_H);
    lv_obj_set_pos(s_bottom_bar, 0, BOTTOM_Y);
    lv_obj_set_style_bg_color(s_bottom_bar, COL_BOTTOM_BG, 0);
    lv_obj_set_style_bg_opa(s_bottom_bar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_bottom_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_bottom_bar, LV_OBJ_FLAG_HIDDEN);

    s_bottom_lbl_l = lv_label_create(s_bottom_bar);
    s_bottom_lbl_m = lv_label_create(s_bottom_bar);
    s_bottom_lbl_r = lv_label_create(s_bottom_bar);
}

static void update_title(const char *text, const char *chip, lv_color_t chip_bg)
{
    lv_label_set_text(s_title_lbl, text);
    if (chip && chip[0]) {
        lv_label_set_text(s_title_chip, chip);
        lv_obj_set_style_bg_color(s_title_chip, chip_bg, 0);
        lv_obj_clear_flag(s_title_chip, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_title_chip, LV_OBJ_FLAG_HIDDEN);
    }
}

static void destroy_body_children(void)
{
    lv_obj_clean(s_body);

    lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_body, 0, BODY_Y);
    lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
    lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);

    s_img_canvas = NULL;
    s_img_placeholder = NULL;
    s_img_time_lbl = NULL;
    s_img_info_lbl = NULL;
    s_img_link_lbl = NULL;
    s_img_status_lbl = NULL;
    s_rx_pct_lbl = NULL;
    s_rx_bar = NULL;
    s_rx_frag_lbl = NULL;
    s_rx_rate_lbl = NULL;
    s_rx_retry_lbl = NULL;
    s_rx_rssi_lbl = NULL;
    for (int i = 0; i < 5; i++) s_link_labels[i] = NULL;
    memset(s_cfg_touch_btns, 0, sizeof(s_cfg_touch_btns));
    memset(s_cfg_touch_lbls, 0, sizeof(s_cfg_touch_lbls));
    s_cfg_wifi_lbl = NULL;
    s_cfg_wifi_btn = NULL;
    s_qr_canvas = NULL;
}

/* PLACEHOLDER_PAGES */

/* ─── Helper: create a kv row ─── */
static lv_obj_t *create_kv_row(lv_obj_t *parent, const char *key, const char *val,
                                lv_obj_t **val_out)
{
    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_size(row, lv_pct(100), 22);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, COL_KV_BORDER, 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_pad_hor(row, 8, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *k = lv_label_create(row);
    lv_obj_set_style_text_font(k, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(k, COL_MUTED, 0);
    lv_label_set_text(k, key);

    lv_obj_t *v = lv_label_create(row);
    lv_obj_set_style_text_font(v, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(v, COL_TEXT_MAIN, 0);
    lv_label_set_text(v, val);
    if (val_out) *val_out = v;
    return row;
}

/* ─── PAGE: Image (Home) ─── */
static void create_image_page(void)
{
    const size_t canvas_pixels = IMG_W * IMG_H;

    if (!s_img_buffers_initialized) {
        s_img_canvas_buf = alloc_image_canvas_buffer(canvas_pixels);
        if (s_img_canvas_buf) {
            s_img_canvas_buffer_count = 1;
            s_img_canvas_back_buf = alloc_image_canvas_buffer(canvas_pixels);
            if (s_img_canvas_back_buf) {
                s_img_canvas_buffer_count++;
                s_img_canvas_spare_buf = alloc_image_canvas_buffer(canvas_pixels);
                if (s_img_canvas_spare_buf) {
                    s_img_canvas_buffer_count++;
                }
            }

            memset(s_img_canvas_buf, 0, canvas_pixels * sizeof(lv_color_t));
            if (s_img_canvas_back_buf) {
                memset(s_img_canvas_back_buf, 0,
                       canvas_pixels * sizeof(lv_color_t));
            }
            if (s_img_canvas_spare_buf) {
                memset(s_img_canvas_spare_buf, 0,
                       canvas_pixels * sizeof(lv_color_t));
            }
        }
        s_img_buffers_initialized = s_img_canvas_buf != NULL;
        if (s_img_buffers_initialized) {
            ESP_LOGI(TAG, "image canvas buffers=%u bytes_each=%u",
                     s_img_canvas_buffer_count,
                     (unsigned)(canvas_pixels * sizeof(lv_color_t)));
        }
    }

    if (s_has_image && s_img_canvas_buf) {
        lv_obj_add_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_body, 0, 0);
        lv_obj_set_size(s_body, SCR_W, SCR_H);
        lv_obj_set_style_bg_color(s_body, lv_color_black(), 0);

        s_img_canvas = lv_canvas_create(s_body);
        lv_canvas_set_buffer(s_img_canvas, s_img_canvas_buf, IMG_W, IMG_H,
                             LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_pos(s_img_canvas, 0, 0);
        if (s_stream_mode) {
            lv_obj_add_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_body, 0, BODY_Y);
        lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
        lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);

        s_img_placeholder = lv_label_create(s_body);
        lv_obj_set_style_text_font(s_img_placeholder, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_img_placeholder, lv_color_hex(0xA0B8AC), 0);
        lv_label_set_text(s_img_placeholder, "Waiting for node...");
        lv_obj_align(s_img_placeholder, LV_ALIGN_CENTER, 0, 0);

        update_title("Latest", "", COL_GREEN);
    }
}

/* PLACEHOLDER_RX_PAGE */

/* ─── PAGE: RX Progress ─── */
static void create_rx_page(void)
{
    /* Percentage */
    s_rx_pct_lbl = lv_label_create(s_body);
    lv_obj_set_style_text_font(s_rx_pct_lbl, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(s_rx_pct_lbl, COL_GREEN, 0);
    lv_label_set_text(s_rx_pct_lbl, "0%");
    lv_obj_set_pos(s_rx_pct_lbl, 0, 10);
    lv_obj_set_width(s_rx_pct_lbl, SCR_W);
    lv_obj_set_style_text_align(s_rx_pct_lbl, LV_TEXT_ALIGN_CENTER, 0);

    /* Progress bar */
    s_rx_bar = lv_bar_create(s_body);
    lv_obj_set_size(s_rx_bar, 208, 14);
    lv_obj_set_pos(s_rx_bar, 16, 62);
    lv_bar_set_range(s_rx_bar, 0, 100);
    lv_bar_set_value(s_rx_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_rx_bar, lv_color_hex(0xD9E4DE), LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_rx_bar, COL_GREEN, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_rx_bar, 7, LV_PART_MAIN);
    lv_obj_set_style_radius(s_rx_bar, 7, LV_PART_INDICATOR);

    /* Stats panel */
    lv_obj_t *panel = lv_obj_create(s_body);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 224, 110);
    lv_obj_set_pos(panel, 8, 88);
    lv_obj_set_style_bg_color(panel, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, COL_PANEL_BORDER, 0);
    lv_obj_set_style_radius(panel, 6, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_style_pad_row(panel, 0, 0);

    create_kv_row(panel, "Packets", "0 / 0", &s_rx_frag_lbl);
    create_kv_row(panel, "Rate", "-- kbps", &s_rx_rate_lbl);
    create_kv_row(panel, "Elapsed", "00:00.0", &s_rx_retry_lbl);
    create_kv_row(panel, "RSSI", "-- dBm", &s_rx_rssi_lbl);

    update_title("Receiving", "RX", COL_AMBER);
}

/* PLACEHOLDER_LINK_PAGE */

/* ─── PAGE: Link Details ─── */
static void create_link_page(void)
{
    lv_obj_t *panel = lv_obj_create(s_body);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 224, 154);
    lv_obj_set_pos(panel, 8, 8);
    lv_obj_set_style_bg_color(panel, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_border_color(panel, COL_PANEL_BORDER, 0);
    lv_obj_set_style_radius(panel, 6, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_set_style_pad_row(panel, 0, 0);

    static const char *keys[] = {
        "Mode", "RSSI", "Rate", "Loss", "Transfer"
    };

    char val_bufs[5][32];

    /* Mode */
    snprintf(val_bufs[0], sizeof(val_bufs[0]), "FLRC 2.6M");

    /* RSSI */
    if (s_link_rssi != 0) {
        snprintf(val_bufs[1], sizeof(val_bufs[1]), "%d dBm", s_link_rssi);
    } else {
        snprintf(val_bufs[1], sizeof(val_bufs[1]), "-- dBm");
    }

    /* Rate */
    if (s_link_rate > 0) {
        snprintf(val_bufs[2], sizeof(val_bufs[2]), "%lu kbps",
                 (unsigned long)(s_link_rate / 1000));
    } else {
        snprintf(val_bufs[2], sizeof(val_bufs[2]), "-- kbps");
    }

    /* Loss = first_missing / total * 100% */
    if (s_stats_total_frags > 0 && s_stats_first_eot_seen) {
        uint32_t loss_x10 = (uint32_t)s_stats_first_missing * 1000 / s_stats_total_frags;
        snprintf(val_bufs[3], sizeof(val_bufs[3]), "%lu.%lu%%",
                 (unsigned long)(loss_x10 / 10), (unsigned long)(loss_x10 % 10));
    } else {
        snprintf(val_bufs[3], sizeof(val_bufs[3]), "--%%");
    }

    /* Transfer time */
    if (s_link_elapsed_ms > 0) {
        snprintf(val_bufs[4], sizeof(val_bufs[4]), "%lu.%lu s",
                 (unsigned long)(s_link_elapsed_ms / 1000),
                 (unsigned long)((s_link_elapsed_ms % 1000) / 100));
    } else {
        snprintf(val_bufs[4], sizeof(val_bufs[4]), "-- s");
    }

    for (int i = 0; i < 5; i++) {
        create_kv_row(panel, keys[i], val_bufs[i], &s_link_labels[i]);
    }

    update_title("Link Status", "OK", COL_GREEN);
}

/* ─── WiFi provision button clicked callback ─── */
static void cfg_wifi_btn_clicked_cb(lv_event_t *e)
{
    (void)e;
    if (s_wifi_connected) {
        if (s_wifi_disconnect_cb) s_wifi_disconnect_cb();
    } else {
        if (s_wifi_prov_cb) s_wifi_prov_cb();
    }
}

static void cfg_style_value(int idx, const char *text);

static void start_capture_action(void)
{
    if (s_intercom_on) {
        ESP_LOGI(TAG, "capture action: intercom armed active=%d",
                 s_intercom_active ? 1 : 0);
        stream_stop();
        if (s_intercom_active) {
            show_page(UI_PAGE_INTERCOM);
            return;
        }

        s_intercom_ui_state = INTERCOM_UI_CONNECTING;
        show_page(UI_PAGE_INTERCOM);
        lv_refr_now(NULL);

        bool started = s_intercom_cb && s_intercom_cb(1);
        s_intercom_active = started;
        s_intercom_ui_state = started ? INTERCOM_UI_ACTIVE : INTERCOM_UI_FAILED;
        if (s_page == UI_PAGE_INTERCOM) {
            show_page(UI_PAGE_INTERCOM);
        }
        return;
    }

    if (s_capture_cb) {
        s_stream_mode = true;
        s_stream_first_shown = false;
        show_page(UI_PAGE_RX);
        if (s_capture_cb()) {
            update_title("Waiting...", "RX", COL_AMBER);
        } else {
            s_stream_mode = false;
            show_page(UI_PAGE_IMAGE);
        }
    }
}

/* ─── Touch button clicked callback (value badges only) ─── */
static void cfg_btn_clicked_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);

    switch (idx) {
    case 0: /* Capture */
        start_capture_action();
        break;
    case 2: /* Interval cycle */ {
        int new_idx = (s_cfg_interval_idx + 1) % INTERVAL_PRESET_COUNT;
        /* Commit index + display only if the config was delivered, so the UI
         * stays the single source of truth. */
        if (s_interval_cb && s_interval_cb(s_interval_presets[new_idx])) {
            s_cfg_interval_idx = new_idx;
            cfg_style_value(2, s_interval_labels[s_cfg_interval_idx]);
        }
        break;
    }
    case 3: /* Volume +1 */
        s_volume_level = (s_volume_level + 1) % 16;
        bsp_audio_set_volume((uint8_t)(s_volume_level * 10));
        {
            char buf[8];
            snprintf(buf, sizeof(buf), "%d", s_volume_level);
            cfg_style_value(3, buf);
        }
        gw_nvs_save_u8("vol", (uint8_t)s_volume_level);
        break;
    case 4: /* Sound trigger cycle */ {
        uint8_t new_idx = (s_sound_trigger_idx + 1) % 4;
        if (s_sound_trigger_cb && s_sound_trigger_cb((uint32_t)new_idx)) {
            s_sound_trigger_idx = new_idx;
            if (s_sound_trigger_idx > 0) {
                if (s_cfg_touch_btns[4]) {
                    lv_obj_set_style_bg_color(s_cfg_touch_btns[4], COL_AMBER, 0);
                    lv_obj_set_style_bg_opa(s_cfg_touch_btns[4], LV_OPA_COVER, 0);
                }
                if (s_cfg_touch_lbls[4]) {
                    lv_obj_set_style_text_color(s_cfg_touch_lbls[4], lv_color_white(), 0);
                    lv_label_set_text(s_cfg_touch_lbls[4], s_trigger_labels[s_sound_trigger_idx]);
                }
            } else {
                cfg_style_value(4, s_trigger_labels[0]);
            }
            gw_nvs_save_u8("snd", (uint8_t)s_sound_trigger_idx);
        }
        break;
    }
    }
}

/* ─── PAGE: Config (phone-settings style) ─── */

/* Helper: create a section card container */
static lv_obj_t *cfg_create_section(lv_obj_t *parent, const char *title)
{
    lv_obj_t *sec = lv_obj_create(parent);
    lv_obj_remove_style_all(sec);
    lv_obj_set_width(sec, 224);
    lv_obj_set_height(sec, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_color(sec, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(sec, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(sec, 1, 0);
    lv_obj_set_style_border_color(sec, COL_PANEL_BORDER, 0);
    lv_obj_set_style_radius(sec, 8, 0);
    lv_obj_set_style_pad_top(sec, 4, 0);
    lv_obj_set_style_pad_bottom(sec, 0, 0);
    lv_obj_set_style_pad_hor(sec, 0, 0);
    lv_obj_set_layout(sec, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(sec, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(sec, 0, 0);
    lv_obj_clear_flag(sec, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *lbl = lv_label_create(sec);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(lbl, COL_MUTED, 0);
    lv_obj_set_style_pad_left(lbl, 10, 0);
    lv_label_set_text(lbl, title);

    return sec;
}

/* Helper: create a setting row with label, description, and a VALUE badge (tap to cycle) */
static lv_obj_t *cfg_create_row(lv_obj_t *section, const char *label, const char *desc,
                                 int btn_idx, lv_obj_t **val_out)
{
    lv_obj_t *row = lv_obj_create(section);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, 224);
    lv_obj_set_height(row, desc ? 36 : 30);
    lv_obj_set_style_pad_hor(row, 10, 0);
    lv_obj_set_style_pad_ver(row, 4, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(0xF0F5F2), 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name_lbl = lv_label_create(row);
    lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(name_lbl, COL_TEXT_MAIN, 0);
    lv_label_set_text(name_lbl, label);
    lv_obj_set_pos(name_lbl, 0, desc ? 2 : 5);

    if (desc) {
        lv_obj_t *desc_lbl = lv_label_create(row);
        lv_obj_set_style_text_font(desc_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(desc_lbl, COL_MUTED, 0);
        lv_label_set_text(desc_lbl, desc);
        lv_obj_set_pos(desc_lbl, 0, 18);
    }

    /* Right-side value badge (tap to cycle) */
    lv_obj_t *val_btn = lv_btn_create(row);
    lv_obj_set_size(val_btn, LV_SIZE_CONTENT, 26);
    lv_obj_set_style_radius(val_btn, 13, 0);
    lv_obj_set_style_pad_hor(val_btn, 12, 0);
    lv_obj_set_style_pad_ver(val_btn, 4, 0);
    lv_obj_set_style_border_width(val_btn, 0, 0);
    lv_obj_set_style_min_width(val_btn, 52, 0);
    lv_obj_align(val_btn, LV_ALIGN_RIGHT_MID, 0, 0);

    lv_obj_t *val_lbl = lv_label_create(val_btn);
    lv_obj_set_style_text_font(val_lbl, &lv_font_montserrat_10, 0);
    lv_obj_center(val_lbl);

    if (btn_idx >= 0) {
        lv_obj_add_event_cb(val_btn, cfg_btn_clicked_cb, LV_EVENT_CLICKED, (void *)(intptr_t)btn_idx);
        s_cfg_touch_btns[btn_idx] = val_btn;
        s_cfg_touch_lbls[btn_idx] = val_lbl;
    }
    if (val_out) *val_out = val_lbl;

    return row;
}

/* Helper: create a setting row with a TOGGLE SWITCH on the right */
static void cfg_switch_cb(lv_event_t *e);

static lv_obj_t *cfg_create_toggle_row(lv_obj_t *section, const char *label, const char *desc,
                                        int btn_idx, bool initial_state)
{
    lv_obj_t *row = lv_obj_create(section);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, 224);
    lv_obj_set_height(row, desc ? 36 : 30);
    lv_obj_set_style_pad_hor(row, 10, 0);
    lv_obj_set_style_pad_ver(row, 4, 0);
    lv_obj_set_style_border_width(row, 1, 0);
    lv_obj_set_style_border_color(row, lv_color_hex(0xF0F5F2), 0);
    lv_obj_set_style_border_side(row, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name_lbl = lv_label_create(row);
    lv_obj_set_style_text_font(name_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(name_lbl, COL_TEXT_MAIN, 0);
    lv_label_set_text(name_lbl, label);
    lv_obj_set_pos(name_lbl, 0, desc ? 2 : 5);

    if (desc) {
        lv_obj_t *desc_lbl = lv_label_create(row);
        lv_obj_set_style_text_font(desc_lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(desc_lbl, COL_MUTED, 0);
        lv_label_set_text(desc_lbl, desc);
        lv_obj_set_pos(desc_lbl, 0, 18);
    }

    /* Toggle switch */
    lv_obj_t *sw = lv_switch_create(row);
    lv_obj_set_size(sw, 40, 22);
    lv_obj_align(sw, LV_ALIGN_RIGHT_MID, -6, 0);
    lv_obj_set_style_bg_color(sw, lv_color_hex(0xCCCCCC), LV_PART_MAIN);
    lv_obj_set_style_bg_color(sw, COL_AMBER, LV_PART_INDICATOR | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(sw, lv_color_white(), LV_PART_KNOB);

    if (initial_state) {
        lv_obj_add_state(sw, LV_STATE_CHECKED);
    }

    lv_obj_add_event_cb(sw, cfg_switch_cb, LV_EVENT_VALUE_CHANGED, (void *)(intptr_t)btn_idx);
    s_cfg_touch_btns[btn_idx] = sw;
    s_cfg_touch_lbls[btn_idx] = NULL;

    return row;
}

/* Helper: style a value badge (green background) */
static void cfg_style_value(int idx, const char *text)
{
    if (!s_cfg_touch_btns[idx] || !s_cfg_touch_lbls[idx]) return;
    lv_obj_set_style_bg_color(s_cfg_touch_btns[idx], lv_color_hex(0xE8F5EE), 0);
    lv_obj_set_style_bg_opa(s_cfg_touch_btns[idx], LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(s_cfg_touch_lbls[idx], COL_GREEN, 0);
    lv_label_set_text(s_cfg_touch_lbls[idx], text);
}

/* Gray out or restore a config control. */
static void cfg_set_ctrl_enabled(int idx, bool enabled)
{
    lv_obj_t *btn = s_cfg_touch_btns[idx];
    if (!btn) return;
    if (enabled) {
        lv_obj_clear_state(btn, LV_STATE_DISABLED);
        lv_obj_set_style_opa(btn, LV_OPA_COVER, 0);
    } else {
        lv_obj_add_state(btn, LV_STATE_DISABLED);
        lv_obj_set_style_opa(btn, LV_OPA_50, 0);
    }
}

/* Low power disables the local sound-trigger control. */
static void cfg_apply_low_power_lock(bool low_power_on)
{
    if (low_power_on) {
        if (s_sound_trigger_idx != 0) {
            s_sound_trigger_idx = 0;
            gw_nvs_save_u8("snd", 0);
            cfg_style_value(4, s_trigger_labels[0]);
        }
    }
    cfg_set_ctrl_enabled(4, !low_power_on);  /* Sound trigger */
}

static void cfg_apply_intercom_lock(bool intercom_on)
{
    const int controls[] = {2, 4, 5, 6, 7};
    for (size_t i = 0; i < sizeof(controls) / sizeof(controls[0]); ++i) {
        cfg_set_ctrl_enabled(controls[i], !intercom_on);
    }
}

/* Switch toggle callback — handles all on/off toggles */
static void cfg_switch_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t *sw = lv_event_get_target(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    switch (idx) {
    case 5: /* PIR */
        if (s_pir_trigger_cb) {
            if (s_pir_trigger_cb(on ? 1 : 0)) {
                s_pir_on = on;
                gw_nvs_save_u8("pir", on ? 1 : 0);
            } else {
                if (on) lv_obj_clear_state(sw, LV_STATE_CHECKED);
                else lv_obj_add_state(sw, LV_STATE_CHECKED);
            }
        }
        break;
    case 6: /* Voice alarm */
        if (s_voice_alarm_cb) {
            if (s_voice_alarm_cb(on ? 1 : 0)) {
                s_alarm_on = on;
                gw_nvs_save_u8("alarm", on ? 1 : 0);
            } else {
                if (on) lv_obj_clear_state(sw, LV_STATE_CHECKED);
                else lv_obj_add_state(sw, LV_STATE_CHECKED);
            }
        }
        break;
    case 7: /* Low power */
        if (s_low_power_cb) {
            if (s_low_power_cb(on ? 1 : 0)) {
                s_low_power_on = on;
                gw_nvs_save_u8("lowpwr", on ? 1 : 0);
                /* Lock/unlock Sound trigger to match low power. */
                cfg_apply_low_power_lock(on);
            } else {
                if (on) lv_obj_clear_state(sw, LV_STATE_CHECKED);
                else lv_obj_add_state(sw, LV_STATE_CHECKED);
            }
        }
        break;
    case 8: /* Intercom */
        if (on) {
            if (!gw_nvs_save_u8("intercom", 1)) {
                lv_obj_clear_state(sw, LV_STATE_CHECKED);
                break;
            }
            s_intercom_on = true;
            cfg_apply_intercom_lock(true);
            if (s_cfg_touch_lbls[0]) {
                lv_label_set_text(s_cfg_touch_lbls[0], "START INTERCOM");
            }
            ESP_LOGI(TAG, "intercom armed persisted=1; press CAPTURE to start");
        } else {
            if (!gw_nvs_save_u8("intercom", 0)) {
                lv_obj_add_state(sw, LV_STATE_CHECKED);
                break;
            }
            bool stopped = !s_intercom_active || !s_intercom_cb || s_intercom_cb(0);
            if (stopped) {
                s_intercom_on = false;
                s_intercom_active = false;
                s_intercom_ui_state = INTERCOM_UI_IDLE;
                cfg_apply_intercom_lock(false);
                cfg_apply_low_power_lock(s_low_power_on);
                if (s_cfg_touch_lbls[0]) {
                    lv_label_set_text(s_cfg_touch_lbls[0], "CAPTURE");
                }
                ESP_LOGI(TAG, "intercom disarmed persisted=0");
            } else {
                (void)gw_nvs_save_u8("intercom", 1);
                lv_obj_add_state(sw, LV_STATE_CHECKED);
            }
        }
        break;
    }
}

static void create_config_page(void)
{
    /* Make body scrollable for this page */
    lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
    lv_obj_add_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_body, 0, 0);

    /* Scroll container */
    lv_obj_t *cont = lv_obj_create(s_body);
    lv_obj_remove_style_all(cont);
    lv_obj_set_width(cont, SCR_W);
    lv_obj_set_height(cont, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(cont, 8, 0);
    lv_obj_set_style_pad_row(cont, 8, 0);
    lv_obj_set_layout(cont, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    /* ── CAPTURE button ── */
    lv_obj_t *cap_btn = lv_btn_create(cont);
    lv_obj_set_size(cap_btn, 224, 36);
    lv_obj_set_style_radius(cap_btn, 8, 0);
    lv_obj_set_style_bg_color(cap_btn, COL_GREEN, 0);
    lv_obj_set_style_bg_opa(cap_btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(cap_btn, 0, 0);
    lv_obj_add_event_cb(cap_btn, cfg_btn_clicked_cb, LV_EVENT_CLICKED, (void *)(intptr_t)0);
    lv_obj_t *cap_lbl = lv_label_create(cap_btn);
    lv_obj_set_style_text_font(cap_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(cap_lbl, lv_color_white(), 0);
    lv_label_set_text(cap_lbl, s_intercom_on ? "START INTERCOM" : "CAPTURE");
    lv_obj_center(cap_lbl);
    s_cfg_touch_btns[0] = cap_btn;
    s_cfg_touch_lbls[0] = cap_lbl;

    /* ── TRIGGER section ── */
    lv_obj_t *sec_trig = cfg_create_section(cont, "TRIGGER");

    cfg_create_toggle_row(sec_trig, "PIR Motion", "Infrared detect", 5, s_pir_on);

    cfg_create_row(sec_trig, "Sound", "Mic trigger level", 4, NULL);
    cfg_style_value(4, s_trigger_labels[s_sound_trigger_idx]);
    if (s_sound_trigger_idx > 0) {
        lv_obj_set_style_bg_color(s_cfg_touch_btns[4], COL_AMBER, 0);
        lv_obj_set_style_text_color(s_cfg_touch_lbls[4], lv_color_white(), 0);
    }

    cfg_create_row(sec_trig, "Timer", "Auto-capture interval", 2, NULL);
    cfg_style_value(2, s_interval_labels[s_cfg_interval_idx]);

    /* ── AUDIO section ── */
    lv_obj_t *sec_audio = cfg_create_section(cont, "AUDIO");

    cfg_create_toggle_row(sec_audio, "Intercom", "Two-way voice", 8, s_intercom_on);
    cfg_create_toggle_row(sec_audio, "Voice Alarm", "Play alert on trigger", 6, s_alarm_on);

    char vol_buf[8];
    snprintf(vol_buf, sizeof(vol_buf), "%d", s_volume_level);
    cfg_create_row(sec_audio, "Volume", NULL, 3, NULL);
    cfg_style_value(3, vol_buf);

    /* ── SYSTEM section ── */
    lv_obj_t *sec_sys = cfg_create_section(cont, "SYSTEM");

    cfg_create_toggle_row(sec_sys, "Low Power", "CAD sleep standby", 7, s_low_power_on);

    /* If low power is already on, gray out Sound trigger to match the node. */
    if (s_low_power_on) {
        cfg_set_ctrl_enabled(4, false);  /* Sound trigger */
    }
    if (s_intercom_on) {
        cfg_apply_intercom_lock(true);
    }

    /* ── WiFi info panel ── */
    lv_obj_t *wifi_box = lv_obj_create(cont);
    lv_obj_set_size(wifi_box, 224, 44);
    lv_obj_set_style_radius(wifi_box, 8, 0);
    lv_obj_set_style_bg_opa(wifi_box, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(wifi_box, lv_color_make(0x3a, 0x7a, 0x5f), 0);
    lv_obj_set_style_border_width(wifi_box, 0, 0);
    lv_obj_set_style_pad_all(wifi_box, 4, 0);
    lv_obj_clear_flag(wifi_box, LV_OBJ_FLAG_SCROLLABLE);
    s_cfg_wifi_btn = wifi_box;

    s_cfg_wifi_lbl = lv_label_create(wifi_box);
    lv_obj_set_style_text_font(s_cfg_wifi_lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(s_cfg_wifi_lbl, lv_color_white(), 0);
    lv_obj_center(s_cfg_wifi_lbl);

    {
        char buf[80];
        snprintf(buf, sizeof(buf), "WiFi: %s", wifi_mgr_get_service_name());
        lv_label_set_text(s_cfg_wifi_lbl, buf);
    }

    update_title("Settings", "CFG", COL_GREEN);
}

static void create_intercom_page(void)
{
    const char *headline = "INTERCOM READY";
    const char *hint = "Press CAPTURE to start";
    const char *chip = "READY";
    lv_color_t color = COL_GREEN;

    if (s_intercom_ui_state == INTERCOM_UI_CONNECTING) {
        headline = "CONNECTING...";
        hint = "Starting two-way voice";
        chip = "WAIT";
        color = COL_AMBER;
    } else if (s_intercom_ui_state == INTERCOM_UI_ACTIVE) {
        headline = "INTERCOM ACTIVE";
        hint = "Two-way voice in progress";
        chip = "LIVE";
        color = COL_GREEN;
    } else if (s_intercom_ui_state == INTERCOM_UI_FAILED) {
        headline = "CONNECTION FAILED";
        hint = "Press CAPTURE to retry";
        chip = "FAIL";
        color = lv_color_hex(0xB53A32);
    }

    lv_obj_t *panel = lv_obj_create(s_body);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 224, 176);
    lv_obj_set_pos(panel, 8, 24);
    lv_obj_set_style_bg_color(panel, COL_PANEL_BG, 0);
    lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_set_style_border_color(panel, color, 0);
    lv_obj_set_style_radius(panel, 12, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *state = lv_label_create(panel);
    lv_obj_set_style_text_font(state, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(state, color, 0);
    lv_label_set_text(state, headline);
    lv_obj_align(state, LV_ALIGN_CENTER, 0, -20);

    lv_obj_t *desc = lv_label_create(panel);
    lv_obj_set_style_text_font(desc, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(desc, COL_MUTED, 0);
    lv_label_set_text(desc, hint);
    lv_obj_align(desc, LV_ALIGN_CENTER, 0, 22);

    update_title("Intercom", chip, color);
}

/* ─── PAGE: QR Code ─── */
static void create_qr_page(void)
{
    lv_obj_add_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_body, 0, 0);
    lv_obj_set_size(s_body, SCR_W, SCR_H - BODY_Y);
    lv_obj_set_style_bg_color(s_body, lv_color_white(), 0);

    #define QR_CANVAS_SIZE 200
    if (!s_qr_canvas_buf) {
        s_qr_canvas_buf = heap_caps_malloc(QR_CANVAS_SIZE * QR_CANVAS_SIZE * sizeof(lv_color_t),
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_qr_canvas_buf) {
            s_qr_canvas_buf = heap_caps_malloc(QR_CANVAS_SIZE * QR_CANVAS_SIZE * sizeof(lv_color_t),
                                               MALLOC_CAP_8BIT);
        }
    }

    if (s_qr_canvas_buf && s_qr_payload[0]) {
        for (int i = 0; i < QR_CANVAS_SIZE * QR_CANVAS_SIZE; i++)
            s_qr_canvas_buf[i] = lv_color_white();

        uint8_t qr_buf[qrcodegen_BUFFER_LEN_FOR_VERSION(6)];
        uint8_t tmp_buf[qrcodegen_BUFFER_LEN_FOR_VERSION(6)];
        bool ok = qrcodegen_encodeText(s_qr_payload, tmp_buf, qr_buf,
            qrcodegen_Ecc_LOW, 1, 6, qrcodegen_Mask_AUTO, true);
        if (ok) {
            int qr_size = qrcodegen_getSize(qr_buf);
            int scale = QR_CANVAS_SIZE / (qr_size + 4);
            if (scale < 1) scale = 1;
            int offset = (QR_CANVAS_SIZE - qr_size * scale) / 2;
            for (int y = 0; y < qr_size; y++) {
                for (int x = 0; x < qr_size; x++) {
                    if (qrcodegen_getModule(qr_buf, x, y)) {
                        for (int dy = 0; dy < scale; dy++) {
                            for (int dx = 0; dx < scale; dx++) {
                                int px = offset + x * scale + dx;
                                int py = offset + y * scale + dy;
                                if (px < QR_CANVAS_SIZE && py < QR_CANVAS_SIZE)
                                    s_qr_canvas_buf[py * QR_CANVAS_SIZE + px] = lv_color_black();
                            }
                        }
                    }
                }
            }
        }

        s_qr_canvas = lv_canvas_create(s_body);
        lv_canvas_set_buffer(s_qr_canvas, s_qr_canvas_buf, QR_CANVAS_SIZE, QR_CANVAS_SIZE,
                             LV_IMG_CF_TRUE_COLOR);
        lv_obj_align(s_qr_canvas, LV_ALIGN_TOP_MID, 0, 10);
    } else {
        lv_obj_t *lbl = lv_label_create(s_body);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(lbl, COL_MUTED, 0);
        lv_label_set_text(lbl, "QR generation failed");
        lv_obj_align(lbl, LV_ALIGN_CENTER, 0, 0);
    }

    lv_obj_t *hint = lv_label_create(s_body);
    lv_obj_set_style_text_font(hint, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(hint, COL_MUTED, 0);
    lv_label_set_text(hint, "Scan QR or connect manually:");
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -56);

    lv_obj_t *info_lbl = lv_label_create(s_body);
    lv_obj_set_style_text_font(info_lbl, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(info_lbl, COL_TEXT_MAIN, 0);
    lv_label_set_text_fmt(info_lbl, "AP: %s\nPW: %s\nIP: 192.168.4.1:80",
        wifi_mgr_get_service_name(), wifi_mgr_get_ap_password());
    lv_obj_set_style_text_line_space(info_lbl, 2, 0);
    lv_obj_align(info_lbl, LV_ALIGN_BOTTOM_MID, 0, -16);

}

/* ─── Page switch ─── */
static void show_page(ui_page_t page)
{
    destroy_body_children();
    s_page = page;

    switch (page) {
    case UI_PAGE_IMAGE:  create_image_page();  break;
    case UI_PAGE_RX:     create_rx_page();     break;
    case UI_PAGE_LINK:   create_link_page();   break;
    case UI_PAGE_CONFIG: create_config_page(); break;
    case UI_PAGE_INTERCOM: create_intercom_page(); break;
    case UI_PAGE_QR:     create_qr_page();     break;
    default: break;
    }
}

/* End the video stream: stop auto-requesting frames and abort any in-flight RX.
 * Safe to call when no stream is active. */
static void stream_stop(void)
{
    bool was_streaming = s_stream_mode;
    s_stream_mode = false;
    s_stream_first_shown = false;
    if (was_streaming && s_rx_abort_cb) {
        s_rx_abort_cb();
    }
}

bool ui_gw_stream_active(void)
{
    return s_stream_mode;
}

/* ─── Swipe gesture ─── */
static const ui_page_t s_swipe_order[] = {UI_PAGE_IMAGE, UI_PAGE_LINK, UI_PAGE_CONFIG};
#define SWIPE_PAGE_COUNT 3

static void gesture_cb(lv_event_t *e)
{
    if (s_page == UI_PAGE_RX || s_page == UI_PAGE_QR) return;

    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    if (dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT) return;

    int cur = -1;
    for (int i = 0; i < SWIPE_PAGE_COUNT; i++) {
        if (s_swipe_order[i] == s_page) { cur = i; break; }
    }
    if (cur < 0) return;

    int next;
    if (dir == LV_DIR_LEFT) {
        next = (cur + 1) % SWIPE_PAGE_COUNT;
    } else {
        next = (cur + SWIPE_PAGE_COUNT - 1) % SWIPE_PAGE_COUNT;
    }

    /* Swiping off the image page ends the video stream. */
    if (s_swipe_order[next] != UI_PAGE_IMAGE) {
        stream_stop();
    }
    show_page(s_swipe_order[next]);
}

/* ─── Public API ─── */
esp_err_t ui_gw_init(void)
{
    s_lock = bsp_lcd_get_lvgl_lock();
    if (!s_lock) {
        ESP_LOGE(TAG, "LVGL lock not available");
        return ESP_ERR_INVALID_STATE;
    }

    s_volume_level = gw_nvs_load_u8("vol", 13);
    s_sound_trigger_idx = gw_nvs_load_u8("snd", 0);
    if (s_sound_trigger_idx > 3) s_sound_trigger_idx = 0;
    s_pir_on = gw_nvs_load_u8("pir", 0) != 0;
    s_alarm_on = gw_nvs_load_u8("alarm", 0) != 0;
    s_low_power_on = gw_nvs_load_u8("lowpwr", 0) != 0;
    /* Restore only the armed setting. A live radio session still starts only
     * after the user presses CAPTURE. */
    s_intercom_on = gw_nvs_load_u8("intercom", 0) != 0;
    s_intercom_active = false;
    s_intercom_ui_state = INTERCOM_UI_IDLE;
    bsp_audio_set_volume((uint8_t)(s_volume_level * 10));
    ESP_LOGI(TAG, "NVS load: vol=%d snd=%d pir=%d alarm=%d intercom_armed=%d",
             s_volume_level, s_sound_trigger_idx, s_pir_on, s_alarm_on,
             s_intercom_on ? 1 : 0);

    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    create_shared_layout();
    lv_obj_add_event_cb(s_scr, gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(s_scr, LV_OBJ_FLAG_GESTURE_BUBBLE);
    show_page(UI_PAGE_IMAGE);
    if (!s_img_present_timer) {
        s_img_present_timer = lv_timer_create(image_present_timer_cb, 1, NULL);
    }

    /* Read our own supply voltage once at boot, then refresh every minute.
     * Force a fresh read here so the bar isn't blank until the 15 s background
     * sampler fires; afterwards the timer just re-reads the cache. */
    (void)bsp_vbat_read_mv();
    gw_vbat_refresh();
    s_gw_vbat_timer = lv_timer_create(gw_vbat_timer_cb, 60000, NULL);

    xSemaphoreGiveRecursive(s_lock);

    ESP_LOGI(TAG, "Gateway UI initialized");
    return ESP_OK;
}

void ui_gw_set_capture_cb(ui_gw_capture_cb_t cb)
{
    s_capture_cb = cb;
}

void ui_gw_set_interval_cb(ui_gw_interval_cb_t cb)
{
    s_interval_cb = cb;
}

void ui_gw_set_sound_trigger_cb(ui_gw_sound_trigger_cb_t cb)
{
    s_sound_trigger_cb = cb;
}

void ui_gw_set_pir_trigger_cb(ui_gw_pir_trigger_cb_t cb)
{
    s_pir_trigger_cb = cb;
}

void ui_gw_set_voice_alarm_cb(ui_gw_voice_alarm_cb_t cb)
{
    s_voice_alarm_cb = cb;
}

void ui_gw_set_low_power_cb(ui_gw_low_power_cb_t cb)
{
    s_low_power_cb = cb;
}

void ui_gw_set_intercom_cb(ui_gw_intercom_cb_t cb)
{
    s_intercom_cb = cb;
}

void ui_gw_set_intercom_active(bool active)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    s_intercom_active = active;
    s_intercom_ui_state = active ? INTERCOM_UI_ACTIVE : INTERCOM_UI_IDLE;
    ESP_LOGI(TAG, "intercom session UI: active=%d armed=%d",
             active ? 1 : 0, s_intercom_on ? 1 : 0);
    if (s_page == UI_PAGE_INTERCOM) {
        show_page(UI_PAGE_INTERCOM);
    }

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_key_event(bsp_btn_id_t key, bool pressed)
{
    if (!pressed) return;
    if (!s_lock) return;

    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    if (key == BSP_BTN_VOL_DN) {
        /* K3 = Capture. When Intercom is armed, the same key starts the
         * two-way voice handshake instead of requesting an image stream. */
        start_capture_action();
    } else if (key == BSP_BTN_VOL_UP) {
        /* K4 = Link page */
        stream_stop();
        show_page(UI_PAGE_LINK);
    } else if (key == BSP_BTN_USER1) {
        /* K5 = Config page */
        stream_stop();
        show_page(UI_PAGE_CONFIG);
    } else if (key == BSP_BTN_PTT) {
        /* K6 = Image page */
        stream_stop();
        show_page(UI_PAGE_IMAGE);
    }

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_begin(uint16_t session_id, uint16_t total_frags)
{
    if (!s_lock) return;
    // Non-blocking lock: if LVGL is mid-flush, skip this update and let the
    // next per-fragment progress call (ui_gw_rx_progress) catch up. This is
    // called from the radio RX hot path (ImageStart handling); blocking here
    // delays the ready-ACK and inflates prepare_ms when LVGL holds the lock.
    if (xSemaphoreTakeRecursive(s_lock, 0) != pdTRUE) {
        return;
    }

    s_rx_total = total_frags;
    s_rx_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_rx_last_rssi = 0;

    s_stats_total_frags = total_frags;
    s_stats_first_missing = 0;
    s_stats_total_retransmitted = 0;
    s_stats_first_eot_seen = false;

    // Stream mode after the first frame: don't switch to the RX progress page,
    // keep the live image on screen and let it refresh in place. Progress
    // updates (ui_gw_rx_progress) already no-op unless we're on the RX page.
    if (s_stream_mode && s_stream_first_shown) {
        xSemaphoreGiveRecursive(s_lock);
        return;
    }

    if (s_page != UI_PAGE_RX) {
        show_page(UI_PAGE_RX);
    }

    char title[32];
    snprintf(title, sizeof(title), "Receiving #%03u", session_id);
    update_title(title, "RX", COL_AMBER);

    // Show total frag count and force flush this label only
    if (s_rx_frag_lbl) {
        char buf[32];
        snprintf(buf, sizeof(buf), "0 / %u", total_frags);
        lv_label_set_text(s_rx_frag_lbl, buf);
        lv_obj_invalidate(s_rx_frag_lbl);
        lv_refr_now(NULL);
    }

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_progress(uint16_t received, uint16_t total, int16_t rssi)
{
    if (!s_lock || s_page != UI_PAGE_RX) return;
    // Non-blocking lock: if LVGL is mid-flush, skip this progress update. The
    // next fragment will retry, and the completion callback (ui_gw_rx_done)
    // ensures the final state is shown. This is called from the radio RX path
    // (per-fragment and completion); blocking here can delay NACK sends.
    if (xSemaphoreTakeRecursive(s_lock, 0) != pdTRUE) {
        return;
    }

    s_rx_last_rssi = rssi;
    uint32_t pct = total > 0 ? (uint32_t)received * 100 / total : 0;

    char buf[32];

    if (s_rx_rssi_lbl) {
        snprintf(buf, sizeof(buf), "%d dBm", rssi);
        lv_label_set_text(s_rx_rssi_lbl, buf);
    }
    snprintf(buf, sizeof(buf), "%lu%%", (unsigned long)pct);
    if (s_rx_pct_lbl) lv_label_set_text(s_rx_pct_lbl, buf);
    if (s_rx_bar) lv_bar_set_value(s_rx_bar, (int32_t)pct, LV_ANIM_OFF);

    snprintf(buf, sizeof(buf), "%u / %u", received, total);
    if (s_rx_frag_lbl) lv_label_set_text(s_rx_frag_lbl, buf);

    uint32_t elapsed_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_rx_start_ms;
    if (elapsed_ms > 0 && received > 0) {
        uint32_t bytes = (uint32_t)received * APP_IMAGE_FRAGMENT_DATA_SIZE;
        uint32_t rate_kbps = (uint32_t)((uint64_t)bytes * 8000 / elapsed_ms / 1000);
        snprintf(buf, sizeof(buf), "%lu kbps", (unsigned long)rate_kbps);
        if (s_rx_rate_lbl) lv_label_set_text(s_rx_rate_lbl, buf);
    }

    uint32_t secs = elapsed_ms / 1000;
    uint32_t tenths = (elapsed_ms % 1000) / 100;
    snprintf(buf, sizeof(buf), "%02lu:%02lu.%lu",
             (unsigned long)(secs / 60), (unsigned long)(secs % 60),
             (unsigned long)tenths);
    if (s_rx_retry_lbl) lv_label_set_text(s_rx_retry_lbl, buf);

    xSemaphoreGiveRecursive(s_lock);
}

/* PLACEHOLDER_RX_COMPLETE */

static void image_present_timer_cb(lv_timer_t *t)
{
    (void)t;

    lv_color_t *pending = NULL;
    img_present_meta_t meta = {0};

    portENTER_CRITICAL(&s_img_canvas_mux);
    if (s_img_canvas_pending_buf) {
        pending = s_img_canvas_pending_buf;
        s_img_canvas_pending_buf = NULL;
        meta = s_img_pending_meta;

        lv_color_t *old_front = s_img_canvas_buf;
        s_img_canvas_buf = pending;
        release_image_canvas_buffer(old_front);
    }
    portEXIT_CRITICAL(&s_img_canvas_mux);

    if (!pending) return;

    s_link_rssi = meta.rssi;
    s_link_elapsed_ms = meta.elapsed_ms;
    s_link_jpeg_size = meta.jpeg_size;
    s_link_rate = meta.elapsed_ms > 0
                      ? (uint32_t)((uint64_t)meta.jpeg_size * 8000 / meta.elapsed_ms)
                      : 0;
    s_has_image = true;

    /* Present when a normal image stream is running OR a live intercom call is
     * active. In-call frames (Stage-4) reuse this same present path so a live
     * photo shows up on the image page, covering the intercom status page. */
    if (!s_stream_mode && !s_intercom_active) return;

    bool page_changed = false;
    if (s_page != UI_PAGE_IMAGE || !s_img_canvas) {
        show_page(UI_PAGE_IMAGE);
        page_changed = true;
    }
    if (s_img_canvas && !lv_obj_has_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN)) {
        lv_obj_add_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(s_body);
        page_changed = true;
    }
    if (page_changed) {
        lv_refr_now(NULL);
    }

    esp_err_t err = bsp_lcd_present_video_frame(
        (const uint16_t *)s_img_canvas_buf, IMG_W, IMG_H);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "direct video present failed: %s", esp_err_to_name(err));
        if (s_img_canvas) {
            lv_canvas_set_buffer(s_img_canvas, s_img_canvas_buf, IMG_W, IMG_H,
                                 LV_IMG_CF_TRUE_COLOR);
            lv_obj_clear_flag(s_img_canvas, LV_OBJ_FLAG_HIDDEN);
            lv_obj_invalidate(s_img_canvas);
        }
    }
    s_stream_first_shown = true;
}

void ui_gw_rx_complete(const uint16_t *rgb565, uint32_t w, uint32_t h,
                       uint32_t jpeg_size, uint32_t elapsed_ms)
{
    if (!s_lock || !rgb565 || w != 320 || h != 240 || !s_img_canvas_buf) return;

    if (s_img_canvas_buffer_count >= 2) {
        lv_color_t *render_buf = NULL;

        portENTER_CRITICAL(&s_img_canvas_mux);
        if (!s_img_canvas_writing && s_img_canvas_back_buf) {
            render_buf = s_img_canvas_back_buf;
            s_img_canvas_back_buf = s_img_canvas_spare_buf;
            s_img_canvas_spare_buf = NULL;
            s_img_canvas_writing = true;
        }
        portEXIT_CRITICAL(&s_img_canvas_mux);

        if (!render_buf) return;

        /* 后台缓冲完成旋转和换色，已就绪帧在此期间保持可提交。 */
        rotate_rgb565_to_canvas(rgb565, render_buf);

        portENTER_CRITICAL(&s_img_canvas_mux);
        lv_color_t *replaced_pending = s_img_canvas_pending_buf;
        s_img_pending_meta.jpeg_size = jpeg_size;
        s_img_pending_meta.elapsed_ms = elapsed_ms;
        s_img_pending_meta.rssi = s_rx_last_rssi;
        s_img_canvas_pending_buf = render_buf;
        release_image_canvas_buffer(replaced_pending);
        s_img_canvas_writing = false;
        portEXIT_CRITICAL(&s_img_canvas_mux);

        return;
    }

    /* PSRAM 不足时保留单缓冲兼容路径，功能不丢失。 */
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    rotate_rgb565_to_canvas(rgb565, s_img_canvas_buf);

    s_link_rssi = s_rx_last_rssi;
    s_link_elapsed_ms = elapsed_ms;
    s_link_jpeg_size = jpeg_size;
    s_link_rate = elapsed_ms > 0
                      ? (uint32_t)((uint64_t)jpeg_size * 8000 / elapsed_ms)
                      : 0;
    s_has_image = true;

    if (s_page == UI_PAGE_IMAGE && s_img_canvas) {
        lv_obj_invalidate(s_img_canvas);
    } else {
        show_page(UI_PAGE_IMAGE);
    }
    s_stream_first_shown = true;
    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_failed(const char *reason)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    show_page(UI_PAGE_IMAGE);
    update_title("Latest", "FAIL", lv_color_hex(0xA94442));
    lv_label_set_text(s_status_lbl_r, reason ? reason : "RX Failed");

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_eot_nack(uint16_t missing_count, bool is_first_eot)
{
    if (is_first_eot) {
        s_stats_first_missing = missing_count;
        s_stats_first_eot_seen = true;
    }
    s_stats_total_retransmitted += missing_count;
}

void ui_gw_set_wifi_prov_cb(ui_gw_wifi_prov_cb_t cb)
{
    s_wifi_prov_cb = cb;
}

void ui_gw_set_wifi_disconnect_cb(ui_gw_wifi_disconnect_cb_t cb)
{
    s_wifi_disconnect_cb = cb;
}

void ui_gw_set_rx_abort_cb(ui_gw_rx_abort_cb_t cb)
{
    s_rx_abort_cb = cb;
}

void ui_gw_wifi_update(const char *state_str, const char *ssid, int8_t rssi)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    s_wifi_connected = (state_str && strcmp(state_str, "Connected") == 0);
    if (ssid) {
        strncpy(s_wifi_ssid, ssid, sizeof(s_wifi_ssid) - 1);
        s_wifi_ssid[sizeof(s_wifi_ssid) - 1] = '\0';
    } else {
        s_wifi_ssid[0] = '\0';
    }
    s_wifi_rssi = rssi;

    if (s_cfg_wifi_lbl && s_cfg_wifi_btn) {
        char buf[80];
        snprintf(buf, sizeof(buf), "WiFi: %s", wifi_mgr_get_service_name());
        lv_label_set_text(s_cfg_wifi_lbl, buf);
    }

    if (s_page == UI_PAGE_QR && s_wifi_connected) {
        show_page(UI_PAGE_CONFIG);
    }

    xSemaphoreGiveRecursive(s_lock);
}

/* Map a supply voltage (mV) to its status-bar text color.
 *   > 3.5V green, 3.3~3.5V amber, < 3.3V red. Unknown (0) stays light. */
static lv_color_t vbat_level_color(uint16_t mv)
{
    if (mv == 0)      return COL_TEXT_LIGHT;
    if (mv > 3500)    return COL_VBAT_GREEN;
    if (mv >= 3300)   return COL_VBAT_AMBER;
    return COL_VBAT_RED;
}

/* Refresh the gateway's own voltage on the status bar left label from the
 * bsp_vbat cache. Caller must hold s_lock. */
static void gw_vbat_refresh(void)
{
    if (!s_status_lbl_l) return;

    uint16_t mv = bsp_vbat_get_cached();
    s_gw_vbat_mv = mv;

    char buf[32];
    if (mv > 0) {
        /* e.g. 3982 mV -> "GW 3.98V" */
        snprintf(buf, sizeof(buf), "GW %u.%02uV", mv / 1000, (mv % 1000) / 10);
    } else {
        snprintf(buf, sizeof(buf), "GW --V");
    }
    lv_label_set_text(s_status_lbl_l, buf);
    lv_obj_set_style_text_color(s_status_lbl_l, vbat_level_color(mv), 0);
}

static void gw_vbat_timer_cb(lv_timer_t *t)
{
    (void)t;
    /* LVGL timers run in the LVGL task which already owns the lock, but take it
     * recursively to be safe against other call sites. */
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    gw_vbat_refresh();
    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_update_vbat(uint16_t vbat_mv)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    s_node_vbat_mv = vbat_mv;

    if (s_status_lbl_r) {
        char buf[32];
        if (vbat_mv > 0) {
            /* e.g. 3982 mV -> "CAM 3.98V" */
            snprintf(buf, sizeof(buf), "CAM %u.%02uV",
                     vbat_mv / 1000, (vbat_mv % 1000) / 10);
        } else {
            snprintf(buf, sizeof(buf), "CAM --V");
        }
        lv_label_set_text(s_status_lbl_r, buf);
        lv_obj_set_style_text_color(s_status_lbl_r, vbat_level_color(vbat_mv), 0);
    }

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_show_qr(const char *payload)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    strncpy(s_qr_payload, payload, sizeof(s_qr_payload) - 1);
    s_qr_payload[sizeof(s_qr_payload) - 1] = '\0';
    show_page(UI_PAGE_QR);

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_hide_qr(void)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    if (s_page == UI_PAGE_QR) {
        show_page(UI_PAGE_CONFIG);
    }

    xSemaphoreGiveRecursive(s_lock);
}
