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

static void gw_nvs_save_u8(const char *key, uint8_t val)
{
    nvs_handle_t h;
    if (nvs_open(kGwNvs, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, key, val);
    nvs_commit(h);
    nvs_close(h);
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

/* ─── Layout ─── */
#define STATUS_H    22
#define TITLE_H     28
#define BODY_Y      (STATUS_H + TITLE_H)
#define BODY_H      242
#define BOTTOM_Y    292
#define BOTTOM_H    28
#define SCR_W       240
#define SCR_H       320

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

/* PAGE_IMAGE objects */
static lv_obj_t *s_img_canvas = NULL;
static lv_color_t *s_img_canvas_buf = NULL;
static lv_obj_t *s_img_placeholder = NULL;
static lv_obj_t *s_img_time_lbl = NULL;
static lv_obj_t *s_img_info_lbl = NULL;
static lv_obj_t *s_img_link_lbl = NULL;
static lv_obj_t *s_img_status_lbl = NULL;
static bool s_has_image = false;

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
static lv_obj_t *s_cfg_touch_btns[8] = {NULL};
static lv_obj_t *s_cfg_touch_lbls[8] = {NULL};
static int s_volume_level = 13; /* 0~15, default 13 → 130% */
static ui_gw_audio_clip_cb_t s_audio_clip_cb = NULL;
static bool s_audio_clip_on = false;
static ui_gw_sound_trigger_cb_t s_sound_trigger_cb = NULL;
static int s_sound_trigger_idx = 0;
static const char *s_trigger_labels[] = {"Off", "Low", "Med", "High"};
static ui_gw_pir_trigger_cb_t s_pir_trigger_cb = NULL;
static bool s_pir_on = false;
static ui_gw_voice_alarm_cb_t s_voice_alarm_cb = NULL;
static bool s_alarm_on = false;
static ui_gw_low_power_cb_t s_low_power_cb = NULL;
static bool s_low_power_on = false;
static ui_gw_wifi_prov_cb_t s_wifi_prov_cb = NULL;
static ui_gw_wifi_disconnect_cb_t s_wifi_disconnect_cb = NULL;

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
static void create_qr_page(void);
static void destroy_body_children(void);
static void update_title(const char *text, const char *chip, lv_color_t chip_bg);

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
    lv_label_set_text(s_status_lbl_l, "");
    lv_obj_align(s_status_lbl_l, LV_ALIGN_LEFT_MID, 7, 0);

    s_status_lbl_r = lv_label_create(s_status_bar);
    lv_obj_set_style_text_color(s_status_lbl_r, COL_TEXT_LIGHT, 0);
    lv_obj_set_style_text_font(s_status_lbl_r, &lv_font_montserrat_10, 0);
    lv_label_set_text(s_status_lbl_r, "GW --% / CAM --%");
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
    #undef IMG_W
    #undef IMG_H
    #define IMG_W 240
    #define IMG_H 320
    const size_t canvas_pixels = IMG_W * IMG_H;

    if (!s_img_canvas_buf) {
        s_img_canvas_buf = heap_caps_malloc(canvas_pixels * sizeof(lv_color_t),
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_img_canvas_buf) {
            s_img_canvas_buf = heap_caps_malloc(canvas_pixels * sizeof(lv_color_t),
                                                MALLOC_CAP_8BIT);
        }
        if (s_img_canvas_buf) {
            for (size_t i = 0; i < canvas_pixels; i++) {
                s_img_canvas_buf[i] = lv_color_hex(0x000000);
            }
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

/* ─── Touch button clicked callback (value badges only) ─── */
static void cfg_btn_clicked_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);

    switch (idx) {
    case 0: /* Capture */
        if (s_capture_cb) {
            show_page(UI_PAGE_RX);
            if (s_capture_cb()) {
                update_title("Waiting...", "RX", COL_AMBER);
            } else {
                update_title("Audio preparing...", "WAIT", COL_AMBER);
            }
        }
        break;
    case 2: /* Interval cycle */
        s_cfg_interval_idx = (s_cfg_interval_idx + 1) % INTERVAL_PRESET_COUNT;
        cfg_style_value(2, s_interval_labels[s_cfg_interval_idx]);
        if (s_interval_cb) s_interval_cb(s_interval_presets[s_cfg_interval_idx]);
        break;
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

/* Switch toggle callback — handles all on/off toggles */
static void cfg_switch_cb(lv_event_t *e)
{
    int idx = (int)(intptr_t)lv_event_get_user_data(e);
    lv_obj_t *sw = lv_event_get_target(e);
    bool on = lv_obj_has_state(sw, LV_STATE_CHECKED);

    switch (idx) {
    case 1: /* Audio clip */
        if (s_audio_clip_cb) {
            if (s_audio_clip_cb(on ? 1 : 0)) {
                s_audio_clip_on = on;
                gw_nvs_save_u8("audio", on ? 1 : 0);
            } else {
                /* revert switch */
                if (on) lv_obj_clear_state(sw, LV_STATE_CHECKED);
                else lv_obj_add_state(sw, LV_STATE_CHECKED);
            }
        }
        break;
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
            } else {
                if (on) lv_obj_clear_state(sw, LV_STATE_CHECKED);
                else lv_obj_add_state(sw, LV_STATE_CHECKED);
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
    lv_label_set_text(cap_lbl, "CAPTURE");
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

    cfg_create_toggle_row(sec_audio, "Audio Clip", "Record 5s with photo", 1, s_audio_clip_on);

    cfg_create_toggle_row(sec_audio, "Voice Alarm", "Play alert on trigger", 6, s_alarm_on);

    char vol_buf[8];
    snprintf(vol_buf, sizeof(vol_buf), "%d", s_volume_level);
    cfg_create_row(sec_audio, "Volume", NULL, 3, NULL);
    cfg_style_value(3, vol_buf);

    /* ── SYSTEM section ── */
    lv_obj_t *sec_sys = cfg_create_section(cont, "SYSTEM");

    cfg_create_toggle_row(sec_sys, "Low Power", "CAD sleep standby", 7, s_low_power_on);

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
        snprintf(buf, sizeof(buf), "WiFi: %s\nGallery: http://192.168.4.1",
                 wifi_mgr_get_service_name());
        lv_label_set_text(s_cfg_wifi_lbl, buf);
    }

    update_title("Settings", "CFG", COL_GREEN);
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
    case UI_PAGE_QR:     create_qr_page();     break;
    default: break;
    }
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
    s_audio_clip_on = gw_nvs_load_u8("audio", 0) != 0;
    s_sound_trigger_idx = gw_nvs_load_u8("snd", 0);
    if (s_sound_trigger_idx > 3) s_sound_trigger_idx = 0;
    s_pir_on = gw_nvs_load_u8("pir", 0) != 0;
    s_alarm_on = gw_nvs_load_u8("alarm", 0) != 0;
    s_low_power_on = gw_nvs_load_u8("lowpwr", 0) != 0;
    bsp_audio_set_volume((uint8_t)(s_volume_level * 10));
    ESP_LOGI(TAG, "NVS load: vol=%d audio=%d snd=%d pir=%d alarm=%d",
             s_volume_level, s_audio_clip_on, s_sound_trigger_idx, s_pir_on, s_alarm_on);

    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    create_shared_layout();
    lv_obj_add_event_cb(s_scr, gesture_cb, LV_EVENT_GESTURE, NULL);
    lv_obj_clear_flag(s_scr, LV_OBJ_FLAG_GESTURE_BUBBLE);
    show_page(UI_PAGE_IMAGE);
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

void ui_gw_set_audio_clip_cb(ui_gw_audio_clip_cb_t cb)
{
    s_audio_clip_cb = cb;
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

void ui_gw_key_event(bsp_btn_id_t key, bool pressed)
{
    if (!pressed) return;
    if (!s_lock) return;

    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    if (key == BSP_BTN_VOL_DN) {
        /* K3 = Capture (works from any page) */
        if (s_capture_cb) {
            if (s_page != UI_PAGE_RX) {
                show_page(UI_PAGE_RX);
            }
            if (s_capture_cb()) {
                update_title("Waiting...", "RX", COL_AMBER);
            } else {
                update_title("Audio preparing...", "WAIT", COL_AMBER);
            }
        }
    } else if (key == BSP_BTN_VOL_UP) {
        /* K4 = Link page */
        show_page(UI_PAGE_LINK);
    } else if (key == BSP_BTN_USER1) {
        /* K5 = Config page */
        show_page(UI_PAGE_CONFIG);
    } else if (key == BSP_BTN_PTT) {
        /* K6 = Image page */
        show_page(UI_PAGE_IMAGE);
    }

    xSemaphoreGiveRecursive(s_lock);
}

void ui_gw_rx_begin(uint16_t session_id, uint16_t total_frags)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    s_rx_total = total_frags;
    s_rx_start_ms = (uint32_t)(esp_timer_get_time() / 1000);
    s_rx_last_rssi = 0;

    s_stats_total_frags = total_frags;
    s_stats_first_missing = 0;
    s_stats_total_retransmitted = 0;
    s_stats_first_eot_seen = false;

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
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

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

void ui_gw_rx_complete(const uint16_t *rgb565, uint32_t w, uint32_t h,
                       uint32_t jpeg_size, uint32_t elapsed_ms)
{
    if (!s_lock) return;
    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    s_link_rssi = s_rx_last_rssi;
    s_link_elapsed_ms = elapsed_ms;
    s_link_jpeg_size = jpeg_size;
    if (elapsed_ms > 0) {
        s_link_rate = (uint32_t)((uint64_t)jpeg_size * 8000 / elapsed_ms);
    } else {
        s_link_rate = 0;
    }

    /* Scale 640x480 → 320x240, then rotate 90° CW into 240x320 canvas (+ R↔B for BGR panel) */
    if (s_img_canvas_buf && rgb565 && w > 0 && h > 0) {
        const int16_t scale_w = 320;
        const int16_t scale_h = 240;
        esp_imgfx_scale_cfg_t scale_cfg = {
            .in_res = { .width = (int16_t)w, .height = (int16_t)h },
            .in_pixel_fmt = ESP_IMGFX_PIXEL_FMT_RGB565_LE,
            .scale_res = { .width = scale_w, .height = scale_h },
            .filter_type = ESP_IMGFX_SCALE_FILTER_TYPE_BILINEAR,
        };
        esp_imgfx_scale_handle_t scaler = NULL;
        esp_imgfx_err_t ret = esp_imgfx_scale_open(&scale_cfg, &scaler);
        if (ret == ESP_IMGFX_ERR_OK && scaler) {
            uint32_t out_size = scale_w * scale_h * 2;
            uint8_t *scale_buf = heap_caps_malloc(out_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            if (scale_buf) {
                esp_imgfx_data_t in_img = { .data = (uint8_t *)rgb565, .data_len = w * h * 2 };
                esp_imgfx_data_t out_img = { .data = scale_buf, .data_len = out_size };
                ret = esp_imgfx_scale_process(scaler, &in_img, &out_img);
                if (ret == ESP_IMGFX_ERR_OK) {
                    const uint16_t *src = (const uint16_t *)scale_buf;
                    for (int out_y = 0; out_y < IMG_H; out_y++) {
                        for (int out_x = 0; out_x < IMG_W; out_x++) {
                            uint16_t px = src[(scale_h - 1 - out_x) * scale_w + out_y];
                            uint16_t r = (px >> 11) & 0x1F;
                            uint16_t g = (px >> 5) & 0x3F;
                            uint16_t b = px & 0x1F;
                            s_img_canvas_buf[out_y * IMG_W + out_x].full = (b << 11) | (g << 5) | r;
                        }
                    }
                }
                heap_caps_free(scale_buf);
            }
            esp_imgfx_scale_close(scaler);
        }
        s_has_image = true;
    }

    show_page(UI_PAGE_IMAGE);

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
        snprintf(buf, sizeof(buf), "WiFi: %s\nGallery: http://192.168.4.1",
                 wifi_mgr_get_service_name());
        lv_label_set_text(s_cfg_wifi_lbl, buf);
    }

    if (s_page == UI_PAGE_QR && s_wifi_connected) {
        show_page(UI_PAGE_CONFIG);
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
