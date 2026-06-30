#include "ui_gateway.h"
#include "app_config.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_imgfx_scale.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "ui_gw";

/* ─── Colors ─── */
#define COL_STATUS_BG   lv_color_hex(0x263831)
#define COL_TITLE_BG    lv_color_hex(0xDFE9E2)
#define COL_BODY_BG     lv_color_hex(0xEEF3EF)
#define COL_BOTTOM_BG   lv_color_hex(0x263831)
#define COL_TEXT_MAIN   lv_color_hex(0x18231F)
#define COL_TEXT_LIGHT  lv_color_hex(0xEAF6EF)
#define COL_GREEN       lv_color_hex(0x2F7D5B)
#define COL_AMBER       lv_color_hex(0xB76A2C)
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
static uint16_t s_rx_total = 0;
static uint32_t s_rx_start_ms = 0;
static int16_t s_rx_last_rssi = 0;

/* PAGE_LINK objects */
static lv_obj_t *s_link_labels[6] = {NULL};
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
static lv_obj_t *s_cfg_rows[6] = {NULL};
static lv_obj_t *s_cfg_val_lbls[6] = {NULL};
static lv_obj_t *s_cfg_btns[2] = {NULL};
static int s_cfg_sel = 0;
static uint32_t s_pkt_delay_us = APP_IMAGE_TX_INTER_PACKET_US;
static ui_gw_pkt_delay_cb_t s_pkt_delay_cb = NULL;
static bool s_cfg_pkt_delay_editing = false;
static ui_gw_audio_clip_cb_t s_audio_clip_cb = NULL;
static bool s_audio_clip_on = false;

/* Forward declarations */
static void create_shared_layout(void);
static void show_page(ui_page_t page);
static void create_image_page(void);
static void create_rx_page(void);
static void create_link_page(void);
static void create_config_page(void);
static void destroy_body_children(void);
static void update_title(const char *text, const char *chip, lv_color_t chip_bg);
static void update_bottom(const char *l, const char *m, const char *r);

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
    lv_label_set_text(s_status_lbl_l, "N01 FLRC");
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
    lv_obj_set_size(s_body, SCR_W, BODY_H);
    lv_obj_set_pos(s_body, 0, BODY_Y);
    lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);
    lv_obj_set_style_bg_opa(s_body, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_body, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(s_body, 0, 0);

    /* Bottom bar */
    s_bottom_bar = lv_obj_create(s_scr);
    lv_obj_remove_style_all(s_bottom_bar);
    lv_obj_set_size(s_bottom_bar, SCR_W, BOTTOM_H);
    lv_obj_set_pos(s_bottom_bar, 0, BOTTOM_Y);
    lv_obj_set_style_bg_color(s_bottom_bar, COL_BOTTOM_BG, 0);
    lv_obj_set_style_bg_opa(s_bottom_bar, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_bottom_bar, LV_OBJ_FLAG_SCROLLABLE);

    s_bottom_lbl_l = lv_label_create(s_bottom_bar);
    lv_obj_set_style_text_color(s_bottom_lbl_l, lv_color_hex(0xDBE9E1), 0);
    lv_obj_set_style_text_font(s_bottom_lbl_l, &lv_font_montserrat_10, 0);
    lv_obj_align(s_bottom_lbl_l, LV_ALIGN_LEFT_MID, 7, 0);

    s_bottom_lbl_m = lv_label_create(s_bottom_bar);
    lv_obj_set_style_text_color(s_bottom_lbl_m, lv_color_hex(0xDBE9E1), 0);
    lv_obj_set_style_text_font(s_bottom_lbl_m, &lv_font_montserrat_10, 0);
    lv_obj_align(s_bottom_lbl_m, LV_ALIGN_CENTER, 0, 0);

    s_bottom_lbl_r = lv_label_create(s_bottom_bar);
    lv_obj_set_style_text_color(s_bottom_lbl_r, lv_color_hex(0xDBE9E1), 0);
    lv_obj_set_style_text_font(s_bottom_lbl_r, &lv_font_montserrat_10, 0);
    lv_obj_align(s_bottom_lbl_r, LV_ALIGN_RIGHT_MID, -7, 0);
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

static void update_bottom(const char *l, const char *m, const char *r)
{
    lv_label_set_text(s_bottom_lbl_l, l ? l : "");
    lv_label_set_text(s_bottom_lbl_m, m ? m : "");
    lv_label_set_text(s_bottom_lbl_r, r ? r : "");
}

static void destroy_body_children(void)
{
    lv_obj_clean(s_body);

    lv_obj_clear_flag(s_status_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_title_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_bottom_bar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_pos(s_body, 0, BODY_Y);
    lv_obj_set_size(s_body, SCR_W, BODY_H);
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
    for (int i = 0; i < 6; i++) s_link_labels[i] = NULL;
    memset(s_cfg_rows, 0, sizeof(s_cfg_rows));
    memset(s_cfg_val_lbls, 0, sizeof(s_cfg_val_lbls));
    memset(s_cfg_btns, 0, sizeof(s_cfg_btns));
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
        lv_obj_add_flag(s_bottom_bar, LV_OBJ_FLAG_HIDDEN);
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
        lv_obj_clear_flag(s_bottom_bar, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(s_body, 0, BODY_Y);
        lv_obj_set_size(s_body, SCR_W, BODY_H);
        lv_obj_set_style_bg_color(s_body, COL_BODY_BG, 0);

        s_img_placeholder = lv_label_create(s_body);
        lv_obj_set_style_text_font(s_img_placeholder, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(s_img_placeholder, lv_color_hex(0xA0B8AC), 0);
        lv_label_set_text(s_img_placeholder, "Waiting for node...");
        lv_obj_align(s_img_placeholder, LV_ALIGN_CENTER, 0, 0);

        update_title("Latest", "", COL_GREEN);
        update_bottom("K6 Back", "K5/K4 Page", "K3 Capture");
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
    create_kv_row(panel, "Rate", "-- KB/s", &s_rx_rate_lbl);
    create_kv_row(panel, "Elapsed", "00:00.0", &s_rx_retry_lbl);
    create_kv_row(panel, "RSSI", "-- dBm", NULL);

    update_title("Receiving", "RX", COL_AMBER);
    update_bottom("K6 Cancel", "", "Receiving");
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
        "Mode", "RSSI", "Rate", "Loss", "Retrans", "Last TX"
    };

    char val_bufs[6][32];

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
        snprintf(val_bufs[2], sizeof(val_bufs[2]), "%lu.%lu KB/s",
                 (unsigned long)(s_link_rate / 1024),
                 (unsigned long)((s_link_rate % 1024) * 10 / 1024));
    } else {
        snprintf(val_bufs[2], sizeof(val_bufs[2]), "-- KB/s");
    }

    /* Loss = first_missing / total * 100% */
    if (s_stats_total_frags > 0 && s_stats_first_eot_seen) {
        uint32_t loss_x10 = (uint32_t)s_stats_first_missing * 1000 / s_stats_total_frags;
        snprintf(val_bufs[3], sizeof(val_bufs[3]), "%lu.%lu%%",
                 (unsigned long)(loss_x10 / 10), (unsigned long)(loss_x10 % 10));
    } else {
        snprintf(val_bufs[3], sizeof(val_bufs[3]), "--%%");
    }

    /* Retrans = total_retransmitted / total * 100% */
    if (s_stats_total_frags > 0 && s_stats_first_eot_seen) {
        uint32_t ret_x10 = (uint32_t)s_stats_total_retransmitted * 1000 / s_stats_total_frags;
        snprintf(val_bufs[4], sizeof(val_bufs[4]), "%lu.%lu%%",
                 (unsigned long)(ret_x10 / 10), (unsigned long)(ret_x10 % 10));
    } else {
        snprintf(val_bufs[4], sizeof(val_bufs[4]), "--%%");
    }

    /* Last TX elapsed */
    if (s_link_elapsed_ms > 0) {
        snprintf(val_bufs[5], sizeof(val_bufs[5]), "%lu.%lu s",
                 (unsigned long)(s_link_elapsed_ms / 1000),
                 (unsigned long)((s_link_elapsed_ms % 1000) / 100));
    } else {
        snprintf(val_bufs[5], sizeof(val_bufs[5]), "-- s");
    }

    for (int i = 0; i < 6; i++) {
        create_kv_row(panel, keys[i], val_bufs[i], &s_link_labels[i]);
    }

    update_title("Link Status", "OK", COL_GREEN);
    update_bottom("K6 Back", "K5/K4 Page", "");
}

/* ─── PAGE: Config ─── */
static void create_config_page(void)
{
    lv_obj_t *panel = lv_obj_create(s_body);
    lv_obj_remove_style_all(panel);
    lv_obj_set_size(panel, 224, 168);
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

    static const char *cfg_keys[] = {"JPEG Quality", "Resolution", "Trigger", "Interval", "Audio", "Pkt Delay"};
    char pkt_delay_str[16];
    snprintf(pkt_delay_str, sizeof(pkt_delay_str), "%luus", (unsigned long)s_pkt_delay_us);
    const char *cfg_vals[] = {"Q=50", "VGA", "Manual", s_interval_labels[s_cfg_interval_idx], s_audio_clip_on ? "On" : "Off", pkt_delay_str};

    for (int i = 0; i < 6; i++) {
        s_cfg_rows[i] = create_kv_row(panel, cfg_keys[i], cfg_vals[i], &s_cfg_val_lbls[i]);
        if (i == s_cfg_sel) {
            lv_obj_set_style_bg_color(s_cfg_rows[i], COL_GREEN, 0);
            lv_obj_set_style_bg_opa(s_cfg_rows[i], LV_OPA_COVER, 0);
        }
    }

    /* Button row */
    lv_obj_t *btn_row = lv_obj_create(s_body);
    lv_obj_remove_style_all(btn_row);
    lv_obj_set_size(btn_row, 224, 32);
    lv_obj_set_pos(btn_row, 8, 184);
    lv_obj_clear_flag(btn_row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_layout(btn_row, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(btn_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(btn_row, 8, 0);

    static const char *btn_texts[] = {"Capture", "Refresh"};
    for (int i = 0; i < 2; i++) {
        lv_obj_t *btn = lv_btn_create(btn_row);
        lv_obj_set_size(btn, 104, 28);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xEDF4EF), 0);
        lv_obj_set_style_border_width(btn, 1, 0);
        lv_obj_set_style_border_color(btn, lv_color_hex(0x9FB5AA), 0);
        lv_obj_set_style_radius(btn, 5, 0);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_10, 0);
        lv_obj_set_style_text_color(lbl, COL_TEXT_MAIN, 0);
        lv_label_set_text(lbl, btn_texts[i]);
        lv_obj_center(lbl);
        s_cfg_btns[i] = btn;
    }

    s_cfg_pkt_delay_editing = false;
    update_title("Config", "CFG", COL_GREEN);
    update_bottom("K6 Back", "K5/K4 Select", "K3 Action");
}

/* PLACEHOLDER_SHOW_PAGE */

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
    default: break;
    }
}

/* ─── Public API ─── */
esp_err_t ui_gw_init(void)
{
    s_lock = bsp_lcd_get_lvgl_lock();
    if (!s_lock) {
        ESP_LOGE(TAG, "LVGL lock not available");
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    create_shared_layout();
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

void ui_gw_set_pkt_delay_cb(ui_gw_pkt_delay_cb_t cb)
{
    s_pkt_delay_cb = cb;
}

void ui_gw_set_audio_clip_cb(ui_gw_audio_clip_cb_t cb)
{
    s_audio_clip_cb = cb;
}

void ui_gw_key_event(bsp_btn_id_t key, bool pressed)
{
    if (!pressed) return;
    if (!s_lock) return;

    xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);

    if (s_page == UI_PAGE_CONFIG) {
        /* Pkt Delay editing mode: K4=10, K5=-10, K3=+1, K6=apply&exit */
        if (s_cfg_pkt_delay_editing) {
            if (key == BSP_BTN_VOL_UP) {
                s_pkt_delay_us += 10;
            } else if (key == BSP_BTN_USER1) {
                if (s_pkt_delay_us >= 10) s_pkt_delay_us -= 10;
                else s_pkt_delay_us = 0;
            } else if (key == BSP_BTN_VOL_DN) {
                s_pkt_delay_us += 1;
            } else if (key == BSP_BTN_PTT) {
                /* K6 = apply and exit editing */
                lv_label_set_text(s_status_lbl_r, "Sending...");
                lv_refr_now(NULL);
                bool ok = s_pkt_delay_cb ? s_pkt_delay_cb(s_pkt_delay_us) : false;
                lv_label_set_text(s_status_lbl_r, ok ? "Set OK" : "Set FAIL");
                s_cfg_pkt_delay_editing = false;
                update_bottom("K6 Back", "K5/K4 Select", "K3 Action");
            }
            if (key != BSP_BTN_PTT && s_cfg_val_lbls[5]) {
                char buf[16];
                snprintf(buf, sizeof(buf), "%luus", (unsigned long)s_pkt_delay_us);
                lv_label_set_text(s_cfg_val_lbls[5], buf);
            }
            goto done;
        }

        /* Normal config page navigation */
        if (key == BSP_BTN_VOL_UP) {
            /* K4 = next item */
            if (s_cfg_rows[s_cfg_sel]) {
                lv_obj_set_style_bg_opa(s_cfg_rows[s_cfg_sel], LV_OPA_TRANSP, 0);
            }
            s_cfg_sel = (s_cfg_sel + 1) % 6;
            if (s_cfg_rows[s_cfg_sel]) {
                lv_obj_set_style_bg_color(s_cfg_rows[s_cfg_sel], COL_GREEN, 0);
                lv_obj_set_style_bg_opa(s_cfg_rows[s_cfg_sel], LV_OPA_COVER, 0);
            }
            if (s_cfg_sel == 5) {
                update_bottom("K6 Apply", "K5-10/K4+10", "K3+1");
            } else {
                update_bottom("K6 Back", "K5/K4 Select", "K3 Action");
            }
        } else if (key == BSP_BTN_USER1) {
            /* K5 = prev item */
            if (s_cfg_rows[s_cfg_sel]) {
                lv_obj_set_style_bg_opa(s_cfg_rows[s_cfg_sel], LV_OPA_TRANSP, 0);
            }
            s_cfg_sel = (s_cfg_sel + 5) % 6;
            if (s_cfg_rows[s_cfg_sel]) {
                lv_obj_set_style_bg_color(s_cfg_rows[s_cfg_sel], COL_GREEN, 0);
                lv_obj_set_style_bg_opa(s_cfg_rows[s_cfg_sel], LV_OPA_COVER, 0);
            }
            if (s_cfg_sel == 5) {
                update_bottom("K6 Apply", "K5-10/K4+10", "K3+1");
            } else {
                update_bottom("K6 Back", "K5/K4 Select", "K3 Action");
            }
        } else if (key == BSP_BTN_VOL_DN) {
            /* K3 = action on selected row */
            if (s_cfg_sel == 3) {
                /* Interval row: cycle preset */
                s_cfg_interval_idx = (s_cfg_interval_idx + 1) % INTERVAL_PRESET_COUNT;
                if (s_cfg_val_lbls[3]) {
                    lv_label_set_text(s_cfg_val_lbls[3], s_interval_labels[s_cfg_interval_idx]);
                }
                lv_label_set_text(s_status_lbl_r, "Sending...");
                lv_refr_now(NULL);
                bool ok = s_interval_cb ? s_interval_cb(s_interval_presets[s_cfg_interval_idx]) : false;
                lv_label_set_text(s_status_lbl_r, ok ? "Set OK" : "Set FAIL");
            } else if (s_cfg_sel == 4) {
                /* Audio clip row: toggle On/Off */
                s_audio_clip_on = !s_audio_clip_on;
                if (s_cfg_val_lbls[4]) {
                    lv_label_set_text(s_cfg_val_lbls[4], s_audio_clip_on ? "On" : "Off");
                }
                lv_label_set_text(s_status_lbl_r, "Sending...");
                lv_refr_now(NULL);
                bool ok = s_audio_clip_cb ? s_audio_clip_cb(s_audio_clip_on ? 1 : 0) : false;
                lv_label_set_text(s_status_lbl_r, ok ? "Set OK" : "Set FAIL");
            } else if (s_cfg_sel == 5) {
                /* Pkt Delay row: enter editing mode */
                s_cfg_pkt_delay_editing = true;
                s_pkt_delay_us += 1;
                if (s_cfg_val_lbls[5]) {
                    char buf[16];
                    snprintf(buf, sizeof(buf), "%luus", (unsigned long)s_pkt_delay_us);
                    lv_label_set_text(s_cfg_val_lbls[5], buf);
                }
                update_bottom("K6 Apply", "K5-10/K4+10", "K3+1");
            } else if (s_cfg_sel == 0 || s_cfg_sel == 1) {
                /* Capture button rows — trigger capture */
                if (s_capture_cb) {
                    show_page(UI_PAGE_RX);
                    update_title("Waiting...", "RX", COL_AMBER);
                    s_capture_cb();
                }
            }
        } else if (key == BSP_BTN_PTT) {
            /* K6 = back */
            show_page(UI_PAGE_IMAGE);
        }
        goto done;
    }

    if (key == BSP_BTN_VOL_UP) {
        /* K4 = next page */
        if (s_page == UI_PAGE_RX) goto done;
        ui_page_t next = (ui_page_t)((s_page + 1) % UI_PAGE_COUNT);
        if (next == UI_PAGE_RX) next = UI_PAGE_LINK;
        show_page(next);
    } else if (key == BSP_BTN_USER1) {
        /* K5 = prev page */
        if (s_page == UI_PAGE_RX) goto done;
        int prev = (int)s_page - 1;
        if (prev < 0) prev = UI_PAGE_COUNT - 1;
        if (prev == UI_PAGE_RX) prev = UI_PAGE_IMAGE;
        show_page((ui_page_t)prev);
    } else if (key == BSP_BTN_VOL_DN) {
        /* K3 = confirm / capture */
        if (s_page == UI_PAGE_IMAGE) {
            if (s_capture_cb) {
                show_page(UI_PAGE_RX);
                update_title("Waiting...", "RX", COL_AMBER);
                s_capture_cb();
            }
        }
    } else if (key == BSP_BTN_PTT) {
        /* K6 = back → go to image page */
        if (s_page != UI_PAGE_IMAGE) {
            show_page(UI_PAGE_IMAGE);
        }
    }

done:
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
    snprintf(buf, sizeof(buf), "%lu%%", (unsigned long)pct);
    if (s_rx_pct_lbl) lv_label_set_text(s_rx_pct_lbl, buf);
    if (s_rx_bar) lv_bar_set_value(s_rx_bar, (int32_t)pct, LV_ANIM_OFF);

    snprintf(buf, sizeof(buf), "%u / %u", received, total);
    if (s_rx_frag_lbl) lv_label_set_text(s_rx_frag_lbl, buf);

    uint32_t elapsed_ms = (uint32_t)(esp_timer_get_time() / 1000) - s_rx_start_ms;
    if (elapsed_ms > 0 && received > 0) {
        uint32_t bytes = (uint32_t)received * APP_IMAGE_FRAGMENT_DATA_SIZE;
        uint32_t rate_bps = bytes * 1000 / elapsed_ms;
        snprintf(buf, sizeof(buf), "%lu.%lu KB/s",
                 (unsigned long)(rate_bps / 1024),
                 (unsigned long)((rate_bps % 1024) * 10 / 1024));
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
        s_link_rate = jpeg_size * 1000 / elapsed_ms;
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

