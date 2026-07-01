#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "wifi_provisioning/manager.h"
#include "wifi_provisioning/scheme_softap.h"
#include <string.h>

static const char *TAG = "wifi_mgr";

#define NVS_NAMESPACE    "wifi_cfg"
#define NVS_KEY_SSID     "ssid"
#define NVS_KEY_PASS     "pass"

static wifi_mgr_state_t s_state = WIFI_MGR_DISCONNECTED;
static wifi_mgr_state_cb_t s_state_cb = NULL;
static char s_ssid[33] = {0};
static char s_service_name[16] = {0};
static char s_ap_password[16] = {0};
static int8_t s_rssi = 0;
static esp_netif_t *s_sta_netif = NULL;
static esp_netif_t *s_ap_netif = NULL;
static bool s_wifi_started = false;

static void set_state(wifi_mgr_state_t st)
{
    s_state = st;
    if (s_state_cb) s_state_cb(st);
}

static bool load_credentials(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return false;
    esp_err_t e1 = nvs_get_str(nvs, NVS_KEY_SSID, ssid, &ssid_len);
    esp_err_t e2 = nvs_get_str(nvs, NVS_KEY_PASS, pass, &pass_len);
    nvs_close(nvs);
    return (e1 == ESP_OK && e2 == ESP_OK && ssid[0] != '\0');
}

static void save_credentials(const char *ssid, const char *pass)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_str(nvs, NVS_KEY_SSID, ssid);
    nvs_set_str(nvs, NVS_KEY_PASS, pass);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "credentials saved: %s", ssid);
}

static void update_rssi(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        s_rssi = ap.rssi;
    }
}

static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_PROV_EVENT) {
        switch (id) {
        case WIFI_PROV_CRED_RECV: {
            wifi_sta_config_t *cfg = (wifi_sta_config_t *)data;
            ESP_LOGI(TAG, "prov cred received: %s", (const char *)cfg->ssid);
            break;
        }
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "prov success");
            wifi_prov_mgr_stop_provisioning();
            break;
        case WIFI_PROV_CRED_FAIL:
            ESP_LOGW(TAG, "prov failed, retrying...");
            wifi_prov_mgr_reset_sm_state_on_failure();
            break;
        case WIFI_PROV_END:
            wifi_prov_mgr_deinit();
            ESP_LOGI(TAG, "prov manager deinitialized");
            break;
        default:
            break;
        }
    } else if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *ev = (wifi_event_sta_connected_t *)data;
            memset(s_ssid, 0, sizeof(s_ssid));
            memcpy(s_ssid, ev->ssid, ev->ssid_len);
            set_state(WIFI_MGR_CONNECTING);
            ESP_LOGI(TAG, "STA connected to %s", s_ssid);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "STA disconnected");
            if (s_state != WIFI_MGR_PROVISIONING) {
                set_state(WIFI_MGR_DISCONNECTED);
                esp_wifi_connect();
            }
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        update_rssi();

        wifi_config_t wcfg;
        if (esp_wifi_get_config(WIFI_IF_STA, &wcfg) == ESP_OK) {
            save_credentials((const char *)wcfg.sta.ssid, (const char *)wcfg.sta.password);
            memset(s_ssid, 0, sizeof(s_ssid));
            strncpy(s_ssid, (const char *)wcfg.sta.ssid, sizeof(s_ssid) - 1);
        }
        set_state(WIFI_MGR_CONNECTED);
    }
}

static void generate_service_name(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_service_name, sizeof(s_service_name), "AM36_%01X%02X%02X",
             mac[3] & 0x0F, mac[4], mac[5]);
    snprintf(s_ap_password, sizeof(s_ap_password), "AM36%02X%02X%01X",
             mac[0], mac[1], mac[2] >> 4);
}

static esp_err_t wifi_hw_start(void)
{
    if (s_wifi_started) return ESP_OK;

    if (!s_sta_netif) {
        s_sta_netif = esp_netif_create_default_wifi_sta();
    }
    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 0;
    cfg.static_rx_buf_num = 6;
    cfg.dynamic_rx_buf_num = 12;
    cfg.static_tx_buf_num = 0;
    cfg.dynamic_tx_buf_num = 12;
    cfg.tx_buf_type = 1;
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "wifi init");

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");

    s_wifi_started = true;
    ESP_LOGI(TAG, "WiFi hardware started");
    return ESP_OK;
}

esp_err_t wifi_mgr_init(void)
{
    esp_err_t err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif init: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop: %s", esp_err_to_name(err));
        return err;
    }

    esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL);

    generate_service_name();

    char ssid[33] = {0};
    char pass[65] = {0};
    if (load_credentials(ssid, sizeof(ssid), pass, sizeof(pass))) {
        ESP_LOGI(TAG, "saved WiFi: %s, starting hw...", ssid);
        ESP_RETURN_ON_ERROR(wifi_hw_start(), TAG, "hw start");

        wifi_config_t wcfg = {};
        strncpy((char *)wcfg.sta.ssid, ssid, sizeof(wcfg.sta.ssid) - 1);
        strncpy((char *)wcfg.sta.password, pass, sizeof(wcfg.sta.password) - 1);
        esp_wifi_set_config(WIFI_IF_STA, &wcfg);
        set_state(WIFI_MGR_CONNECTING);
    } else {
        ESP_LOGI(TAG, "no saved credentials, WiFi deferred");
        set_state(WIFI_MGR_DISCONNECTED);
    }

    return ESP_OK;
}

esp_err_t wifi_mgr_start_provisioning(void)
{
    ESP_RETURN_ON_ERROR(wifi_hw_start(), TAG, "hw start for prov");

    if (!s_ap_netif) {
        s_ap_netif = esp_netif_create_default_wifi_ap();
    }

    wifi_prov_mgr_deinit();

    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_softap,
        .scheme_event_handler = WIFI_PROV_EVENT_HANDLER_NONE,
    };
    ESP_RETURN_ON_ERROR(wifi_prov_mgr_init(config), TAG, "prov mgr init");

    wifi_prov_security_t security = WIFI_PROV_SECURITY_1;
    const char *pop = s_ap_password;

    ESP_LOGI(TAG, "starting SoftAP prov: name=%s pass=%s", s_service_name, s_ap_password);

    ESP_RETURN_ON_ERROR(
        wifi_prov_mgr_start_provisioning(security, pop, s_service_name, s_ap_password),
        TAG, "start prov");

    set_state(WIFI_MGR_PROVISIONING);
    return ESP_OK;
}

void wifi_mgr_stop_provisioning(void)
{
    wifi_prov_mgr_stop_provisioning();
    wifi_prov_mgr_deinit();
    set_state(WIFI_MGR_DISCONNECTED);
    ESP_LOGI(TAG, "provisioning stopped");
}

wifi_mgr_state_t wifi_mgr_get_state(void)
{
    if (s_state == WIFI_MGR_CONNECTED) update_rssi();
    return s_state;
}

const char *wifi_mgr_get_ssid(void)
{
    return s_ssid;
}

int8_t wifi_mgr_get_rssi(void)
{
    return s_rssi;
}

const char *wifi_mgr_get_service_name(void)
{
    return s_service_name;
}

const char *wifi_mgr_get_ap_password(void)
{
    return s_ap_password;
}

void wifi_mgr_set_state_cb(wifi_mgr_state_cb_t cb)
{
    s_state_cb = cb;
}
