#include "image_store.h"
#include "app_config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_sntp.h"
#include "opus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static const char *TAG = "img_store";

typedef struct {
    image_meta_t meta;
    uint8_t     *jpeg;
    uint8_t     *opus;
} image_slot_t;

static image_slot_t s_slots[IMAGE_STORE_MAX_SLOTS];
static int s_head = 0;
static int s_count = 0;
static size_t s_total_bytes = 0;
static volatile bool s_abort = false;
static bool s_sntp_started = false;

/* ---- SNTP ---- */

static void sntp_sync_cb(struct timeval *tv)
{
    struct tm t;
    localtime_r(&tv->tv_sec, &t);
    ESP_LOGI(TAG, "SNTP synced: %04d-%02d-%02d %02d:%02d:%02d",
             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
             t.tm_hour, t.tm_min, t.tm_sec);
}

void image_store_start_sntp(void)
{
    if (s_sntp_started) return;
    setenv("TZ", "CST-8", 1);
    tzset();
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_set_time_sync_notification_cb(sntp_sync_cb);
    esp_sntp_init();
    s_sntp_started = true;
    ESP_LOGI(TAG, "SNTP started");
}

/* ---- Storage ---- */

esp_err_t image_store_init(void)
{
    memset(s_slots, 0, sizeof(s_slots));
    s_head = 0;
    s_count = 0;
    s_total_bytes = 0;
    s_abort = false;
    ESP_LOGI(TAG, "initialized (%d slots, %uKB max)",
             IMAGE_STORE_MAX_SLOTS, IMAGE_STORE_MAX_BYTES / 1024);
    return ESP_OK;
}

static void free_slot(int idx)
{
    image_slot_t *s = &s_slots[idx];
    if (s->jpeg) {
        s_total_bytes -= s->meta.jpeg_len;
        heap_caps_free(s->jpeg);
        s->jpeg = NULL;
    }
    if (s->opus) {
        s_total_bytes -= s->meta.opus_len;
        heap_caps_free(s->opus);
        s->opus = NULL;
    }
    s->meta.valid = false;
    s->meta.jpeg_len = 0;
    s->meta.opus_len = 0;
}

esp_err_t image_store_save(const uint8_t *jpeg, size_t jpeg_len,
                           const uint8_t *opus, size_t opus_len,
                           uint16_t session_id)
{
    if (!jpeg || jpeg_len == 0) return ESP_ERR_INVALID_ARG;

    size_t needed = jpeg_len + opus_len;
    if (needed > IMAGE_STORE_MAX_BYTES) {
        ESP_LOGW(TAG, "image too large: %u bytes", (unsigned)needed);
        return ESP_ERR_NO_MEM;
    }

    while (s_total_bytes + needed > IMAGE_STORE_MAX_BYTES && s_count > 0) {
        int oldest = (s_head - s_count + IMAGE_STORE_MAX_SLOTS) % IMAGE_STORE_MAX_SLOTS;
        free_slot(oldest);
        s_count--;
    }

    free_slot(s_head);

    uint8_t *jbuf = (uint8_t *)heap_caps_malloc(jpeg_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!jbuf) {
        ESP_LOGE(TAG, "PSRAM alloc failed for jpeg (%u)", (unsigned)jpeg_len);
        return ESP_ERR_NO_MEM;
    }
    memcpy(jbuf, jpeg, jpeg_len);

    uint8_t *obuf = NULL;
    if (opus && opus_len > 0) {
        obuf = (uint8_t *)heap_caps_malloc(opus_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!obuf) {
            heap_caps_free(jbuf);
            ESP_LOGE(TAG, "PSRAM alloc failed for opus (%u)", (unsigned)opus_len);
            return ESP_ERR_NO_MEM;
        }
        memcpy(obuf, opus, opus_len);
    }

    image_slot_t *slot = &s_slots[s_head];
    slot->jpeg = jbuf;
    slot->opus = obuf;
    slot->meta.jpeg_len = jpeg_len;
    slot->meta.opus_len = opus_len;
    slot->meta.session_id = session_id;
    slot->meta.timestamp = (uint32_t)time(NULL);
    slot->meta.valid = true;
    image_store_start_sntp();

    s_total_bytes += jpeg_len + opus_len;
    s_head = (s_head + 1) % IMAGE_STORE_MAX_SLOTS;
    if (s_count < IMAGE_STORE_MAX_SLOTS) s_count++;

    ESP_LOGI(TAG, "saved image #%d: jpeg=%u opus=%u total_used=%uKB",
             s_count, (unsigned)jpeg_len, (unsigned)opus_len,
             (unsigned)(s_total_bytes / 1024));
    return ESP_OK;
}

int image_store_count(void)
{
    return s_count;
}

static int logical_to_physical(int index)
{
    if (index < 0 || index >= s_count) return -1;
    return (s_head - s_count + index + IMAGE_STORE_MAX_SLOTS) % IMAGE_STORE_MAX_SLOTS;
}

const image_meta_t *image_store_get_meta(int index)
{
    int phys = logical_to_physical(index);
    if (phys < 0) return NULL;
    return &s_slots[phys].meta;
}

const uint8_t *image_store_get_jpeg(int index, size_t *out_len)
{
    int phys = logical_to_physical(index);
    if (phys < 0) return NULL;
    if (out_len) *out_len = s_slots[phys].meta.jpeg_len;
    return s_slots[phys].jpeg;
}

const uint8_t *image_store_get_opus(int index, size_t *out_len)
{
    int phys = logical_to_physical(index);
    if (phys < 0) return NULL;
    if (!s_slots[phys].opus) return NULL;
    if (out_len) *out_len = s_slots[phys].meta.opus_len;
    return s_slots[phys].opus;
}

void image_store_abort_transfer(void)
{
    s_abort = true;
}

/* ---- HTTP Handlers ---- */

#define CHUNK_SIZE  4096

static esp_err_t send_jpeg_chunked(httpd_req_t *req, const uint8_t *data, size_t len)
{
    s_abort = false;
    httpd_resp_set_type(req, "image/jpeg");

    size_t sent = 0;
    while (sent < len) {
        if (s_abort) {
            ESP_LOGW(TAG, "HTTP transfer aborted by capture");
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_OK;
        }
        size_t chunk = (len - sent > CHUNK_SIZE) ? CHUNK_SIZE : (len - sent);
        esp_err_t err = httpd_resp_send_chunk(req, (const char *)&data[sent], chunk);
        if (err != ESP_OK) return err;
        sent += chunk;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t handler_api_images(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    char buf[64];
    httpd_resp_sendstr_chunk(req, "[");

    int count = image_store_count();
    for (int i = 0; i < count; i++) {
        const image_meta_t *m = image_store_get_meta(i);
        if (!m || !m->valid) continue;
        int n = snprintf(buf, sizeof(buf),
            "%s{\"id\":%d,\"jpeg\":%lu,\"opus\":%lu,\"ts\":%lu}",
            (i > 0) ? "," : "",
            i,
            (unsigned long)m->jpeg_len,
            (unsigned long)m->opus_len,
            (unsigned long)m->timestamp);
        httpd_resp_send_chunk(req, buf, n);
    }

    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t handler_img(httpd_req_t *req)
{
    int idx = -1;
    const char *uri = req->uri;
    const char *num = uri + 5;  /* skip "/img/" */
    idx = atoi(num);

    size_t len = 0;
    const uint8_t *data = image_store_get_jpeg(idx, &len);
    if (!data) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Image not found");
        return ESP_FAIL;
    }
    return send_jpeg_chunked(req, data, len);
}

static esp_err_t handler_audio(httpd_req_t *req)
{
    const char *num = req->uri + 7;  /* skip "/audio/" */
    int idx = atoi(num);

    size_t opus_len = 0;
    const uint8_t *opus_data = image_store_get_opus(idx, &opus_len);
    if (!opus_data) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Audio not found");
        return ESP_FAIL;
    }

    /* Count frames */
    size_t pos = 0, num_frames = 0;
    while (pos < opus_len) {
        uint8_t flen = opus_data[pos++];
        if (flen == 0 || pos + flen > opus_len) break;
        pos += flen;
        num_frames++;
    }
    if (num_frames == 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No frames");
        return ESP_FAIL;
    }

    uint32_t pcm_samples = num_frames * APP_AUDIO_FRAME_SAMPLES;
    uint32_t pcm_bytes = pcm_samples * 2;
    size_t wav_size = 44 + pcm_bytes;

    uint8_t *wav_buf = (uint8_t *)heap_caps_malloc(wav_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!wav_buf) {
        ESP_LOGE(TAG, "WAV alloc failed (%u)", (unsigned)wav_size);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    /* WAV header */
    uint32_t file_sz = 36 + pcm_bytes;
    uint32_t sr = APP_AUDIO_SAMPLE_RATE_HZ;
    uint32_t brate = sr * 2;
    uint8_t *h = wav_buf;
    memcpy(h, "RIFF", 4);
    h[4]=file_sz; h[5]=file_sz>>8; h[6]=file_sz>>16; h[7]=file_sz>>24;
    memcpy(h+8, "WAVE", 4);
    memcpy(h+12, "fmt ", 4);
    h[16]=16; h[17]=0; h[18]=0; h[19]=0;
    h[20]=1; h[21]=0;
    h[22]=1; h[23]=0;
    h[24]=sr; h[25]=sr>>8; h[26]=sr>>16; h[27]=sr>>24;
    h[28]=brate; h[29]=brate>>8; h[30]=brate>>16; h[31]=brate>>24;
    h[32]=2; h[33]=0;
    h[34]=16; h[35]=0;
    memcpy(h+36, "data", 4);
    h[40]=pcm_bytes; h[41]=pcm_bytes>>8; h[42]=pcm_bytes>>16; h[43]=pcm_bytes>>24;

    /* Decode */
    int err_code = 0;
    OpusDecoder *dec = opus_decoder_create(APP_AUDIO_SAMPLE_RATE_HZ, 1, &err_code);
    if (!dec) {
        heap_caps_free(wav_buf);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Decoder err");
        return ESP_FAIL;
    }

    int16_t *pcm = (int16_t *)(wav_buf + 44);
    pos = 0;
    size_t soff = 0;
    while (pos < opus_len && soff < pcm_samples) {
        uint8_t flen = opus_data[pos++];
        if (flen == 0 || pos + flen > opus_len) break;
        int got = opus_decode(dec, &opus_data[pos], flen, &pcm[soff], APP_AUDIO_FRAME_SAMPLES, 0);
        pos += flen;
        if (got > 0) soff += got;
    }
    opus_decoder_destroy(dec);

    /* Send */
    s_abort = false;
    httpd_resp_set_type(req, "audio/wav");
    size_t sent = 0;
    while (sent < wav_size) {
        if (s_abort) {
            httpd_resp_send_chunk(req, NULL, 0);
            heap_caps_free(wav_buf);
            return ESP_OK;
        }
        size_t chunk = (wav_size - sent > CHUNK_SIZE) ? CHUNK_SIZE : (wav_size - sent);
        esp_err_t err = httpd_resp_send_chunk(req, (const char *)&wav_buf[sent], chunk);
        if (err != ESP_OK) { heap_caps_free(wav_buf); return err; }
        sent += chunk;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    heap_caps_free(wav_buf);
    return ESP_OK;
}

static const char GALLERY_HTML[] =
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>AM36 Gallery</title>"
    "<style>"
    "*{box-sizing:border-box;margin:0;padding:0}"
    "body{font-family:system-ui,sans-serif;background:#1a1a2e;color:#eee;padding:16px}"
    "h1{text-align:center;margin-bottom:16px;font-size:1.4em;color:#0af}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:8px}"
    ".card{background:#16213e;border-radius:8px;overflow:hidden;cursor:pointer;transition:transform .2s}"
    ".card:hover{transform:scale(1.03)}"
    ".card img{width:100%;height:120px;object-fit:cover}"
    ".card .info{padding:6px;font-size:0.75em;color:#aaa}"
    ".overlay{display:none;position:fixed;top:0;left:0;width:100%;height:100%;"
    "background:rgba(0,0,0,.9);z-index:99;align-items:center;justify-content:center;flex-direction:column}"
    ".overlay.show{display:flex}"
    ".overlay img{max-width:95%;max-height:70vh;object-fit:contain}"
    ".overlay .close{position:absolute;top:12px;right:16px;font-size:2em;color:#fff;cursor:pointer}"
    ".overlay .dl{margin-top:12px;color:#0af;text-decoration:none;font-size:0.9em}"
    "audio{margin-top:10px;width:250px}"
    ".empty{text-align:center;margin-top:40px;color:#666}"
    "</style></head><body>"
    "<h1>AM36 Image Gallery</h1>"
    "<div class='grid' id='grid'></div>"
    "<div class='overlay' id='ov'>"
    "<span class='close' onclick='closeOv()'>&times;</span>"
    "<img id='ovimg'>"
    "<audio id='ovaudio' controls style='display:none'></audio>"
    "<a class='dl' id='ovdl' download>Download</a>"
    "</div>"
    "<div class='empty' id='empty'>No images yet</div>"
    "<script>"
    "var grid=document.getElementById('grid');"
    "var ov=document.getElementById('ov');"
    "var ovimg=document.getElementById('ovimg');"
    "var ovaudio=document.getElementById('ovaudio');"
    "var ovdl=document.getElementById('ovdl');"
    "var empty=document.getElementById('empty');"
    "function fmtT(ts){if(ts<1600000000)return'--:--';"
    "var d=new Date(ts*1000);return d.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit',hour12:false})"
    "+' '+d.toLocaleDateString([],{month:'short',day:'numeric'})}"
    "function openOv(id,hasAudio){"
    "ovimg.src='/img/'+id;ovdl.href='/img/'+id;"
    "if(hasAudio){ovaudio.src='/audio/'+id;ovaudio.style.display='block'}"
    "else{ovaudio.style.display='none';ovaudio.src=''}"
    "ov.classList.add('show')}"
    "function closeOv(){ov.classList.remove('show');ovimg.src='';ovaudio.pause();ovaudio.src=''}"
    "ov.addEventListener('click',function(e){if(e.target===ov)closeOv()});"
    "function load(){"
    "fetch('/api/images').then(function(r){return r.json()}).then(function(imgs){"
    "grid.innerHTML='';"
    "if(!imgs.length){empty.style.display='block';return}"
    "empty.style.display='none';"
    "for(var i=imgs.length-1;i>=0;i--){var m=imgs[i];"
    "var d=document.createElement('div');d.className='card';"
    "d.setAttribute('data-id',m.id);d.setAttribute('data-audio',m.opus>0?'1':'0');"
    "d.onclick=function(){var el=this;openOv(el.getAttribute('data-id'),el.getAttribute('data-audio')==='1')};"
    "var sz=m.jpeg>1024?(m.jpeg/1024).toFixed(1)+'KB':m.jpeg+'B';"
    "var tm=fmtT(m.ts);"
    "d.innerHTML='<img src=\"/img/'+m.id+'\" loading=\"lazy\"><div class=\"info\">'+tm+' '+sz+(m.opus>0?' | Audio':'')+'</div>';"
    "grid.appendChild(d)}})}"
    "load();setInterval(load,5000);"
    "</script></body></html>";

static esp_err_t handler_gallery(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, GALLERY_HTML, sizeof(GALLERY_HTML) - 1);
    return ESP_OK;
}

esp_err_t image_store_register_httpd(httpd_handle_t httpd)
{
    if (!httpd) return ESP_ERR_INVALID_ARG;

    const httpd_uri_t uri_gallery = {
        .uri = "/gallery",
        .method = HTTP_GET,
        .handler = handler_gallery,
    };
    const httpd_uri_t uri_api = {
        .uri = "/api/images",
        .method = HTTP_GET,
        .handler = handler_api_images,
    };
    const httpd_uri_t uri_img = {
        .uri = "/img/*",
        .method = HTTP_GET,
        .handler = handler_img,
    };
    const httpd_uri_t uri_audio = {
        .uri = "/audio/*",
        .method = HTTP_GET,
        .handler = handler_audio,
    };

    httpd_register_uri_handler(httpd, &uri_gallery);
    httpd_register_uri_handler(httpd, &uri_api);
    httpd_register_uri_handler(httpd, &uri_img);
    httpd_register_uri_handler(httpd, &uri_audio);

    ESP_LOGI(TAG, "HTTP handlers registered: /gallery /api/images /img/* /audio/*");
    return ESP_OK;
}
