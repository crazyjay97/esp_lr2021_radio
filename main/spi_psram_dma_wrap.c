#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_memory_utils.h"
#include "esp_private/spi_common_internal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include "spi_psram_dma_wrap.h"

#define SPI_POLLING_DMA_BUFFER_BYTES 1024U
#define SPI_POLLING_DMA_BUFFER_ALIGNMENT CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE

static const char *TAG = "spi_dma_wrap";

/* LR20xx polling transactions originate from stack VLAs. Once ESP-SR AFE is
 * resident, allocating two temporary internal DMA buffers for every large
 * packet can fail. Reserve one aligned pair at link time and serialize users. */
static DMA_ATTR uint8_t s_polling_dma_tx[SPI_POLLING_DMA_BUFFER_BYTES]
    __attribute__((aligned(SPI_POLLING_DMA_BUFFER_ALIGNMENT)));
static DMA_ATTR uint8_t s_polling_dma_rx[SPI_POLLING_DMA_BUFFER_BYTES]
    __attribute__((aligned(SPI_POLLING_DMA_BUFFER_ALIGNMENT)));
static StaticSemaphore_t s_polling_dma_mutex_storage;
static SemaphoreHandle_t s_polling_dma_mutex;
static portMUX_TYPE s_polling_dma_init_lock = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t polling_dma_mutex(void)
{
    if (s_polling_dma_mutex == NULL) {
        portENTER_CRITICAL(&s_polling_dma_init_lock);
        if (s_polling_dma_mutex == NULL) {
            s_polling_dma_mutex =
                xSemaphoreCreateMutexStatic(&s_polling_dma_mutex_storage);
        }
        portEXIT_CRITICAL(&s_polling_dma_init_lock);
    }
    return s_polling_dma_mutex;
}

esp_err_t app_spi_polling_dma_prepare(spi_host_device_t host_id)
{
#if SOC_GDMA_SUPPORTED
    /* IDF enables a 32-byte AHB data burst for both SPI GDMA directions. On
     * ESP32-S3 the RX burst makes every RX byte count require 4-byte alignment.
     * Disable data burst only on this host's RX channel, then refresh the
     * alignment cached by spi_master. TX burst remains enabled. */
    const spi_dma_ctx_t *const_ctx = spi_bus_get_dma_ctx(host_id);
    if (const_ctx == NULL || const_ctx->rx_dma_chan == NULL) {
        ESP_LOGE(TAG, "SPI%d RX DMA context unavailable", (int)host_id + 1);
        return ESP_ERR_INVALID_STATE;
    }

    gdma_transfer_config_t transfer_cfg = {
        .max_data_burst_size = 0,
        .access_ext_mem = true,
    };
    esp_err_t err = gdma_config_transfer(const_ctx->rx_dma_chan, &transfer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI%d disable RX data burst failed: %s",
                 (int)host_id + 1, esp_err_to_name(err));
        return err;
    }

    size_t int_alignment = 0;
    size_t ext_alignment = 0;
    err = gdma_get_alignment_constraints(const_ctx->rx_dma_chan,
                                         &int_alignment, &ext_alignment);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI%d read RX alignment failed: %s",
                 (int)host_id + 1, esp_err_to_name(err));
        return err;
    }

    /* spi_bus_get_dma_ctx() intentionally exposes a const diagnostic view, but
     * spi_master stores the channel's alignment in this same private context.
     * Keep this IDF-private compatibility adjustment isolated in one helper. */
    spi_dma_ctx_t *ctx = (spi_dma_ctx_t *)const_ctx;
    ctx->dma_align_rx_int = int_alignment;
    ctx->dma_align_rx_ext = ext_alignment;
    ESP_LOGI(TAG, "SPI%d RX data burst off, align internal=%u external=%u",
             (int)host_id + 1, (unsigned)int_alignment,
             (unsigned)ext_alignment);
    return ESP_OK;
#else
    (void)host_id;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t __real_spi_device_queue_trans(spi_device_handle_t handle,
                                        spi_transaction_t *trans_desc,
                                        TickType_t ticks_to_wait);

esp_err_t __real_spi_device_polling_transmit(spi_device_handle_t handle,
                                             spi_transaction_t *trans_desc);

esp_err_t __wrap_spi_device_queue_trans(spi_device_handle_t handle,
                                        spi_transaction_t *trans_desc,
                                        TickType_t ticks_to_wait)
{
    if (!(trans_desc->flags & SPI_TRANS_USE_TXDATA) &&
        trans_desc->tx_buffer &&
        esp_ptr_external_ram(trans_desc->tx_buffer) &&
        esp_ptr_dma_ext_capable(trans_desc->tx_buffer)) {
        trans_desc->flags |= SPI_TRANS_DMA_USE_PSRAM;
    }

    return __real_spi_device_queue_trans(handle, trans_desc, ticks_to_wait);
}

esp_err_t __wrap_spi_device_polling_transmit(spi_device_handle_t handle,
                                             spi_transaction_t *trans_desc)
{
    if (trans_desc == NULL ||
        (trans_desc->flags & (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA)) != 0 ||
        (trans_desc->length & 7U) != 0 ||
        (trans_desc->rxlength & 7U) != 0) {
        return __real_spi_device_polling_transmit(handle, trans_desc);
    }

    const bool has_tx = trans_desc->tx_buffer != NULL;
    const bool has_rx = trans_desc->rx_buffer != NULL;
    const size_t tx_bytes = trans_desc->length / 8U;
    const size_t rx_bytes = has_rx
                                ? ((trans_desc->rxlength != 0
                                        ? trans_desc->rxlength
                                        : trans_desc->length) /
                                   8U)
                                : 0U;

    const bool can_use_reserved_dma =
        (has_tx || has_rx) &&
        tx_bytes <= SPI_POLLING_DMA_BUFFER_BYTES &&
        rx_bytes <= SPI_POLLING_DMA_BUFFER_BYTES;
    if (!can_use_reserved_dma) {
        return __real_spi_device_polling_transmit(handle, trans_desc);
    }

    SemaphoreHandle_t mutex = polling_dma_mutex();
    if (mutex == NULL || xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }

    spi_transaction_t dma_trans = *trans_desc;
    if (has_tx) {
        memcpy(s_polling_dma_tx, trans_desc->tx_buffer, tx_bytes);
        dma_trans.tx_buffer = s_polling_dma_tx;
    }
    if (has_rx) {
        dma_trans.rx_buffer = s_polling_dma_rx;
    }
    dma_trans.flags &= ~SPI_TRANS_DMA_USE_PSRAM;

    const esp_err_t err =
        __real_spi_device_polling_transmit(handle, &dma_trans);
    if (err == ESP_OK && has_rx) {
        memcpy(trans_desc->rx_buffer, s_polling_dma_rx, rx_bytes);
    }

    xSemaphoreGive(mutex);
    return err;
}
