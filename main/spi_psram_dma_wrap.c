#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_memory_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#define SPI_POLLING_DMA_BUFFER_BYTES 1024U
#define SPI_POLLING_DMA_ALIGNMENT    CONFIG_ESP32S3_DATA_CACHE_LINE_SIZE

/* LR20xx polling transactions originate from stack VLAs. Once ESP-SR AFE is
 * resident, allocating two temporary internal DMA buffers for every large
 * packet can fail. Reserve one aligned pair at link time and serialize users. */
static DMA_ATTR uint8_t s_polling_dma_tx[SPI_POLLING_DMA_BUFFER_BYTES]
    __attribute__((aligned(SPI_POLLING_DMA_ALIGNMENT)));
static DMA_ATTR uint8_t s_polling_dma_rx[SPI_POLLING_DMA_BUFFER_BYTES]
    __attribute__((aligned(SPI_POLLING_DMA_ALIGNMENT)));
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
        rx_bytes <= SPI_POLLING_DMA_BUFFER_BYTES &&
        (!has_tx || (tx_bytes % SPI_POLLING_DMA_ALIGNMENT) == 0U) &&
        (!has_rx || (rx_bytes % SPI_POLLING_DMA_ALIGNMENT) == 0U);
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
