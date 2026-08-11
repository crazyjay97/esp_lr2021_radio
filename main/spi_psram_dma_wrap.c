#include "driver/spi_master.h"
#include "esp_memory_utils.h"

esp_err_t __real_spi_device_queue_trans(spi_device_handle_t handle,
                                        spi_transaction_t *trans_desc,
                                        TickType_t ticks_to_wait);

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
