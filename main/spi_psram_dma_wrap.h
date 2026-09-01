#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configure one initialized SPI host so RX GDMA accepts exact byte lengths.
 * This is used only by the LR20xx host; it does not change the LCD SPI bus. */
esp_err_t app_spi_polling_dma_prepare(spi_host_device_t host_id);

#ifdef __cplusplus
}
#endif
