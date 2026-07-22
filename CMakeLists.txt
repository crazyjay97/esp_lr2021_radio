# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# ccache on Windows can fail when the project path contains non-ASCII
# characters. Keep it disabled for this workspace path.
set(CCACHE_ENABLE OFF CACHE BOOL "Disable ccache for non-ASCII workspace paths" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(lr2021_radio)

# Workaround: GCC 13.2 segfaults compiling esp_lcd_panel_rgb.c when
# CONFIG_ESP32S3_INSTRUCTION_CACHE_SIZE=0x8000 (32KB) with -Og.
# Lower optimization to -O1 for the entire esp_lcd component to avoid the ICE.
idf_component_get_property(esp_lcd_lib esp_lcd COMPONENT_LIB)
target_compile_options(${esp_lcd_lib} PRIVATE -O1)
