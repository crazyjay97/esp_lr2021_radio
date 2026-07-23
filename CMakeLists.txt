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

# Work around a GCC internal compiler error (Segmentation fault) when compiling
# esp_lcd_panel_rgb.c at the default optimization level. Force just this one IDF
# source file down to -O0; the last -O flag on the command line wins, so this
# overrides the project-wide optimization without touching any other file.
# TARGET_DIRECTORY scopes the property to the esp_lcd component that owns the
# file (requires CMake >= 3.18; IDF 5.5.x bundles a newer CMake).
set_source_files_properties(
    "$ENV{IDF_PATH}/components/esp_lcd/rgb/esp_lcd_panel_rgb.c"
    TARGET_DIRECTORY __idf_esp_lcd
    PROPERTIES COMPILE_OPTIONS "-O0"
)
