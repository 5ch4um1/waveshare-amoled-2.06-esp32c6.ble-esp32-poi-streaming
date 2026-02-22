# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(COMPONENTS
    main
    nimble
    led_strip
    nvs_flash
    esp_littlefs
    fatfs
    esp_adc
    esp_driver_gpio
    esp_driver_spi
    esp_hw_support
    esp_system
    freertos
    log
    heap
    soc
    hal
    esp_rom
    esp_common
    esp_ringbuf
    vfs
    newlib
    esp_dsp
    # esp_wifi removed
)



include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
set(EXTRA_COMPONENT_DIRS
    ./components/bsp_extra/include
    )

idf_build_set_property(MINIMAL_BUILD ON)
project(hello_world)
