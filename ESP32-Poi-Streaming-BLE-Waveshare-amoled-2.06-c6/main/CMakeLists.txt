set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

idf_component_register(
    SRCS main.cpp rtc_pcf85063a.cpp
    INCLUDE_DIRS "." "${PROJECT_DIR}/components/bsp_extra/include"
    REQUIRES waveshare__esp32_c6_touch_amoled_2_06 XPowersLib bt esp-dsp chmorgan__esp-audio-player chmorgan__esp-file-iterator bsp_extra
    PRIV_REQUIRES driver nvs_flash
)

idf_component_get_property(LVGL_LIB lvgl__lvgl COMPONENT_LIB)
target_compile_options(
    ${LVGL_LIB} 
    PRIVATE
        -DLV_LVGL_H_INCLUDE_SIMPLE
)
