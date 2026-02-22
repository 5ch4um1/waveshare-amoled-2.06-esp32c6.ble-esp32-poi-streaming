#include <stdio.h>
#include <cstring>
#include <math.h>
#include <time.h>
#include <sys/time.h> // For struct timeval
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" // For SemaphoreHandle_t
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_random.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "qmi8658.h"
#include "esp_lvgl_port.h"
#include "driver/i2c_master.h" // For I2C master functions
#include "rtc_pcf85063a.h" // For PCF85063A RTC

/* NimBLE BLE */
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "esp_dsp.h" // For FFT functions
#include "bsp_board_extra.h" // For bsp_extra_codec_init and bsp_extra_i2s_read

static const char *TAG = "WATCH_POI";

// --- Hardware Configuration ---
#define POWER_BUTTON_GPIO GPIO_NUM_18
#define MODE_BUTTON_GPIO  GPIO_NUM_9
#define TARGET_NAME       "Open Pixel Poi"
#define START_BYTE        0xD0
#define CC_START_STREAM   21
#define CC_STREAM_DATA    24
#define CC_GET_CONFIG     23
#define BYTES_PER_PIXEL   3
#define GLOBAL_BRIGHTNESS 0.27
#define NUM_LEDS          21
#define LVGL_PORT_LOCK_TIMEOUT_MS 50
#define MIN_BRIGHTNESS    0.05f // Minimum brightness to ensure LEDs are never completely off

// Uncomment the following line to enable initial RTC time setting
// #define SET_INITIAL_RTC_TIME

// PMU I2C Config (using defaults if not in sdkconfig)
#ifndef CONFIG_I2C_MASTER_PORT_NUM
#define CONFIG_I2C_MASTER_PORT_NUM  I2C_NUM_0
#endif
#ifndef CONFIG_I2C_MASTER_FREQUENCY
#define CONFIG_I2C_MASTER_FREQUENCY 400000
#endif
#ifndef CONFIG_PMU_I2C_SCL
#define CONFIG_PMU_I2C_SCL          7
#endif
#ifndef CONFIG_PMU_I2C_SDA
#define CONFIG_PMU_I2C_SDA          8
#endif
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY
#define I2C_MASTER_SDA_IO (gpio_num_t) CONFIG_PMU_I2C_SDA
#define I2C_MASTER_SCL_IO (gpio_num_t) CONFIG_PMU_I2C_SCL
#define I2C_MASTER_TIMEOUT_MS 1000

// --- Audio FFT Configuration ---
#define N_SAMPLES           16  // Reduced FFT size for memory efficiency
#define SAMPLE_RATE         2000 // Reduced sample rate for efficiency
#define CHANNELS            2    // Stereo audio

// Audio & FFT Buffers (aligned for DSP library)
__attribute__((aligned(16))) int16_t raw_data[N_SAMPLES * CHANNELS];
__attribute__((aligned(16))) float audio_buffer[N_SAMPLES];
__attribute__((aligned(16))) float wind[N_SAMPLES];
__attribute__((aligned(16))) float fft_buffer[N_SAMPLES * 2];
__attribute__((aligned(16))) float spectrum[N_SAMPLES / 2]; // Stores magnitude spectrum in dB

// --- Display Stuff (LVGL Object Pointers) ---
static lv_obj_t *battery_label;
// Removed scr_dashboard and scr_clock
static lv_obj_t *poi_info_box;
static lv_obj_t *poi_info_label;
static lv_obj_t *clock_time_label;
static lv_obj_t *clock_date_label;
static lv_obj_t *clock_unix_label;
static lv_obj_t *mic_sens_low_btn;      // New: Mic Sensitivity Low button
static lv_obj_t *mic_sens_medium_btn;   // New: Mic Sensitivity Medium button
static lv_obj_t *mic_sens_high_btn;     // New: Mic Sensitivity High button
static float mic_sensitivity_low = 1.0f;
static float mic_sensitivity_medium = 3.0f;
static float mic_sensitivity_high = 5.0f;
static float current_mic_sensitivity = mic_sensitivity_medium; // Default to medium

// New screen objects
static lv_obj_t *scr_poi_modes_1; // First page of POI modes
static lv_obj_t *scr_poi_modes_2; // Second page of POI modes
static lv_obj_t *scr_audio_modes;
static lv_obj_t *scr_system_info;

typedef enum {
    SCREEN_POI_MODES_1,
    SCREEN_POI_MODES_2,
    SCREEN_AUDIO_MODES,
    SCREEN_SYSTEM_INFO,
    NUM_SCREENS
} app_screen_t;

static app_screen_t current_app_screen = SCREEN_POI_MODES_1;

static lv_obj_t *selected_mode_btn = NULL; // To keep track of the currently selected mode button

// Forward declaration for the event callback
static void mode_button_event_cb(lv_event_t * e);


// --- Mode Function Pointer Type ---
typedef void (*poi_mode_fn)(qmi8658_data_t *s, uint8_t *pixel_data, size_t len);

// --- Forward Declarations for POI Modes (Functions defined later) ---
void mode_gravity_rainbow(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_spin_fire(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_centrifugal_rainbow(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_flow_trail(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_gravity_compass(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_velocity_prism(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_warp_speed(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_plasma_ghost(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_fire_ice_split(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_shifting_horizon(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_gravity_ball(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_compass_navigator(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_audio_spectrum(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_audio_wave(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_audio_bass_pulse(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_audio_motion_fusion(qmi8658_data_t *s, uint8_t *p, size_t l);      // New mode
void mode_audio_peak_color(qmi8658_data_t *s, uint8_t *p, size_t l);      // New mode
void mode_audio_rainbow_cycle(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_audio_vu_meter(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_audio_beat_fade(qmi8658_data_t *s, uint8_t *p, size_t l);
void mode_audio_frequency_lava(qmi8658_data_t *s, uint8_t *p, size_t l);


// --- Mode Names, Current Mode, Mode Table, and Mode Count (Defined early for UI and tasks) ---
static const char* mode_names[] = {
"Gravity \n Rainbow",
    "Spin \n Fire",
    "Centrifugal \n Rain",
    "Flow \n Trail",
    "Gravity \n Compass",
    "Velocity \n Prism",
    "Warp \n Speed",
    "Plasma \n Ghost",
    "Fire/Ice \n Split",
    "Shifting \n Horizon",
    "Gravity \n Ball",
    "Navigator",
    "Audio Spectrum",
    "Audio Wave",          // New mode
    "Audio Bass Pulse",     // New mode
    "Audio+Motion",         // New mode
    "Audio Peak",            // New mode
    "Audio \n Rainbow",
    "Audio \n VU Meter",
    "Audio \n Beat Fade",
    "Audio \n Lava"
};

static int current_mode = 0; // Global declaration for current mode

// Note: mode_table definition uses function pointers, so the functions need to be
//       either defined or forward-declared before this table is initialized.
//       Forward declarations are above.
poi_mode_fn mode_table[] = {
mode_gravity_rainbow,
    mode_spin_fire,
    mode_centrifugal_rainbow,
    mode_flow_trail,
    mode_gravity_compass,
    mode_velocity_prism,
    mode_warp_speed,
    mode_plasma_ghost,
    mode_fire_ice_split,
    mode_shifting_horizon,
    mode_gravity_ball,
    mode_compass_navigator,
    mode_audio_spectrum,
    mode_audio_wave,          // New mode
    mode_audio_bass_pulse,     // New mode
    mode_audio_motion_fusion,  // New mode
    mode_audio_peak_color,      // New mode
    mode_audio_rainbow_cycle,
    mode_audio_vu_meter,
    mode_audio_beat_fade,
    mode_audio_frequency_lava
};

#define MODE_COUNT (sizeof(mode_table) / sizeof(poi_mode_fn))

// Helper function to create a mode icon
static lv_obj_t *create_mode_icon(lv_obj_t *parent, int mode_idx, const char *mode_name) {
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, 75, 75); // Icon size
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x303030), 0); // Default background color
    lv_obj_set_style_border_color(btn, lv_color_make(0xFF, 0x00, 0x00), LV_STATE_CHECKED); // Red border for selected
    lv_obj_set_style_border_width(btn, 3, LV_STATE_CHECKED);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE); // Make it checkable for selection feedback
    lv_obj_set_style_text_align(btn, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, mode_name);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0); // Smaller font for icons
    lv_obj_center(label);

    // Use the static free function as the event callback
    lv_obj_add_event_cb(btn, mode_button_event_cb, LV_EVENT_CLICKED, (void *)new int(mode_idx)); // Pass mode index as user data

    return btn;
}

// Event callback for mode buttons (declared as a static free function)
static void mode_button_event_cb(lv_event_t * e) {
    lv_obj_t *btn_target = (lv_obj_t*)lv_event_get_target(e); // Added explicit cast
    int *mode_index_ptr = (int *)lv_event_get_user_data(e);
    current_mode = *mode_index_ptr;
    ESP_LOGI(TAG, "Mode button clicked: %s, setting mode to %d", mode_names[current_mode], current_mode);

    if (selected_mode_btn != NULL) {
        lv_obj_clear_state(selected_mode_btn, LV_STATE_CHECKED); // Clear previous selection
    }
    lv_obj_add_state(btn_target, LV_STATE_CHECKED); // Set new selection
    selected_mode_btn = btn_target;

    // The original mode_label update is no longer needed/relevant for the new UI
    // if (lvgl_port_lock(LVGL_PORT_LOCK_TIMEOUT_MS)) {
    //     if (mode_label != NULL) {
    //         lv_label_set_text_fmt(mode_label, "MODE: \n\n %s", mode_names[current_mode]);
    //     }
    //     lvgl_port_unlock();
    // }
}


// Forward declarations for PMU I2C callbacks
static int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);
static int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len);

// --- Global State ---
static qmi8658_dev_t imu_dev;
static uint8_t own_addr_type;
static bool is_streaming = false;
static SemaphoreHandle_t audio_spectrum_buffer_mutex; // For audio FFT

// --- PMU Global State ---
static i2c_master_bus_handle_t i2c_bus_handle = NULL; // Global handle for the shared I2C bus
static i2c_master_dev_handle_t pmu_dev_handle = NULL; // Handle for the PMU device on the bus
static XPowersPMU pmu(AXP2101_SLAVE_ADDRESS, (iic_fptr_t)pmu_register_read, (iic_fptr_t)pmu_register_write_byte);
static float battery_voltage = 0.0f;
static int battery_percentage = 0;
static bool is_charging = false;
static bool is_usb_connected = false;
static SemaphoreHandle_t pmu_data_mutex; // Mutex to protect PMU data

typedef struct {
    uint16_t conn_handle;
    uint16_t rx_char_handle;
    uint16_t tx_char_handle; // Handle for the TX characteristic
    bool discovered;
    bool stream_started;
    // POI Device Configuration
    uint8_t num_leds;
    uint8_t protocol_version;
    uint16_t frame_buffer_size;
    uint16_t hardware_buffer_limit;
    float battery_voltage; // Stored as float for display
    int free_space_kb; // Stored as int for display
    bool config_received; // Flag to indicate config has been received
} poi_device_t;

static poi_device_t devices[2] = {
    { .conn_handle = BLE_HS_CONN_HANDLE_NONE, .rx_char_handle = 0, .tx_char_handle = 0, .discovered = false, .stream_started = false,
      .num_leds = 0, .protocol_version = 0, .frame_buffer_size = 0, .hardware_buffer_limit = 0, .battery_voltage = 0.0f, .free_space_kb = 0, .config_received = false },
    { .conn_handle = BLE_HS_CONN_HANDLE_NONE, .rx_char_handle = 0, .tx_char_handle = 0, .discovered = false, .stream_started = false,
      .num_leds = 0, .protocol_version = 0, .frame_buffer_size = 0, .hardware_buffer_limit = 0, .battery_voltage = 0.0f, .free_space_kb = 0, .config_received = false }
};

static const ble_uuid128_t rx_uuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t tx_uuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

// Forward declarations for LVGL event callbacks and BLE central functions
static void gesture_event_cb(lv_event_t * e);
static void set_mic_sensitivity_low_cb(lv_event_t * e);    // New
static void set_mic_sensitivity_medium_cb(lv_event_t * e); // New
static void set_mic_sensitivity_high_cb(lv_event_t * e);   // New
static int on_disc_char(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);
static int ble_central_event(struct ble_gap_event *event, void *arg);
void poi_scan_start(void);


// --- UI Initialization Function ---
void ui_init() {
    // --- POI MODES SCREEN 1 ---
    scr_poi_modes_1 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_poi_modes_1, lv_color_black(), 0);
    lv_obj_add_flag(scr_poi_modes_1, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_poi_modes_1, gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *poi1_flex_cont = lv_obj_create(scr_poi_modes_1);
    lv_obj_set_size(poi1_flex_cont, LV_PCT(100), LV_PCT(100)); // Take full screen
    lv_obj_set_style_bg_opa(poi1_flex_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(poi1_flex_cont, 0, 0);
    lv_obj_set_flex_flow(poi1_flex_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(poi1_flex_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY); // Adjust alignment for grid
    lv_obj_set_style_pad_all(poi1_flex_cont, 5, 0); // Reduced padding
    lv_obj_center(poi1_flex_cont);

    for (int i = 0; i < 9 && i < MODE_COUNT; ++i) { // Modes 0-8 for a 3x3 grid
        create_mode_icon(poi1_flex_cont, i, mode_names[i]);
    }


    // --- POI MODES SCREEN 2 ---
    scr_poi_modes_2 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_poi_modes_2, lv_color_black(), 0);
    lv_obj_add_flag(scr_poi_modes_2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_poi_modes_2, gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *poi2_flex_cont = lv_obj_create(scr_poi_modes_2);
    lv_obj_set_size(poi2_flex_cont, LV_PCT(100), LV_PCT(100)); // Take full screen
    lv_obj_set_style_bg_opa(poi2_flex_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(poi2_flex_cont, 0, 0);
    lv_obj_set_flex_flow(poi2_flex_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(poi2_flex_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY); // Adjust alignment for grid
    lv_obj_set_style_pad_all(poi2_flex_cont, 5, 0); // Reduced padding
    lv_obj_center(poi2_flex_cont);

    for (int i = 9; i < 18 && i < MODE_COUNT; ++i) { // Modes 9-17 for a 3x3 grid
        create_mode_icon(poi2_flex_cont, i, mode_names[i]);
    }


    // --- AUDIO MODES SCREEN ---
    scr_audio_modes = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_audio_modes, lv_color_black(), 0);
    lv_obj_add_flag(scr_audio_modes, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_audio_modes, gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *audio_modes_flex_cont = lv_obj_create(scr_audio_modes);
    lv_obj_set_size(audio_modes_flex_cont, LV_PCT(100), LV_PCT(70)); // Take 70% height
    lv_obj_set_style_bg_opa(audio_modes_flex_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(audio_modes_flex_cont, 0, 0);
    lv_obj_set_flex_flow(audio_modes_flex_cont, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(audio_modes_flex_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_SPACE_EVENLY); // Adjust alignment for grid
    lv_obj_set_style_pad_all(audio_modes_flex_cont, 5, 0); // Reduced padding
    lv_obj_align(audio_modes_flex_cont, LV_ALIGN_TOP_MID, 0, 0); // Align to top-mid

    for (int i = 12; i < MODE_COUNT; ++i) { // Modes 12 to end
        create_mode_icon(audio_modes_flex_cont, i, mode_names[i]);
    }

    // Mic Sensitivity Buttons Container on Audio Screen
    lv_obj_t *btn_cont = lv_obj_create(scr_audio_modes);
    lv_obj_set_size(btn_cont, LV_PCT(100), LV_PCT(30)); // Full width, 30% height
    lv_obj_set_style_bg_opa(btn_cont, LV_OPA_TRANSP, 0); // Transparent background
    lv_obj_set_style_border_width(btn_cont, 0, 0);       // No border
    lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW);    // Arrange in a row
    lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); // Distribute evenly
    lv_obj_align(btn_cont, LV_ALIGN_BOTTOM_MID, 0, 0); // Align to bottom-mid, no offset
    lv_obj_set_style_pad_all(btn_cont, 5, 0); // Add some padding

    // Low Button
    mic_sens_low_btn = lv_btn_create(btn_cont);
    lv_obj_set_size(mic_sens_low_btn, 70, 40); // Made bigger
    lv_obj_add_event_cb(mic_sens_low_btn, set_mic_sensitivity_low_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *low_label = lv_label_create(mic_sens_low_btn);
    lv_label_set_text(low_label, "Low");
    lv_obj_center(low_label);

    // Medium Button
    mic_sens_medium_btn = lv_btn_create(btn_cont);
    lv_obj_set_size(mic_sens_medium_btn, 80, 40); // Made bigger
    lv_obj_add_event_cb(mic_sens_medium_btn, set_mic_sensitivity_medium_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *medium_label = lv_label_create(mic_sens_medium_btn);
    lv_label_set_text(medium_label, "Medium");
    lv_obj_center(medium_label);

    // High Button
    mic_sens_high_btn = lv_btn_create(btn_cont);
    lv_obj_set_size(mic_sens_high_btn, 70, 40); // Made bigger
    lv_obj_add_event_cb(mic_sens_high_btn, set_mic_sensitivity_high_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *high_label = lv_label_create(mic_sens_high_btn);
    lv_label_set_text(high_label, "High");
    lv_obj_center(high_label);

    // Set initial button colors based on default sensitivity
    lv_obj_set_style_bg_color(mic_sens_low_btn, lv_color_hex(0x404040), 0);
    lv_obj_set_style_bg_color(mic_sens_medium_btn, lv_color_make(0xFF, 0x00, 0x00), 0); // Default to red
    lv_obj_set_style_bg_color(mic_sens_high_btn, lv_color_hex(0x404040), 0);


    // --- SYSTEM INFO SCREEN ---
    scr_system_info = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr_system_info, lv_color_hex(0x000020), 0); // Dark Blueish
    lv_obj_add_flag(scr_system_info, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(scr_system_info, gesture_event_cb, LV_EVENT_GESTURE, NULL);

    lv_obj_t *sys_info_cont = lv_obj_create(scr_system_info);
    lv_obj_set_size(sys_info_cont, lv_pct(95), lv_pct(95));
    lv_obj_set_style_bg_opa(sys_info_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(sys_info_cont, 0, 0);
    lv_obj_set_flex_flow(sys_info_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(sys_info_cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(sys_info_cont, 5, 0);
    lv_obj_center(sys_info_cont);

    // Battery Status (PMU)
    battery_label = lv_label_create(sys_info_cont);
    lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_26, 0);
    lv_label_set_text(battery_label, "BAT: --V --% (---)"); 
    lv_obj_set_width(battery_label, lv_pct(100));
    lv_obj_set_style_text_align(battery_label, LV_TEXT_ALIGN_CENTER, 0);


    // POI Info Box
    poi_info_box = lv_obj_create(sys_info_cont);
    lv_obj_set_size(poi_info_box, lv_pct(90), 80);
    lv_obj_set_style_bg_color(poi_info_box, lv_color_hex(0x202020), 0);
    lv_obj_set_style_pad_all(poi_info_box, 5, 0);
    lv_obj_set_style_align(poi_info_box, LV_ALIGN_CENTER, 0);

    poi_info_label = lv_label_create(poi_info_box);
    lv_label_set_text(poi_info_label, "Connecting...");
    lv_obj_set_style_text_align(poi_info_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(poi_info_label, &lv_font_montserrat_18, 0);
    lv_obj_center(poi_info_label);


    // Clock Info
    clock_time_label = lv_label_create(sys_info_cont);
    lv_obj_set_style_text_font(clock_time_label, &lv_font_montserrat_40, 0);
    lv_obj_set_width(clock_time_label, lv_pct(100));
    lv_obj_set_style_text_align(clock_time_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(clock_time_label, "00:00:00");

    clock_date_label = lv_label_create(sys_info_cont);
    lv_obj_set_width(clock_date_label, lv_pct(100));
    lv_obj_set_style_text_align(clock_date_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(clock_date_label, "Mon, Jan 01 2000");

    clock_unix_label = lv_label_create(sys_info_cont);
    lv_obj_set_width(clock_unix_label, lv_pct(100));
    lv_obj_set_style_text_align(clock_unix_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(clock_unix_label, &lv_font_montserrat_26, 0); // Make it bigger
    lv_obj_set_style_text_color(clock_unix_label, lv_color_hex(0x00FF00), 0); // Matrix green
    lv_label_set_text(clock_unix_label, "UNIX: 0");

    // Initial screen load
    lv_scr_load(scr_poi_modes_1);
}



static void gesture_event_cb(lv_event_t * e) {
    lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
    // lv_obj_t * current_lv_screen = lv_scr_act(); // Unused, can be removed

    if (lvgl_port_lock(LVGL_PORT_LOCK_TIMEOUT_MS)) {
        if (dir == LV_DIR_LEFT) {
            current_app_screen = (app_screen_t)((current_app_screen + 1) % NUM_SCREENS);
            ESP_LOGI(TAG, "Swiped Left, Screen Switched to: %d", current_app_screen);
        } else if (dir == LV_DIR_RIGHT) {
            current_app_screen = (app_screen_t)((current_app_screen - 1 + NUM_SCREENS) % NUM_SCREENS); // Handle negative modulo
            ESP_LOGI(TAG, "Swiped Right, Screen Switched to: %d", current_app_screen);
        } else {
            lvgl_port_unlock();
            return; // Not a horizontal swipe
        }

        // Load the new screen
        switch (current_app_screen) {
            case SCREEN_POI_MODES_1:
                lv_scr_load(scr_poi_modes_1);
                break;
            case SCREEN_POI_MODES_2:
                lv_scr_load(scr_poi_modes_2);
                break;
            case SCREEN_AUDIO_MODES:
                lv_scr_load(scr_audio_modes);
                break;
            case SCREEN_SYSTEM_INFO:
                lv_scr_load(scr_system_info);
                break;
            default:
                // Should not happen
                break;
        }
        lvgl_port_unlock();
    }
}

static void set_mic_sensitivity_low_cb(lv_event_t * e) {
    current_mic_sensitivity = mic_sensitivity_low;

    // Set active button to red
    lv_obj_set_style_bg_color(mic_sens_low_btn, lv_color_make(0xFF, 0x00, 0x00), 0);
    // Set inactive buttons to default color
    lv_obj_set_style_bg_color(mic_sens_medium_btn, lv_color_hex(0x404040), 0);
    lv_obj_set_style_bg_color(mic_sens_high_btn, lv_color_hex(0x404040), 0);

    ESP_LOGI(TAG, "Mic Sensitivity set to LOW (%.1fx)", current_mic_sensitivity);
}

static void set_mic_sensitivity_medium_cb(lv_event_t * e) {
    current_mic_sensitivity = mic_sensitivity_medium;

    // Set active button to red
    lv_obj_set_style_bg_color(mic_sens_medium_btn, lv_color_make(0xFF, 0x00, 0x00), 0);
    // Set inactive buttons to default color
    lv_obj_set_style_bg_color(mic_sens_low_btn, lv_color_hex(0x404040), 0);
    lv_obj_set_style_bg_color(mic_sens_high_btn, lv_color_hex(0x404040), 0);

    ESP_LOGI(TAG, "Mic Sensitivity set to MEDIUM (%.1fx)", current_mic_sensitivity);
}

static void set_mic_sensitivity_high_cb(lv_event_t * e) {
    current_mic_sensitivity = mic_sensitivity_high;

    // Set active button to red
    lv_obj_set_style_bg_color(mic_sens_high_btn, lv_color_make(0xFF, 0x00, 0x00), 0);
    // Set inactive buttons to default color
    lv_obj_set_style_bg_color(mic_sens_low_btn, lv_color_hex(0x404040), 0);
    lv_obj_set_style_bg_color(mic_sens_medium_btn, lv_color_hex(0x404040), 0);

    ESP_LOGI(TAG, "Mic Sensitivity set to HIGH (%.1fx)", current_mic_sensitivity);
}



// --- PMU I2C Functions ---

// I2C init - Adds PMU device to an existing bus
esp_err_t i2c_init_pmu_device(i2c_master_bus_handle_t main_bus_handle) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP2101_SLAVE_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0
        }
    };
    // Use the main_bus_handle to add the PMU device
    esp_err_t ret = i2c_master_bus_add_device(main_bus_handle, &dev_config, &pmu_dev_handle);
    if (ret == ESP_OK) {
        // Also assign the main_bus_handle to global i2c_bus_handle for consistency if needed
        i2c_bus_handle = main_bus_handle;
    }
    return ret;
}

// PMU read function using new API
static int pmu_register_read(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    esp_err_t ret = i2c_master_transmit_receive(pmu_dev_handle, &regAddr, 1, data, len, I2C_MASTER_TIMEOUT_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMU READ FAILED! Addr:0x%02x, Reg:0x%02x, Error:%d", devAddr, regAddr, ret);
        return -1;
    }
    return 0;
}

// PMU write function using new API
static int pmu_register_write_byte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint8_t len) {
    uint8_t *buffer = (uint8_t *)malloc(len + 1);
    if (!buffer) return -1;
    buffer[0] = regAddr;
    memcpy(&buffer[1], data, len);

    esp_err_t ret = i2c_master_transmit(pmu_dev_handle, buffer, len + 1, I2C_MASTER_TIMEOUT_MS);
    free(buffer);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMU WRITE FAILED! Addr:0x%02x, Reg:0x%02x, Error:%d", devAddr, regAddr, ret);
        return -1;
    }
    return 0;
}

// --- Helper: HSV to RGB ---
void hsv_to_rgb(uint8_t h_in, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint16_t h_scaled = h_in * 3;
    if (h_scaled < 255) { *r = 255 - h_scaled; *g = h_scaled; *b = 0; }
    else if (h_scaled < 510) { h_scaled -= 255; *r = 0; *g = 255 - h_scaled; *b = h_scaled; }
    else { h_scaled -= 510; *r = h_scaled; *g = 0; *b = 255 - h_scaled; }
}

// =============================================================================
// POI MODE FUNCTION DEFINITIONS
// =============================================================================
void mode_gravity_rainbow(qmi8658_data_t *s, uint8_t *p, size_t l) {
    float angle = atan2f(s->accelY, s->accelX);
    uint8_t r, g, b;
    hsv_to_rgb((uint8_t)(((angle + M_PI) / (2.0f * M_PI)) * 255.0f), &r, &g, &b);
    for (int i = 0; i < l; i += 3) { p[i] = r; p[i+1] = g; p[i+2] = b; }
}

void mode_spin_fire(qmi8658_data_t *s, uint8_t *p, size_t l) {
    // Increase sensitivity: Divide by 4 instead of 64
    // Add +40 so it's always a little bit visible even when still
    float val = (fabs(s->gyroZ) / 4.0f) + 40.0f;
    uint8_t intensity = (uint8_t)fminf(val, 255.0f);

    for (int i = 0; i < l; i += 3) {
        p[i] = intensity;          // Red
        p[i+1] = intensity / 3;    // Orange tint (slightly more than /4)
        p[i+2] = 0;
    }
}


void mode_centrifugal_rainbow(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float hue = 0;
    // Lowered divisor from 2000 to 500 for more "pop"
    hue += fabs(s->gyroZ) / 500.0f;

    if (hue >= 255) hue -= 255; // Use subtraction to keep it smooth
    for (int i = 0; i < l; i += 3) {
        uint8_t r, g, b;
        // Adding (i/3) creates a slight gradient across the 12 LEDs
        hsv_to_rgb((uint8_t)hue + (i * 2), &r, &g, &b);
        p[i] = r; p[i+1] = g; p[i+2] = b;
    }
}

void mode_flow_trail(qmi8658_data_t *s, uint8_t *p, size_t l) {
    // Brightness based on spin speed
    uint8_t br = (uint8_t)fminf(fabs(s->gyroZ) / 4.0f, 255.0f);
    for (int i = 0; i < l; i += 3) { p[i] = 0; p[i+1] = br; p[i+2] = br; }
}

void mode_gravity_compass(qmi8658_data_t *s, uint8_t *p, size_t l) {
    float angle = atan2f(s->accelY, s->accelX);
    uint8_t hue = (uint8_t)(((angle + M_PI) / (2.0f * M_PI)) * 255.0f);
    for (int i = 0; i < l; i += 3) {
        uint8_t r, g, b; hsv_to_rgb(hue + (i/3), &r, &g, &b);
        p[i] = r; p[i+1] = g; p[i+2] = b;
    }
}

void mode_velocity_prism(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float smoothed_vel = 0.0f;
    static float hue_offset = 0.0f;

    float current_vel = sqrtf(s->gyroX * s->gyroX + s->gyroY * s->gyroY + s->gyroZ * s->gyroZ); // Use all gyro axes for velocity

    // Smooth the velocity to reduce flashiness
    smoothed_vel = smoothed_vel * 0.9f + current_vel * 0.1f; // Exponential moving average

    // Map smoothed_vel to a dynamic range for hue, making it less direct and more subtle
    // Max vel could be around 2000-3000, so a divisor of 100.0f will give 0-30 hue range
    float dynamic_hue_base = fminf(smoothed_vel / 50.0f, 255.0f); // Adjust divisor for desired sensitivity

    // Slowly shift hue offset for a flowing effect
    hue_offset += 0.2f + (dynamic_hue_base * 0.05f); // Faster shift with higher velocity
    if (hue_offset >= 255.0f) hue_offset -= 255.0f;

    for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
        uint8_t r, g, b;
        float led_pos_norm = (float)led_idx / (NUM_LEDS - 1);

        // Create a subtle wave or gradient across the LEDs, influenced by hue_offset and velocity
        uint8_t hue = (uint8_t)fmodf(dynamic_hue_base + (led_pos_norm * 50.0f) + hue_offset, 255.0f); // 50.0f for spread

        // Brightness can also be influenced by smoothed velocity, but gently
        float brightness = 0.4f + fminf(smoothed_vel / 1000.0f, 0.6f); // Base brightness with gentle boost from velocity
        brightness = fmaxf(MIN_BRIGHTNESS, fminf(1.0f, brightness));

        hsv_to_rgb(hue, &r, &g, &b);

        int p_idx = led_idx * 3;
        p[p_idx] = (uint8_t)(r * brightness);
        p[p_idx+1] = (uint8_t)(g * brightness);
        p[p_idx+2] = (uint8_t)(b * brightness);
    }
}

void mode_warp_speed(qmi8658_data_t *s, uint8_t *p, size_t l) {
    // Brightness flashes based on Gyro Z (spin speed)
    // Creates a "strobe" effect the faster you spin
    uint8_t flash = (uint8_t)fminf(fabs(s->gyroZ) / 5.0f, 255.0f);
    for (int i = 0; i < l; i += 3) {
        p[i] = flash; p[i+1] = flash; p[i+2] = flash;
    }
}

void mode_plasma_ghost(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float global_hue = 0.0f;
    static float plasma_seed = 0.0f; // For subtle, organic movement

    // Global hue shifts slowly, creating a "breathing" color effect
    global_hue += 0.1f;
    if (global_hue >= 255.0f) global_hue -= 255.0f;

    // Plasma movement seed (subtle variation)
    plasma_seed += 0.01f;

    // Overall acceleration magnitude for intensity
    float accel_mag = sqrtf(s->accelX*s->accelX + s->accelY*s->accelY + s->accelZ*s->accelZ);
    float intensity_boost = fminf(accel_mag / 10.0f, 1.0f); // Boost intensity with movement

    for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
        uint8_t r, g, b;
        float led_pos_norm = (float)led_idx / (NUM_LEDS - 1);

        // Base hue for the "plasma" effect, influenced by global_hue and LED position
        uint8_t base_hue = (uint8_t)fmodf(global_hue + (led_pos_norm * 80.0f) + sinf(plasma_seed + led_pos_norm * M_PI * 2.0f) * 20.0f, 255.0f);

        // A "ghostly" complementary color that blends in
        uint8_t secondary_hue = (uint8_t)fmodf(base_hue + 120.0f, 255.0f);

        uint8_t r_base, g_base, b_base;
        hsv_to_rgb(base_hue, &r_base, &g_base, &b_base);

        uint8_t r_sec, g_sec, b_sec;
        hsv_to_rgb(secondary_hue, &r_sec, &g_sec, &b_sec);

        // Blend the colors, with blending factor subtly varied by a sine wave for "plasma" feel
        float blend_factor = (sinf(plasma_seed * 2.0f + led_pos_norm * M_PI * 4.0f) + 1.0f) / 2.0f; // 0 to 1

        r = (uint8_t)(r_base * (1.0f - blend_factor) + r_sec * blend_factor);
        g = (uint8_t)(g_base * (1.0f - blend_factor) + g_sec * blend_factor);
        b = (uint8_t)(b_base * (1.0f - blend_factor) + b_sec * blend_factor);

        // Overall brightness, with a boost from movement
        float brightness = 0.3f + intensity_boost * 0.7f;
        brightness = fmaxf(MIN_BRIGHTNESS, fminf(1.0f, brightness));

        int p_idx = led_idx * 3;
        p[p_idx] = (uint8_t)(r * brightness);
        p[p_idx+1] = (uint8_t)(g * brightness);
        p[p_idx+2] = (uint8_t)(b * brightness);
    }
}

void mode_fire_ice_split(qmi8658_data_t *s, uint8_t *p, size_t l) {
    float tilt_norm = (s->accelY + 1.0f) / 2.0f; // Normalized tilt from 0.0 to 1.0

    for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
        uint8_t r, g, b;
        float led_pos_norm = (float)led_idx / (NUM_LEDS - 1); // Normalized LED position 0.0 to 1.0

        // Determine the "balance" between fire and ice for this LED
        // Create a blend zone that moves with tilt_norm
        float blend_width = 0.3f; // Width of the blend zone
        float blend_start = tilt_norm - (blend_width / 2.0f);
        float blend_end = tilt_norm + (blend_width / 2.0f);

        float fire_factor; // How much fire color should contribute (0 to 1)
        if (led_pos_norm < blend_start) { // Pure ice
            fire_factor = 0.0f;
        } else if (led_pos_norm > blend_end) { // Pure fire
            fire_factor = 1.0f;
        } else { // In the blend zone
            fire_factor = (led_pos_norm - blend_start) / blend_width;
        }

        // Fire hues (red to yellow/orange, maybe some white/bright in the middle)
        // Base hue for fire part, slightly shifted by led position
        uint8_t fire_hue = (uint8_t)(led_pos_norm * 40.0f); // Red (0) to Orange/Yellow (40)

        // Ice hues (blue to cyan/white)
        // Base hue for ice part, slightly shifted by led position
        uint8_t ice_hue = (uint8_t)(180 + (1.0f - led_pos_norm) * 40.0f); // Blue (180) to Cyan (220)

        uint8_t r_src_fire, g_src_fire, b_src_fire;
        hsv_to_rgb(fire_hue, &r_src_fire, &g_src_fire, &b_src_fire);

        uint8_t r_src_ice, g_src_ice, b_src_ice;
        hsv_to_rgb(ice_hue, &r_src_ice, &g_src_ice, &b_src_ice);

        // Blend the colors based on fire_factor
        r = (uint8_t)(r_src_ice * (1.0f - fire_factor) + r_src_fire * fire_factor);
        g = (uint8_t)(g_src_ice * (1.0f - fire_factor) + g_src_fire * fire_factor);
        b = (uint8_t)(b_src_ice * (1.0f - fire_factor) + b_src_fire * fire_factor);

        int p_idx = led_idx * 3;
        p[p_idx] = r;
        p[p_idx+1] = g;
        p[p_idx+2] = b;
    }
}

void mode_shifting_horizon(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float hue_offset = 0.0f;
    static float horizon_pos_smoothed = 0.5f; // Normalized position of the horizon line

    // Map accelZ (-1.0 to 1.0) to a normalized horizon position (0.0 to 1.0)
    float target_horizon_pos = (s->accelZ + 1.0f) / 2.0f;

    // Smooth the horizon position for less jittery movement
    horizon_pos_smoothed = horizon_pos_smoothed * 0.9f + target_horizon_pos * 0.1f;

    // Slowly shift overall hue for dynamic colors
    hue_offset += 0.08f;
    if (hue_offset >= 255.0f) hue_offset -= 255.0f;

    for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
        uint8_t r, g, b;
        float led_pos_norm = (float)led_idx / (NUM_LEDS - 1); // 0.0 to 1.0

        // Calculate distance from the smoothed horizon line
        float dist_from_horizon = fabsf(led_pos_norm - horizon_pos_smoothed);

        // Intensity/brightness based on proximity to the horizon
        float proximity_intensity = 1.0f - fminf(dist_from_horizon * 4.0f, 1.0f); // Peak at horizon, falloff

        // Create a color gradient that shifts across the horizon
        // Hue varies across the strip, with a bias towards the global_hue_offset
        uint8_t hue = (uint8_t)fmodf(hue_offset + (led_pos_norm * 120.0f), 255.0f);
        hsv_to_rgb(hue, &r, &g, &b);

        // Apply brightness: higher near the horizon, with a base minimum
        float brightness = fmaxf(MIN_BRIGHTNESS, 0.2f + proximity_intensity * 0.8f);
        brightness = fminf(1.0f, brightness); // Clamp to 1.0

        int p_idx = led_idx * 3;
        p[p_idx] = (uint8_t)(r * brightness);
        p[p_idx+1] = (uint8_t)(g * brightness);
        p[p_idx+2] = (uint8_t)(b * brightness);
    }
}

void mode_gravity_ball(qmi8658_data_t *s, uint8_t *p, size_t l) {
    memset(p, 0, l);

    // 1. Normalize the input. If it's > 10, assume it's in milli-Gs (1000)
    float ay = s->accelY;
    if (fabsf(ay) > 10.0f) ay /= 1000.0f;

    // 2. Clamp the value to the expected range [-1.0, 1.0]
    if (ay > 1.0f) ay = 1.0f;
    if (ay < -1.0f) ay = -1.0f;

    // 3. Map -1.0..1.0 to 0.0..1.0
    float pos = (ay + 1.0f) / 2.0f;

    // 4. Calculate LED index (0 to 11)
    int led_idx = (int)(pos * NUM_LEDS -1);
    int p_idx = led_idx * 3; // 3 bytes per pixel

    // Safety check and light it up
    if (p_idx <= l - 3) {
        p[p_idx] = 255;
        p[p_idx+1] = 255;
        p[p_idx+2] = 255;
    }
}

void mode_compass_navigator(qmi8658_data_t *s, uint8_t *p, size_t l) {
    float angle = atan2f(s->accelY, s->accelX); // -PI to PI
    float angle_norm = (angle + M_PI) / (2.0f * M_PI); // 0.0 to 1.0

    // Map the normalized angle to a target LED position (0 to NUM_LEDS-1)
    float target_led_pos = angle_norm * (NUM_LEDS - 1);

    // Dynamic glow/pulse based on spin speed
    float spin_intensity = fminf(fabs(s->gyroZ) / 50.0f, 1.0f); // Normalize gyroZ to 0..1

    for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
        uint8_t r, g, b;
        float led_pos_norm = (float)led_idx / (NUM_LEDS - 1); // 0.0 to 1.0

        // Calculate distance from target LED position, wrapping around the strip
        float dist = fabsf(led_idx - target_led_pos);
        if (dist > NUM_LEDS / 2.0f) {
            dist = NUM_LEDS - dist; // Handle wrap-around for shortest distance
        }
        float proximity_factor = 1.0f - fminf(dist / (NUM_LEDS / 4.0f), 1.0f); // 1.0 at target, 0.0 further away

        // Base hue shifts slowly, perhaps based on time or a slow cycle, to make it more interesting
        static float global_hue_offset = 0.0f;
        global_hue_offset += 0.05f; // Slow rotation
        if (global_hue_offset >= 255.0f) global_hue_offset -= 255.0f;

        // Determine hue: a base color, blended with a "pointer" color at the target_led_pos
        uint8_t base_hue = (uint8_t)fmodf(led_pos_norm * 255.0f + global_hue_offset, 255.0f);
        uint8_t pointer_hue = (uint8_t)fmodf(base_hue + 120.0f, 255.0f); // Complementary hue for the pointer

        uint8_t r_base, g_base, b_base;
        hsv_to_rgb(base_hue, &r_base, &g_base, &b_base);

        uint8_t r_pointer, g_pointer, b_pointer;
        hsv_to_rgb(pointer_hue, &r_pointer, &g_pointer, &b_pointer);

        // Blend base and pointer colors based on proximity
        r = (uint8_t)(r_base * (1.0f - proximity_factor) + r_pointer * proximity_factor);
        g = (uint8_t)(g_base * (1.0f - proximity_factor) + g_pointer * proximity_factor);
        b = (uint8_t)(b_base * (1.0f - proximity_factor) + b_pointer * proximity_factor);

        // Apply brightness: higher near the pointer, and a boost from spin intensity
        float brightness = 0.5f + proximity_factor * 0.5f + spin_intensity * 0.3f;
        brightness = fmaxf(MIN_BRIGHTNESS, fminf(1.0f, brightness));

        int p_idx = led_idx * 3;
        p[p_idx] = (uint8_t)(r * brightness);
        p[p_idx+1] = (uint8_t)(g * brightness);
        p[p_idx+2] = (uint8_t)(b * brightness);
    }
}

void mode_audio_spectrum(qmi8658_data_t *s, uint8_t *p, size_t l) {
    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        int num_spectrum_bins = N_SAMPLES / 2; // This is 8 (N_SAMPLES = 16)

        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            float target_spectrum_pos = (float)led_idx / (NUM_LEDS - 1) * (num_spectrum_bins - 1);
            int spectrum_idx_low = (int)floorf(target_spectrum_pos);
            int spectrum_idx_high = (int)ceilf(target_spectrum_pos);
            float interp_factor = target_spectrum_pos - spectrum_idx_low;

            float mag_low = spectrum[spectrum_idx_low];
            float mag_high = spectrum[spectrum_idx_high];
            float interpolated_magnitude = mag_low * (1.0f - interp_factor) + mag_high * interp_factor;

            // **Adjust dB range for higher sensitivity to lower sounds**
            float normalized_magnitude = (interpolated_magnitude + 70.0f) / 70.0f; // Shift range from -70 to 0dB
            normalized_magnitude = fmaxf(0.0f, fminf(1.0f, normalized_magnitude));

            // **Even stronger baseline and audio reaction**
            float effective_brightness = fmaxf(MIN_BRIGHTNESS * 2.0f, normalized_magnitude * 1.2f + MIN_BRIGHTNESS * 1.0f); // Even higher floor and stronger audio scaling

            uint8_t r, g, b;
            // Map magnitude to hue: 0 (red) -> 85 (green) -> 170 (blue) for low to high magnitude
            // Invert hue so low magnitude is blue, high is red (red is 0, so 170 - hue_val)
            uint8_t hue_val = (uint8_t)(normalized_magnitude * 220.0f); // Even wider hue range for more color diversity
            hsv_to_rgb(170 - hue_val, &r, &g, &b);

            // Apply effective brightness
            r = (uint8_t)(r * effective_brightness);
            g = (uint8_t)(g * effective_brightness);
            b = (uint8_t)(b * effective_brightness);

            int p_idx = led_idx * 3;
            if (p_idx <= l - 3) {
                p[p_idx] = r;
                p[p_idx+1] = g;
                p[p_idx+2] = b;
            }
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}

void mode_audio_wave(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float wave_phase = 0.0f; // Use phase for smoother wave motion
    static float hue_offset = 0.0f; // Global hue offset for color diversity

    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        float sum_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_amplitude = sum_amplitude / N_SAMPLES;

        // **Even much higher sensitivity: Multiplier 15.0f**
        float normalized_amplitude = fminf(avg_amplitude * 15.0f, 1.0f);

        // Base brightness, boosted even more strongly by amplitude
        float effective_brightness = fmaxf(MIN_BRIGHTNESS * 2.0f, normalized_amplitude * 1.5f + MIN_BRIGHTNESS * 1.0f); // Higher floor, stronger audio impact

        // Hue changes over time, influenced by amplitude (faster change with louder audio) and motion
        hue_offset += (1.0f + normalized_amplitude * 4.0f + fabs(s->gyroZ) / 200.0f); // Faster global hue shift, more motion influence
        if (hue_offset >= 255.0f) hue_offset -= 255.0f;

        // Wave motion influenced more strongly by audio amplitude
        wave_phase += (0.2f + normalized_amplitude * 2.0f); // Faster wave with louder audio

        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            uint8_t r, g, b;

            // Create a wave pattern: sine wave along the strip
            // Wave amplitude and frequency influenced by audio more intensely
            float wave_amplitude = 0.4f + normalized_amplitude * 0.6f; // More dynamic wave peaks
            float wave_frequency = 1.0f + normalized_amplitude * 0.7f; // More compression with louder audio

            float wave_value = sinf((float)led_idx / (NUM_LEDS - 1) * M_PI * wave_frequency + wave_phase) * wave_amplitude;
            wave_value = (wave_value + 1.0f) / 2.0f; // Map -1 to 1 to 0 to 1

            // Combine global hue, wave value, and led position for color diversity
            // Added current_pixel_brightness into hue calculation for more color diversity
            uint8_t hue = (uint8_t)fmodf(hue_offset + (wave_value * 120.0f) + ((float)led_idx / NUM_LEDS * 50.0f), 255.0f);

            hsv_to_rgb(hue, &r, &g, &b);

            // Brightness affected by wave value and audio amplitude
            float pixel_brightness = effective_brightness * (0.5f + wave_value * 0.5f);
            pixel_brightness = fmaxf(MIN_BRIGHTNESS, pixel_brightness);
            if (pixel_brightness > 1.0f) pixel_brightness = 1.0f;

            p[led_idx * 3]     = (uint8_t)(r * pixel_brightness);
            p[led_idx * 3 + 1] = (uint8_t)(g * pixel_brightness);
            p[led_idx * 3 + 2] = (uint8_t)(b * pixel_brightness);
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}


void mode_audio_bass_pulse(qmi8658_data_t *s, uint8_t *p, size_t l) {
    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        // Average the lowest few frequency bins for bass
        float bass_magnitude_sum = 0.0f;
        int num_bass_bins = 3;
        for (int i = 0; i < num_bass_bins; i++) {
            bass_magnitude_sum += spectrum[i];
        }
        float avg_bass_magnitude = bass_magnitude_sum / num_bass_bins;
        float normalized_bass = (avg_bass_magnitude + 65.0f) / 65.0f; // Slightly more sensitive bass range
        normalized_bass = fmaxf(0.0f, fminf(1.0f, normalized_bass));

        // Analyze mid and higher frequency bins for nuanced high tones/melody
        float mid_magnitude_sum = 0.0f;
        float treble_magnitude_sum = 0.0f;
        float max_treble_magnitude = -100.0f;
        int peak_treble_bin = num_bass_bins;

        int num_mid_bins_start = num_bass_bins;
        int num_mid_bins_end = num_bass_bins + (N_SAMPLES / 2 - num_bass_bins) / 2; // Middle half of remaining bins

        int num_treble_bins_start = num_mid_bins_end;
        int num_treble_bins_end = N_SAMPLES / 2;

        for (int i = num_mid_bins_start; i < num_mid_bins_end; i++) {
            mid_magnitude_sum += spectrum[i];
        }
        for (int i = num_treble_bins_start; i < num_treble_bins_end; i++) {
            treble_magnitude_sum += spectrum[i];
            if (spectrum[i] > max_treble_magnitude) {
                max_treble_magnitude = spectrum[i];
                peak_treble_bin = i;
            }
        }

        float avg_mid_magnitude = (num_mid_bins_end - num_mid_bins_start > 0) ? (mid_magnitude_sum / (num_mid_bins_end - num_mid_bins_start)) : 0.0f;
        float avg_treble_magnitude = (num_treble_bins_end - num_treble_bins_start > 0) ? (treble_magnitude_sum / (num_treble_bins_end - num_treble_bins_start)) : 0.0f;

        float normalized_mid_avg = (avg_mid_magnitude + 65.0f) / 65.0f;
        normalized_mid_avg = fmaxf(0.0f, fminf(1.0f, normalized_mid_avg));

        float normalized_treble_avg = (avg_treble_magnitude + 65.0f) / 65.0f;
        normalized_treble_avg = fmaxf(0.0f, fminf(1.0f, normalized_treble_avg));

        float normalized_treble_peak_val = (max_treble_magnitude + 65.0f) / 65.0f;
        normalized_treble_peak_val = fmaxf(0.0f, fminf(1.0f, normalized_treble_peak_val));

        float sum_total_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_total_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_total_amplitude = sum_total_amplitude / N_SAMPLES;
        float normalized_amplitude = fminf(avg_total_amplitude * 15.0f, 1.0f); // Adjust multiplier as needed


        // Base brightness always present, boosted by all frequency components
        float effective_base_brightness = fmaxf(MIN_BRIGHTNESS * 2.0f, normalized_bass * 0.6f + normalized_mid_avg * 0.3f + normalized_treble_avg * 0.2f + MIN_BRIGHTNESS * 1.5f);


        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            uint8_t r, g, b;
            float current_pixel_brightness = effective_base_brightness;

            // Base color for bass: red/orange, pulsating with bass intensity
            uint8_t hue_bass = (uint8_t)(normalized_bass * 60.0f); // Red (0) to Yellow (60) for more range
            hsv_to_rgb(hue_bass, &r, &g, &b);

            // Mid tones add a different nuance (e.g., green/yellow)
            if (normalized_mid_avg > 0.1f) {
                float mid_influence_factor = normalized_mid_avg * 0.9f;
                uint8_t mid_hue = (uint8_t)(60 + normalized_mid_avg * 30); // Yellow to Greenish
                uint8_t mr, mg, mb;
                hsv_to_rgb(mid_hue, &mr, &mg, &mb);

                r = (uint8_t)(r * (1.0f - mid_influence_factor) + mr * mid_influence_factor);
                g = (uint8_t)(g * (1.0f - mid_influence_factor) + mg * mid_influence_factor);
                b = (uint8_t)(b * (1.0f - mid_influence_factor) + mb * mid_influence_factor);
                current_pixel_brightness = fmaxf(current_pixel_brightness, mid_influence_factor);
            }

            // High tones add a distinct color (e.g., blue/purple)
            if (normalized_treble_avg > 0.1f) {
                float treble_influence_factor = normalized_treble_avg * 1.0f;

                uint8_t treble_hue = (uint8_t)(((float)(peak_treble_bin - num_treble_bins_start) / (num_treble_bins_end - num_treble_bins_start)) * 90.0f + 180); // Blue to Magenta range

                uint8_t tr, tg, tb;
                hsv_to_rgb(treble_hue, &tr, &tg, &tb);

                r = (uint8_t)(r * (1.0f - treble_influence_factor) + tr * treble_influence_factor);
                g = (uint8_t)(g * (1.0f - treble_influence_factor) + tg * treble_influence_factor);
                b = (uint8_t)(b * (1.0f - treble_influence_factor) + tb * treble_influence_factor);
                current_pixel_brightness = fmaxf(current_pixel_brightness, treble_influence_factor);
            }

            // Apply slight "sparkle" or intensity boost for very strong high-frequency peaks
            if (normalized_treble_peak_val > 0.5f) {
                float sparkle_intensity = normalized_treble_peak_val * 0.8f;
                // Localize sparkle based on LED position relative to peak_treble_bin
                float peak_pos_norm = (float)peak_treble_bin / (N_SAMPLES / 2 - 1); // 0 to 1
                float led_pos_norm = (float)led_idx / (NUM_LEDS - 1);
                float distance_from_treble_peak = fabsf(led_pos_norm - peak_pos_norm);

                sparkle_intensity *= (1.0f - distance_from_treble_peak * 2.0f); // Falloff
                sparkle_intensity = fmaxf(0.0f, sparkle_intensity);

                current_pixel_brightness = fmaxf(current_pixel_brightness, sparkle_intensity);
            }


            // Apply final brightness and ensure minimum light
            float final_pixel_brightness = fmaxf(MIN_BRIGHTNESS, current_pixel_brightness * (0.7f + normalized_amplitude * 0.3f)); // Overall amplitude for final boost
            if (final_pixel_brightness > 1.0f) final_pixel_brightness = 1.0f;

            p[led_idx * 3]     = (uint8_t)(r * final_pixel_brightness);
            p[led_idx * 3 + 1] = (uint8_t)(g * final_pixel_brightness);
            p[led_idx * 3 + 2] = (uint8_t)(b * final_pixel_brightness);
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}

void mode_audio_motion_fusion(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float global_hue_cycle = 0.0f;
    static float motion_flow_speed = 0.0f;
    static float last_accel_magnitude = 0.0f; // For subtle acceleration-based color shifts

    // Calculate overall acceleration magnitude for motion reactivity
    float current_accel_magnitude = sqrtf(s->accelX*s->accelX + s->accelY*s->accelY + s->accelZ*s->accelZ);
    float delta_accel_magnitude = fabs(current_accel_magnitude - last_accel_magnitude);
    last_accel_magnitude = current_accel_magnitude;

    // Smoothed gyroscope Z for rotation influence
    float smoothed_gyro_z = fabs(s->gyroZ) / 50.0f; // Stronger influence from spin

    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        float sum_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_amplitude = sum_amplitude / N_SAMPLES;

        // **Highly sensitive audio reaction**
        float audio_reactivity = fminf(avg_amplitude * 20.0f, 1.0f);

        // Base brightness always present, with a high floor, and highly boosted by audio
        float base_brightness = fmaxf(MIN_BRIGHTNESS * 3.0f, audio_reactivity * 1.0f + MIN_BRIGHTNESS * 1.5f);

        // Motion influences global hue cycle speed and a secondary pattern
        global_hue_cycle += (0.1f + smoothed_gyro_z * 0.5f); // Spin speeds up hue cycle
        if (global_hue_cycle >= 255.0f) global_hue_cycle -= 255.0f;

        // Motion flow influenced by gyro (speed) and accel (jerkiness)
        motion_flow_speed = fminf(2.0f, 0.1f + smoothed_gyro_z * 0.3f + delta_accel_magnitude * 5.0f);

        for (int i = 0; i < l; i += 3) {
            uint8_t r, g, b;
            float led_pos_norm = (float)(i / 3) / (NUM_LEDS - 1);

            // Core pattern: a flowing, motion-driven color gradient
            uint8_t base_pattern_hue = (uint8_t)fmodf(global_hue_cycle + (led_pos_norm * 150.0f) + (sinf(led_pos_norm * M_PI * 4.0f + motion_flow_speed) * 30.0f), 255.0f);

            // Audio layers on top, influencing a secondary color pulse or shift
            uint8_t audio_layer_hue = (uint8_t)fmodf(base_pattern_hue + 90.0f, 255.0f); // Complementary or shifted hue

            // Interpolate between base and audio layer based on audio reactivity
            uint8_t final_hue;
            if (audio_reactivity > 0.1f) {
                final_hue = (uint8_t)(base_pattern_hue * (1.0f - audio_reactivity) + audio_layer_hue * audio_reactivity);
            } else {
                final_hue = base_pattern_hue;
            }

            hsv_to_rgb(final_hue, &r, &g, &b);

            // Saturation: always high, but audio can boost it to max
            float saturation = 0.8f + audio_reactivity * 0.2f;
            if (saturation > 1.0f) saturation = 1.0f;

            // Final brightness: influenced by base brightness, audio, and motion (gyro)
            float final_pixel_brightness = base_brightness * (0.8f + audio_reactivity * 0.4f) + smoothed_gyro_z * 0.2f;
            final_pixel_brightness = fmaxf(MIN_BRIGHTNESS, final_pixel_brightness * saturation); // Ensure min, apply saturation
            final_pixel_brightness = fminf(1.0f, final_pixel_brightness);

            p[i] = (uint8_t)(r * final_pixel_brightness);
            p[i+1] = (uint8_t)(g * final_pixel_brightness);
            p[i+2] = (uint8_t)(b * final_pixel_brightness);
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}

void mode_audio_peak_color(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float global_hue_offset = 0.0f;
    static float peak_travel_pos = 0.0f;

    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        float max_magnitude = -100.0f;
        int peak_bin = 0;
        int num_spectrum_bins = N_SAMPLES / 2;
        float sum_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_amplitude = sum_amplitude / N_SAMPLES;
        float normalized_overall_amplitude = fminf(avg_amplitude * 10.0f, 1.0f); // Increased overall audio reactivity

        for (int i = 0; i < num_spectrum_bins; i++) {
            if (spectrum[i] > max_magnitude) {
                max_magnitude = spectrum[i];
                peak_bin = i;
            }
        }

        float normalized_peak = (max_magnitude + 65.0f) / 65.0f; // Slightly more sensitive peak detection
        normalized_peak = fmaxf(0.0f, fminf(1.0f, normalized_peak));

        // Hue for the peak, slightly dynamic based on peak_bin or motion
        uint8_t peak_hue = (uint8_t)((float)peak_bin / (num_spectrum_bins - 1) * 190.0f); // Wider peak hue range
        peak_hue = (uint8_t)fmodf(peak_hue + global_hue_offset, 255.0f);
        uint8_t peak_r, peak_g, peak_b;
        hsv_to_rgb(peak_hue, &peak_r, &peak_g, &peak_b);

        // Background hue cycle, more influenced by overall audio
        global_hue_offset += (0.2f + normalized_overall_amplitude * 0.8f);
        if (global_hue_offset >= 255.0f) global_hue_offset -= 255.0f;

        // Peak traveling effect - smoother and more responsive to peak changes
        float target_peak_led_pos = (float)peak_bin / (num_spectrum_bins - 1) * (NUM_LEDS - 1);
        peak_travel_pos = peak_travel_pos * 0.7f + target_peak_led_pos * 0.3f; // Faster smoothing


        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            uint8_t r, g, b;
            float current_brightness;

            // Base background color, more reactive to overall audio amplitude
            float background_brightness = MIN_BRIGHTNESS * 1.0f + normalized_overall_amplitude * 0.4f; // Stronger, more reactive background
            hsv_to_rgb((uint8_t)fmodf(global_hue_offset + (float)led_idx * 7.0f, 255.0f), &r, &g, &b); // Faster background animation
            current_brightness = background_brightness;

            // Calculate influence from the traveling peak
            float distance_from_traveling_peak = fabsf((float)led_idx - peak_travel_pos);

            // **Significantly wider exponential falloff from the traveling peak**
            float peak_falloff = expf(-distance_from_traveling_peak / (NUM_LEDS / 2.0f)); // Much wider spread

            // Combine with normalized peak magnitude for intensity
            float peak_effect_intensity = normalized_peak * peak_falloff;

            // Blend peak color and background color - stronger blend
            float blend_factor = peak_effect_intensity * (0.9f + normalized_overall_amplitude * 0.3f);
            if (blend_factor > 1.0f) blend_factor = 1.0f;

            r = (uint8_t)(r * (1.0f - blend_factor) + peak_r * blend_factor);
            g = (uint8_t)(g * (1.0f - blend_factor) + peak_g * blend_factor);
            b = (uint8_t)(b * (1.0f - blend_factor) + peak_b * blend_factor);

            // Brightness is influenced by peak effect, but with an even stronger minimum floor
            current_brightness = fmaxf(background_brightness, current_brightness + (peak_effect_intensity * 1.0f));
            if (current_brightness > 1.0f) current_brightness = 1.0f;

            p[led_idx * 3]     = (uint8_t)(r * current_brightness);
            p[led_idx * 3 + 1] = (uint8_t)(g * current_brightness);
            p[led_idx * 3 + 2] = (uint8_t)(b * current_brightness);
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}
void mode_audio_rainbow_cycle(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float global_hue_offset = 0.0f; // Continuous global hue shift
    static float current_amplitude_smooth = 0.0f; // Smoothed amplitude for reactivity

    const float BASE_CYCLE_SPEED = 0.1f; // Slowest cycle speed
    const float AUDIO_SPEED_MULTIPLIER = 8.0f; // How much audio speeds up the cycle
    const float BASE_RAINBOW_SPREAD = 2.0f; // Base number of full rainbow cycles along the strip
    const float AUDIO_SPREAD_MODULATOR = 0.8f; // How much audio changes the spread
    const float BRIGHTNESS_PULSATION_STRENGTH = 0.2f; // How much brightness pulsates with audio

    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        float sum_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_amplitude = sum_amplitude / N_SAMPLES;
        float normalized_amplitude = fminf(avg_amplitude * 15.0f, 1.0f); // Higher sensitivity

        // Smooth amplitude for less "jumpy" reactions
        current_amplitude_smooth = current_amplitude_smooth * 0.9f + normalized_amplitude * 0.1f;

        // Cycle speed: base speed + audio influence
        float cycle_speed = BASE_CYCLE_SPEED + (current_amplitude_smooth * AUDIO_SPEED_MULTIPLIER);
        global_hue_offset += cycle_speed;
        if (global_hue_offset >= 255.0f) global_hue_offset -= 255.0f;

        // Rainbow spread: base spread, modulated by audio
        float rainbow_spread = BASE_RAINBOW_SPREAD + (current_amplitude_smooth * AUDIO_SPREAD_MODULATOR);

        // Base brightness: always present, subtly modulated by audio pulse
        float base_overall_brightness = fmaxf(MIN_BRIGHTNESS * 2.5f, MIN_BRIGHTNESS * 2.0f + current_amplitude_smooth * 0.5f);

        // Add a subtle brightness pulsation based on audio
        base_overall_brightness *= (1.0f + BRIGHTNESS_PULSATION_STRENGTH * sinf(global_hue_offset / 10.0f) * current_amplitude_smooth);

        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            uint8_t r, g, b;

            // Hue calculation: global offset + LED position modulated by dynamic spread
            uint8_t hue = (uint8_t)fmodf(global_hue_offset + (float)led_idx * (255.0f / NUM_LEDS) * rainbow_spread, 255.0f);

            // Saturation: always high, audio boosts it slightly
            float saturation_mod = 0.9f + current_amplitude_smooth * 0.1f;
            if (saturation_mod > 1.0f) saturation_mod = 1.0f;

            // Apply HSV to RGB
            hsv_to_rgb(hue, &r, &g, &b);

            // Apply brightness
            float final_pixel_brightness = base_overall_brightness * saturation_mod;
            final_pixel_brightness = fmaxf(MIN_BRIGHTNESS, final_pixel_brightness); // Ensure minimum light
            if (final_pixel_brightness > 1.0f) final_pixel_brightness = 1.0f;

            int p_idx = led_idx * 3;
            if (p_idx <= l - 3) {
                p[p_idx] = (uint8_t)(r * final_pixel_brightness);
                p[p_idx+1] = (uint8_t)(g * final_pixel_brightness);
                p[p_idx+2] = (uint8_t)(b * final_pixel_brightness);
            }
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}

void mode_audio_vu_meter(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float global_hue_offset = 0.0f; // For shifting overall color
    static float smoothed_amplitude = 0.0f; // For smoother reactions

    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        float sum_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_amplitude = sum_amplitude / N_SAMPLES;
        float normalized_amplitude = fminf(avg_amplitude * 20.0f, 1.0f); // Significantly increased sensitivity

        // Smooth amplitude for less flickering
        smoothed_amplitude = smoothed_amplitude * 0.8f + normalized_amplitude * 0.2f;

        // Shift global hue slowly, influenced by audio activity
        global_hue_offset += (0.05f + smoothed_amplitude * 0.5f);
        if (global_hue_offset >= 255.0f) global_hue_offset -= 255.0f;

        // Calculate how many LEDs should be active based on smoothed amplitude
        int active_leds = (int)(smoothed_amplitude * NUM_LEDS);
        if (active_leds > NUM_LEDS) active_leds = NUM_LEDS;

        // Base brightness for inactive LEDs, subtly pulsing
        float inactive_base_brightness = MIN_BRIGHTNESS * 1.5f * (0.8f + 0.2f * sinf(global_hue_offset / 20.0f));
        inactive_base_brightness = fmaxf(MIN_BRIGHTNESS, inactive_base_brightness);

        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            uint8_t r, g, b;
            float current_pixel_brightness;

            if (led_idx < active_leds) {
                // Active VU meter LEDs have dynamic colors
                float meter_progress = (float)led_idx / (NUM_LEDS - 1); // 0 to 1
                // Wider hue range, influenced by global hue and meter progress
                uint8_t hue = (uint8_t)fmodf(global_hue_offset + meter_progress * 170.0f, 255.0f);
                hsv_to_rgb(hue, &r, &g, &b);
                current_pixel_brightness = smoothed_amplitude * 1.2f + 0.1f; // Brighter active LEDs, more proportional to amplitude
                if (current_pixel_brightness > 1.0f) current_pixel_brightness = 1.0f;
            } else {
                // Inactive LEDs show a subtle base color, shifted by global hue
                uint8_t inactive_hue = (uint8_t)fmodf(global_hue_offset + (float)led_idx * 5.0f, 255.0f);
                hsv_to_rgb(inactive_hue, &r, &g, &b);
                current_pixel_brightness = inactive_base_brightness;
            }

            // Apply brightness
            int p_idx = led_idx * 3;
            if (p_idx <= l - 3) {
                p[p_idx] = (uint8_t)(r * current_pixel_brightness);
                p[p_idx+1] = (uint8_t)(g * current_pixel_brightness);
                p[p_idx+2] = (uint8_t)(b * current_pixel_brightness);
            }
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}

void mode_audio_beat_fade(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float current_beat_brightness_boost = 0.0f; // Smoother brightness boost
    static float last_normalized_amplitude = 0.0f;
    static int beat_count = 0;
    static float target_base_hue = 0.0f; // For smooth color transitions every N beats
    static float current_fade_hue = 0.0f; // Currently displayed hue

    // Tuned constants for less flicker, more regularity, and compressed brightness range
    const float MIN_AUDIO_LEVEL_FOR_BEAT = 0.08f; // Even lower threshold for beat detection
    const float BEAT_SENSITIVITY = 0.15f; // Slightly higher sensitivity to detect clearer peaks
    const float BRIGHTNESS_DECAY_RATE = 0.04f; // Even slower decay for much less flicker
    const int BEATS_PER_COLOR_CHANGE = 4;
    const float HUE_TRANSITION_RATE = 0.02f; // Slower, smoother hue transition

    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        float sum_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_amplitude = sum_amplitude / N_SAMPLES;
        float normalized_amplitude = fminf(avg_amplitude * 18.0f, 1.0f); // Even more amplified sensitivity

        // Improved Beat Detection: look for a significant rise from a low point
        if (normalized_amplitude > MIN_AUDIO_LEVEL_FOR_BEAT &&
            (normalized_amplitude - last_normalized_amplitude > BEAT_SENSITIVITY)) {

            beat_count++;
            current_beat_brightness_boost = 1.0f; // Max boost on beat

            if (beat_count >= BEATS_PER_COLOR_CHANGE) {
                target_base_hue = fmodf(target_base_hue + 90.0f + (esp_random() % 60), 255.0f); // More distinct and random color shift
                beat_count = 0;
            }
        }
        last_normalized_amplitude = normalized_amplitude;

        // Smoothly transition current hue towards target hue
        current_fade_hue += (target_base_hue - current_fade_hue) * HUE_TRANSITION_RATE;
        current_fade_hue = fmodf(current_fade_hue, 255.0f);
        if (current_fade_hue < 0) current_fade_hue += 255.0f;

        // Decay brightness boost smoothly
        current_beat_brightness_boost = fmaxf(0.0f, current_beat_brightness_boost - BRIGHTNESS_DECAY_RATE);

        // Calculate base brightness, ensuring minimum light and adding smooth beat boost
        // Goal: less difference between max brightness and base brightness
        float base_overall_brightness = fmaxf(MIN_BRIGHTNESS * 3.5f, MIN_BRIGHTNESS * 2.5f + normalized_amplitude * 0.3f); // Higher floor, lower direct amplitude scaling
        base_overall_brightness += current_beat_brightness_boost * 0.3f; // Even smaller beat boost relative to base (compressing range)

        uint8_t current_r, current_g, current_b;
        hsv_to_rgb((uint8_t)current_fade_hue, &current_r, &current_g, &current_b);

        for (int i = 0; i < l; i += 3) {
            float pixel_brightness = base_overall_brightness;

            // Optional: Subtle individual LED reaction to audio amplitude
            // Reduced individual reaction contribution to avoid flicker and maintain compressed range
            float individual_led_audio_reaction = fminf(normalized_amplitude * 0.3f, 0.3f);
            pixel_brightness += individual_led_audio_reaction * sinf((float)i / l * M_PI); // Use sine for gentle spread

            if (pixel_brightness > 1.0f) pixel_brightness = 1.0f;
            pixel_brightness = fmaxf(MIN_BRIGHTNESS, pixel_brightness); // Ensure minimum light

            p[i] = (uint8_t)(current_r * pixel_brightness);
            p[i+1] = (uint8_t)(current_g * pixel_brightness);
            p[i+2] = (uint8_t)(current_b * pixel_brightness);
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}

void mode_audio_frequency_lava(qmi8658_data_t *s, uint8_t *p, size_t l) {
    static float global_hue_shift = 0.0f; // Overall lava color shift
    static float flow_position = 0.0f;     // Position of the "lava" flow

    const float BASE_FLOW_SPEED = 0.05f; // Base speed of the lava flow
    const float HUE_SPREAD = 80.0f;      // How much hues spread out in blobs

    if (xSemaphoreTake(audio_spectrum_buffer_mutex, (TickType_t)10) == pdTRUE) {
        float sum_amplitude = 0.0f;
        for (int i = 0; i < N_SAMPLES; i++) {
            sum_amplitude += fabsf(audio_buffer[i]);
        }
        float avg_amplitude = sum_amplitude / N_SAMPLES;
        float normalized_amplitude = fminf(avg_amplitude * 10.0f, 1.0f); // Overall audio reactivity

        // Analyze frequency bands
        float low_freq_mag = spectrum[0]; // First bin for low frequencies
        float mid_freq_mag = (spectrum[1] + spectrum[2]) / 2.0f; // Mid bins
        float high_freq_mag = spectrum[N_SAMPLES / 2 - 1]; // Highest bin for high frequencies

        float normalized_low = fmaxf(0.0f, fminf(1.0f, (low_freq_mag + 60.0f) / 60.0f));
        float normalized_mid = fmaxf(0.0f, fminf(1.0f, (mid_freq_mag + 60.0f) / 60.0f));
        float normalized_high = fmaxf(0.0f, fminf(1.0f, (high_freq_mag + 60.0f) / 60.0f));

        // Overall global hue slowly shifts
        global_hue_shift += 0.1f;
        if (global_hue_shift >= 255.0f) global_hue_shift -= 255.0f;

        // Flow speed influenced by overall audio amplitude
        flow_position += BASE_FLOW_SPEED + (normalized_amplitude * 0.2f);
        if (flow_position >= NUM_LEDS * 2) flow_position -= NUM_LEDS * 2; // Cycle flow

        float base_brightness = fmaxf(MIN_BRIGHTNESS * 1.5f, normalized_amplitude * 0.4f + MIN_BRIGHTNESS * 1.0f);

        for (int led_idx = 0; led_idx < NUM_LEDS; led_idx++) {
            uint8_t r, g, b;
            float current_pixel_brightness = base_brightness;

            // Base lava color with slight modulation from flow
            uint8_t base_lava_hue = (uint8_t)fmodf(global_hue_shift + sinf((float)led_idx / NUM_LEDS * M_PI + flow_position / 10.0f) * 20.0f, 255.0f);
            hsv_to_rgb(base_lava_hue, &r, &g, &b);

            // Frequency band influence: create "blobs" of color or brighter areas
            float band_influence = 0.0f;
            uint8_t band_hue = 0;

            if (led_idx < NUM_LEDS / 3) { // Lower part of strip for low frequencies
                band_influence = normalized_low;
                band_hue = (uint8_t)fmodf(global_hue_shift + 0, 255); // Reds/Oranges
            } else if (led_idx < NUM_LEDS * 2 / 3) { // Middle part for mid frequencies
                band_influence = normalized_mid;
                band_hue = (uint8_t)fmodf(global_hue_shift + HUE_SPREAD, 255); // Yellows/Greens
            } else { // Upper part for high frequencies
                band_influence = normalized_high;
                band_hue = (uint8_t)fmodf(global_hue_shift + HUE_SPREAD * 2, 255); // Blues/Violets
            }

            // Localized brightness boost and color shift from frequency bands
            if (band_influence > 0.1f) {
                float blend_factor = band_influence * 0.8f; // Stronger blend
                uint8_t tr, tg, tb;
                hsv_to_rgb(band_hue, &tr, &tg, &tb);

                r = (uint8_t)(r * (1.0f - blend_factor) + tr * blend_factor);
                g = (uint8_t)(g * (1.0f - blend_factor) + tg * blend_factor);
                b = (uint8_t)(b * (1.0f - blend_factor) + tb * blend_factor);
                current_pixel_brightness = fmaxf(current_pixel_brightness, blend_factor); // Boost brightness
            }

            // Overall amplitude can make the lava "bubble" or glow more intensely
            current_pixel_brightness *= (1.0f + normalized_amplitude * 0.5f);

            current_pixel_brightness = fmaxf(MIN_BRIGHTNESS, current_pixel_brightness);
            if (current_pixel_brightness > 1.0f) current_pixel_brightness = 1.0f;

            p[led_idx * 3]     = (uint8_t)(r * current_pixel_brightness);
            p[led_idx * 3 + 1] = (uint8_t)(g * current_pixel_brightness);
            p[led_idx * 3 + 2] = (uint8_t)(b * current_pixel_brightness);
        }
        xSemaphoreGive(audio_spectrum_buffer_mutex);
    }
}


// =============================================================================
// BLE & SYSTEM LOGIC
// =============================================================================

// BLE-related Forward Declarations
static int ble_central_event(struct ble_gap_event *event, void *arg);
void poi_scan_start(void);
static int on_disc_char(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg);



static void check_and_start_streaming(void) {
    int connected_and_discovered_count = 0;
    for (int i = 0; i < 2; i++) {
        if (devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE && devices[i].discovered) {
            connected_and_discovered_count++;
        }
    }
    if (connected_and_discovered_count == 2) {
        is_streaming = true;
        ESP_LOGI(TAG, "Both POIs connected and discovered. Starting streaming.");
        if (lvgl_port_lock(LVGL_PORT_LOCK_TIMEOUT_MS)) {
            if (poi_info_label != NULL) {
                lv_label_set_text(poi_info_label, "All Connected. Streaming!");
                lv_obj_set_style_text_color(poi_info_label, lv_color_make(0x00, 0xFF, 0x00), 0);
            }
            lvgl_port_unlock();
        }
    }
}

void poi_scan_start(void) {
    int conn_count = 0;
    for(int i=0; i<2; i++) {
        if(devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE) conn_count++;
    }
    if (conn_count >= 2) return;

    struct ble_gap_disc_params dp;
    memset(&dp, 0, sizeof(dp));

    dp.filter_duplicates = 1;
    dp.passive = 0; // Active scanning to get the device name

    int rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &dp, ble_central_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Scan Error: %d", rc);
    }
}

static int on_disc_char(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg) {
    poi_device_t *dev = (poi_device_t *)arg;

    if (error->status == 0) {
        if (ble_uuid_cmp(&rx_uuid.u, &chr->uuid.u) == 0) {
            dev->rx_char_handle = chr->val_handle;
            ESP_LOGI(TAG, "RX Characteristic Found (Handle: %d)!", dev->rx_char_handle);
        } else if (ble_uuid_cmp(&tx_uuid.u, &chr->uuid.u) == 0) {
            dev->tx_char_handle = chr->val_handle;
            ESP_LOGI(TAG, "TX Characteristic Found (Handle: %d)!", dev->tx_char_handle);
        }
    } else if (error->status == BLE_HS_EDONE) {
        // Discovery finished for this device
        if (dev->rx_char_handle != 0 && dev->tx_char_handle != 0) {
            dev->discovered = true;
            dev->config_received = true; // Implicitly true as we are no longer reading config
            ESP_LOGI(TAG, "All characteristics discovered for device (Handle: %d).", conn_handle);
            check_and_start_streaming(); // Check if streaming can start
            poi_scan_start(); // Now ready to look for other devices if needed
        } else {
            ESP_LOGE(TAG, "Discovery done but not all characteristics found for device (Handle: %d). RX: %d, TX: %d", conn_handle, dev->rx_char_handle, dev->tx_char_handle);
            poi_scan_start(); // Try again/next if characteristics were not found for this device.
        }
    }
    return 0;
}



static int ble_central_event(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
                                    if (fields.name_len > 0 && strncmp((char *)fields.name, TARGET_NAME, fields.name_len) == 0) {
                                        // Only connect if we have an empty slot
                                        for (int i = 0; i < 2; i++) {
                                            if (devices[i].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                                                ble_gap_disc_cancel();
                                                ESP_LOGI(TAG, "Connecting to Poi %d...", i);
                                                ble_gap_connect(own_addr_type, &event->disc.addr, 30000, NULL, ble_central_event, NULL);
                                                break;
                                            }
                                        }
                                    }            break;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            poi_scan_start();
            break;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Connected! Negotiating link...");

                // 1. Request larger MTU so the 38-byte packet fits in one transmission
                ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);

                // 2. Update Connection Parameters (Cleanly to avoid warnings)
                struct ble_gap_upd_params params;
                memset(&params, 0, sizeof(params));
                params.itvl_min = 12; // 15ms
                params.itvl_max = 24; // 30ms
                params.latency = 0;
                params.supervision_timeout = 400; // 4s
                ble_gap_update_params(event->connect.conn_handle, &params);

                for (int i = 0; i < 2; i++) {
                    if (devices[i].conn_handle == BLE_HS_CONN_HANDLE_NONE) {
                        devices[i].conn_handle = event->connect.conn_handle;
                        ble_gattc_disc_all_chrs(devices[i].conn_handle, 1, 0xffff, on_disc_char, &devices[i]);
                        break;
                    }
                }
            } else { 
                ESP_LOGE(TAG, "Connection failed: %d", event->connect.status);
                poi_scan_start(); // Restart scan to try again
            }
            break;



        case BLE_GAP_EVENT_DISCONNECT:
            for (int i = 0; i < 2; i++) {
                if (devices[i].conn_handle == event->disconnect.conn.conn_handle) {
                    devices[i].conn_handle = BLE_HS_CONN_HANDLE_NONE;
                    devices[i].discovered = false;
                    devices[i].stream_started = false;
                    break;
                }
            }
            poi_scan_start(); // Restart scan to find a replacement if needed
            break;
    } // Closing brace for switch (event->type)
    return 0;
} // Closing brace for ble_central_event function



void init_buttons() {
    // Reset both to clear any weird states
    gpio_reset_pin((gpio_num_t)MODE_BUTTON_GPIO);
    gpio_reset_pin((gpio_num_t)POWER_BUTTON_GPIO);

    // Mode Button: Input + Pull-Down
    gpio_set_direction((gpio_num_t)MODE_BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)MODE_BUTTON_GPIO, GPIO_PULLDOWN_ONLY);

    // Power Button: Input + Pull-Down
    gpio_set_direction((gpio_num_t)POWER_BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)POWER_BUTTON_GPIO, GPIO_PULLDOWN_ONLY);
}

// --- Tasks ---
static void button_monitor_task(void *arg) {
    init_buttons(); 

    while (1) {
        // Read states once per loop
        bool pwr_pressed  = (gpio_get_level((gpio_num_t)POWER_BUTTON_GPIO) == 1);

        // --- POWER BUTTON ---
        static TickType_t pwr_press_start_time = 0;
        static bool pwr_button_was_pressed = false;

        if (pwr_pressed) { // Button is currently pressed
            if (!pwr_button_was_pressed) { // First detected press
                pwr_press_start_time = xTaskGetTickCount();
                pwr_button_was_pressed = true;
                ESP_LOGI(TAG, "Power button pressed.");
            }
            // No action for holding here, PMU hardware will handle 4s hard shutdown
        } else { // Button is currently released
            if (pwr_button_was_pressed) { // Button was just released
                TickType_t hold_duration_ms = (xTaskGetTickCount() - pwr_press_start_time) * portTICK_PERIOD_MS;
                ESP_LOGI(TAG, "Power button released after %lu ms.", hold_duration_ms);

                // Graceful shutdown after 2 seconds press and release
                if (hold_duration_ms >= 2000) { // If held for 2 seconds or more
                    ESP_LOGI(TAG, "Power button pressed and released for >= 2 seconds. Initiating graceful shutdown.");

                    // Preparatory steps for graceful shutdown
                    lvgl_port_stop();
                    ble_gap_disc_cancel();
                    is_streaming = false;
                    bsp_display_backlight_off();
                    vTaskDelay(pdMS_TO_TICKS(50)); // Give UART time to flush

                    pmu.shutdown(); // Graceful shutdown via PMU
                    // The system will power off here
                }
                // Reset for next press
                pwr_button_was_pressed = false;
                pwr_press_start_time = 0;
            }
        }

        // --- MODE BUTTON (Independent) ---
        // Removed mode button handling as mode selection is now via UI icons.
        // static bool mode_was_pressed = false;
        // if (mode_pressed && !mode_was_pressed) {
        //     current_mode = (current_mode + 1) % MODE_COUNT;
        //     ESP_LOGI(TAG, "Mode Switched to: %d", current_mode);

        //     // Lock the UI before touching labels
        //     if (lvgl_port_lock(LVGL_PORT_LOCK_TIMEOUT_MS)) {
        //         if (mode_label != NULL) {
        //             lv_label_set_text_fmt(mode_label, "MODE: \n\n %s", mode_names[current_mode]);
        //         }
        //         lvgl_port_unlock();
        //     }
        //     mode_was_pressed = true;

        // } else if (!mode_pressed) {
        //     mode_was_pressed = false;
        // }

        // --- UI UPDATES ---
        static TickType_t last_ui_update = 0;
        if ((xTaskGetTickCount() - last_ui_update) > pdMS_TO_TICKS(500)) { // Update UI every 500ms
            last_ui_update = xTaskGetTickCount();
            if (lvgl_port_lock(LVGL_PORT_LOCK_TIMEOUT_MS)) {
                // Update Watch Battery Label on scr_system_info
                if (battery_label != NULL) {
                    if (xSemaphoreTake(pmu_data_mutex, (TickType_t)10) == pdTRUE) {
                        char batt_status[50];
                        const char* charge_state = is_usb_connected ? (is_charging ? "Charging" : "USB") : "Discharging";
                        snprintf(batt_status, sizeof(batt_status), "BAT: %.2fV %d%% (%s)",
                                 battery_voltage, battery_percentage, charge_state);
                        lv_label_set_text(battery_label, batt_status);

                        // Set color based on charging state
                        if (is_charging) {
                            lv_obj_set_style_text_color(battery_label, lv_color_make(0x00, 0xFF, 0x00), 0); // Green
                        } else if (battery_percentage < 20 && !is_usb_connected) {
                            lv_obj_set_style_text_color(battery_label, lv_color_make(0xFF, 0x00, 0x00), 0); // Red for low battery
                        } else {
                            lv_obj_set_style_text_color(battery_label, lv_color_make(0xFF, 0xFF, 0xFF), 0); // White
                        }
                        xSemaphoreGive(pmu_data_mutex);
                    }
                }

                // Update POI Info Box on scr_system_info
                if (poi_info_box != NULL && poi_info_label != NULL) {
                    char full_poi_info_str[200];
                    full_poi_info_str[0] = '\0'; // Initialize empty string
                    int connected_pois_count = 0;

                    for (int i = 0; i < 2; i++) {
                        if (devices[i].discovered) {
                            connected_pois_count++;
                            char temp_poi_str[100];
                            snprintf(temp_poi_str, sizeof(temp_poi_str), "POI %d: Connected\n", i + 1);
                            strcat(full_poi_info_str, temp_poi_str);
                        }
                    }

                    if (connected_pois_count > 0) {
                        lv_label_set_text(poi_info_label, full_poi_info_str);
                        lv_obj_set_style_text_color(poi_info_label, lv_color_make(0x00, 0xFF, 0x00), 0); // Green for connected
                    } else {
                        lv_label_set_text(poi_info_label, "Connecting...");
                        lv_obj_set_style_text_color(poi_info_label, lv_color_make(0xFF, 0xFF, 0xFF), 0); // White for connecting
                    }
                    lv_obj_center(poi_info_label); // Re-center after text change
                }

                // Clock labels are now updated by rtc_time_update_task


                lvgl_port_unlock();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void stream_task(void *param) {
    uint8_t packet[2 + (NUM_LEDS * 3)];
    packet[0] = START_BYTE;
    packet[1] = CC_STREAM_DATA;
    qmi8658_data_t imu_data;

    while (1) {
        if (is_streaming) {
            qmi8658_read_accel(&imu_dev, &imu_data.accelX, &imu_data.accelY, &imu_data.accelZ);
            qmi8658_read_gyro(&imu_dev, &imu_data.gyroX, &imu_data.gyroY, &imu_data.gyroZ);
            mode_table[current_mode % MODE_COUNT](&imu_data, &packet[2], NUM_LEDS * 3);
            // 2. APPLY GLOBAL BRIGHTNESS SCALING
            // We start at index 2 to skip the header bytes
            for (int j = 2; j < sizeof(packet); j++) {
                packet[j] = (uint8_t)(packet[j] * GLOBAL_BRIGHTNESS);
            }

            for (int i = 0; i < 2; i++) {
                if (devices[i].conn_handle != BLE_HS_CONN_HANDLE_NONE && devices[i].discovered) {

                    if (!devices[i].stream_started) {
                        uint8_t sc[] = {START_BYTE, CC_START_STREAM, 0xD1};
                        if (ble_gattc_write_flat(devices[i].conn_handle, devices[i].rx_char_handle, sc, 3, NULL, NULL) == 0) {
                            devices[i].stream_started = true;
                            ESP_LOGI(TAG, "Handshake sent to Poi %d", i);
                        }
                    } else {
                        // Attempt to write
                        int rc = ble_gattc_write_no_rsp_flat(devices[i].conn_handle, devices[i].rx_char_handle, packet, sizeof(packet));

                        if (rc == BLE_HS_ENOMEM) {
                            // Buffer full! Just skip this frame for this device to keep things moving
                            continue;
                        } else if (rc != 0) {
                            ESP_LOGD(TAG, "Write error on device %d: %d", i, rc);
                        }
                    }
                }
            }
        }
        // Let's slow down slightly to 50ms (20fps) to stabilize dual-stream
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

/* ------------------ 音频 FFT 任务 ------------------ */
void audio_fft_task(void *pvParameters)
{
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "FFT init failed: %d", ret);
        vTaskDelete(NULL);
    }

    dsps_wind_hann_f32(wind, N_SAMPLES);
    ESP_LOGI(TAG, "FFT and window initialized");

    if (bsp_extra_codec_init() != ESP_OK)
    {
        ESP_LOGE(TAG, "Audio codec init failed");
        vTaskDelete(NULL);
    }

    size_t bytes_read;

    while (1)
    {
        ret = bsp_extra_i2s_read(raw_data, N_SAMPLES * CHANNELS * sizeof(int16_t), &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK || bytes_read != N_SAMPLES * CHANNELS * sizeof(int16_t))
        {
            ESP_LOGW(TAG, "I2S read error: %d, bytes: %d", ret, bytes_read);
            continue;
        }

        // 合并左右声道并归一化
        for (int i = 0; i < N_SAMPLES; i++)
        {
            int16_t left = raw_data[i * CHANNELS];
            int16_t right = raw_data[i * CHANNELS + 1];
            audio_buffer[i] = (left + right) / (2.0f * 32768.0f);
            // Apply microphone sensitivity
            audio_buffer[i] *= current_mic_sensitivity;
        }

        dsps_mul_f32(audio_buffer, wind, audio_buffer, N_SAMPLES, 1, 1, 1);

        // 填充 FFT 输入缓冲
        for (int i = 0; i < N_SAMPLES; i++)
        {
            fft_buffer[2 * i] = audio_buffer[i];
            fft_buffer[2 * i + 1] = 0;
        }

        dsps_fft2r_fc32(fft_buffer, N_SAMPLES);
        dsps_bit_rev_fc32(fft_buffer, N_SAMPLES);

        // Calculate magnitude spectrum (dB)
        if (xSemaphoreTake(audio_spectrum_buffer_mutex, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < N_SAMPLES / 2; i++)
            {
                float real = fft_buffer[2 * i];
                float imag = fft_buffer[2 * i + 1];
                float magnitude = sqrtf(real * real + imag * imag);
                spectrum[i] = 20 * log10f(magnitude / (N_SAMPLES / 2) + 1e-9);
            }
            xSemaphoreGive(audio_spectrum_buffer_mutex);
        }
    }
}

// --- PMU Battery Monitor Task ---
void battery_monitor_task(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(pmu_data_mutex, portMAX_DELAY) == pdTRUE) {
            battery_voltage = pmu.getBattVoltage() / 1000.0f; // Convert mV to V
            battery_percentage = pmu.getBatteryPercent();
            is_charging = pmu.isCharging();
            is_usb_connected = pmu.isVbusIn();
            xSemaphoreGive(pmu_data_mutex);
        }
        ESP_LOGI(TAG, "Battery: %.2fV, %d%%, Charging: %d, USB: %d",
                 battery_voltage, battery_percentage, is_charging, is_usb_connected);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Check every 5 seconds
    }
}

// --- RTC Time Update Task ---
void rtc_time_update_task(void *pvParameters) {
    struct tm timeinfo;
    char strftime_buf[64];

    while (1) {
        // Read time from RTC
        if (rtc_pcf85063a_get_time(&timeinfo) == ESP_OK) {
            // Convert to UNIX timestamp for lv_label_set_text_fmt
            time_t unix_time = mktime(&timeinfo);

            // Update LVGL labels (acquire lock first)
            if (lvgl_port_lock(LVGL_PORT_LOCK_TIMEOUT_MS)) {
                // Time
                strftime(strftime_buf, sizeof(strftime_buf), "%H:%M:%S", &timeinfo);
                lv_label_set_text(clock_time_label, strftime_buf);

                // Date
                strftime(strftime_buf, sizeof(strftime_buf), "%a, %b %d %Y", &timeinfo);
                lv_label_set_text(clock_date_label, strftime_buf);

                // Unix timestamp
                lv_label_set_text_fmt(clock_unix_label, "UNIX: %lld", unix_time);

                lvgl_port_unlock();
            }
            ESP_LOGD(TAG, "RTC Time: %s, Unix: %ld", strftime_buf, unix_time);
        } else {
            ESP_LOGE(TAG, "Failed to read time from RTC!");
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
    }
}

static void on_sync(void) { ble_hs_id_infer_auto(0, &own_addr_type); poi_scan_start(); }
void host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Task started.");
    nimble_port_run();
    ESP_LOGE(TAG, "NimBLE Host Task unexpectedly exited!");
}

extern "C" void app_main(void) {
    nvs_flash_init();
    bsp_display_start();
    bsp_display_backlight_on();

    // Init IMU
    i2c_master_bus_handle_t bus = bsp_i2c_get_handle();
    qmi8658_init(&imu_dev, bus, QMI8658_ADDRESS_HIGH);

    // Init PMU (AXP2101)
    ESP_ERROR_CHECK(i2c_init_pmu_device(bus)); // Initialize I2C for PMU
    pmu_data_mutex = xSemaphoreCreateMutex(); // Initialize PMU data mutex
    if (!pmu.init()) { // Call pmu.init() without bus argument
        ESP_LOGE(TAG, "PMU init failed!");
        return; // Critical error, halt
    }
    ESP_LOGI(TAG, "PMU initialized successfully!");

    // Init RTC (PCF85063A)
    if (rtc_pcf85063a_init(bus) != ESP_OK) {
        ESP_LOGE(TAG, "RTC init failed!");
        // Non-critical error, maybe continue without RTC, or return
        // For now, let's return to indicate a problem.
        return;
    }
    ESP_LOGI(TAG, "RTC initialized successfully!");

#ifdef SET_INITIAL_RTC_TIME
    struct tm initial_time = {
        .tm_sec = 0,    // Second (0-59)
        .tm_min = 50,   // Minute (0-59)
        .tm_hour = 21,  // Hour (0-23)
        .tm_mday = 22,  // Day of the month (1-31)
        .tm_mon = 1,    // Month (0-11, so 1 for February)
        .tm_year = 2026 - 1900, // Year since 1900
        .tm_wday = 0,   // Day of the week (0-6, Sunday is 0) - not strictly necessary for PCF85063A but good practice
        .tm_yday = 0,   // Day in the year (0-365)
        .tm_isdst = -1  // Daylight saving time (-1 unknown)
    };

    ESP_LOGI(TAG, "Attempting to set RTC time to %02d:%02d:%02d %02d/%02d/%d",
             initial_time.tm_hour, initial_time.tm_min, initial_time.tm_sec,
             initial_time.tm_mon + 1, initial_time.tm_mday, initial_time.tm_year + 1900);

    if (rtc_pcf85063a_set_time(&initial_time) == ESP_OK) {
        ESP_LOGI(TAG, "RTC time set successfully!");
    } else {
        ESP_LOGE(TAG, "Failed to set RTC time!");
    }
#endif // SET_INITIAL_RTC_TIME

    // Enable necessary PMU measurements
    pmu.enableBattDetection();
    pmu.enableBattVoltageMeasure();
    pmu.enableVbusVoltageMeasure();
    pmu.enableSystemVoltageMeasure();

    // Configure AXP2101 for hard power off after 4-second button press (closest to 5s)
    pmu.setOffLevel(0); // 0 corresponds to 4 seconds for OFFLEVEL configuration

    // Init BLE
    nimble_port_init();
    ble_hs_cfg.sync_cb = on_sync;

    //Init display
lvgl_port_cfg_t lvgl_cfg = {};

// 2. Set only the absolutely necessary fields
lvgl_cfg.task_priority = 5;
lvgl_cfg.task_stack = 6192;
lvgl_cfg.task_affinity = -1;
lvgl_cfg.timer_period_ms = 5; // Use 5ms for smooth animations

// 3. Initialize and CHECK if it failed
esp_err_t err = lvgl_port_init(&lvgl_cfg);
if (err != ESP_OK) {
    ESP_LOGE(TAG, "LVGL Port Init Failed: %s", esp_err_to_name(err));
    return; // Don't proceed if UI failed
}
	// 2. Create the UI
	ui_init();


    xTaskCreate(button_monitor_task, "btn", 3072, NULL, 5, NULL);
    xTaskCreate(stream_task, "stream", 4096, NULL, 10, NULL);

    audio_spectrum_buffer_mutex = xSemaphoreCreateMutex();
    xTaskCreate(audio_fft_task, "audio_fft", 4 * 1024, NULL, 5, NULL);
    xTaskCreate(battery_monitor_task, "batt_mon", 2048, NULL, 5, NULL);
    xTaskCreate(rtc_time_update_task, "rtc_time_update", 2048, NULL, 5, NULL); // New RTC time update task

    nimble_port_freertos_init(host_task);
    ESP_LOGI(TAG, "App_main finished. Free heap: %u bytes", esp_get_free_heap_size());
}
