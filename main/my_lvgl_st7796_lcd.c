/* ESP32-3248S035 with LVGL v9.4
 * ESP-IDF v5.5.1
 * 
 * Pin Configuration:
 * LCD (ST7796) - SPI Interface:
 * - MOSI: GPIO 13
 * - MISO: GPIO 12 (not used for display)
 * - SCLK: GPIO 14
 * - CS:   GPIO 15
 * - DC:   GPIO 2
 * - RST:  Not connected (-1)
 * - Backlight: GPIO 27
 * 
 * Display: 320x480 ST7796
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#include "esp_lcd_st7796.h"

static const char *TAG = "LVGL_DEMO";

// LCD Pin definitions
#define LCD_HOST            SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ  (40 * 1000 * 1000)
#define LCD_BK_LIGHT_ON     1
#define LCD_BK_LIGHT_OFF    0

#define PIN_NUM_MOSI        13
#define PIN_NUM_CLK         14
#define PIN_NUM_CS          15
#define PIN_NUM_DC          2
#define PIN_NUM_RST         -1
#define PIN_NUM_BL          27

// Touch Pin definitions (XPT2046 shares SPI with LCD)
#define TOUCH_HOST          SPI2_HOST  // Same SPI bus as LCD
#define TOUCH_CS            33         // Touch chip select
#define TOUCH_IRQ           36         // Touch interrupt (optional)
#define TOUCH_CLOCK_HZ      (2 * 1000 * 1000)  // 2.5MHz max for XPT2046

// LCD resolution
#define LCD_H_RES           480
#define LCD_V_RES           320

// Bit number used to represent color depth (RGB565 = 16 bits)
#define LCD_BIT_PER_PIXEL   16

// LVGL draw buffer size (1/10 of screen)
#define LVGL_BUFFER_SIZE    (LCD_H_RES * LCD_V_RES / 10)

// LVGL tick timer period (ms)
#define LVGL_TICK_PERIOD_MS 2

static lv_display_t *lvgl_disp = NULL;
static lv_indev_t *lvgl_touch_indev = NULL;

// LVGL tick callback
static void increase_lvgl_tick(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// LVGL display flush callback
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsety1 = area->y1;
    int offsetx2 = area->x2;
    int offsety2 = area->y2;
    
    // Copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
    lv_display_flush_ready(disp);
}

// LVGL touch read callback
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t touch_handle = (esp_lcd_touch_handle_t)lv_indev_get_user_data(indev);
    uint16_t touchpad_x;
    uint16_t touchpad_y;
    uint16_t touch_strength;
    uint8_t touch_cnt = 0;

    // Set default state to released
    data->state = LV_INDEV_STATE_RELEASED;

    // Read touch data
    esp_lcd_touch_read_data(touch_handle);
    
    // Get coordinates
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch_handle, &touchpad_x, &touchpad_y, 
                                                          &touch_strength, &touch_cnt, 1);
    
    if (touchpad_pressed && touch_cnt > 0) {
        data->state = LV_INDEV_STATE_PRESSED;
        data->point.x = touchpad_x;
        data->point.y = touchpad_y;
        
        ESP_LOGI(TAG, "Touch: x=%d, y=%d, strength=%d", touchpad_x, touchpad_y, touch_strength);
    }
}

// Configure and initialize backlight GPIO
static void init_backlight_gpio(void)
{
    ESP_LOGI(TAG, "Configuring backlight GPIO");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(PIN_NUM_BL, LCD_BK_LIGHT_OFF);
}

// Initialize SPI bus for LCD and touch communication
static void init_spi_bus(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = 12,  // MISO needed for touch controller
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
}

// Attach LCD to SPI bus and create panel IO handle
static esp_lcd_panel_io_handle_t init_panel_io(void)
{
    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));
    return io_handle;
}

// Install and configure ST7796 panel driver
static esp_lcd_panel_handle_t init_lcd_panel(esp_lcd_panel_io_handle_t io_handle)
{
    ESP_LOGI(TAG, "Install ST7796 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
    return panel_handle;
}

// Reset and configure LCD panel settings
static void configure_lcd_panel(esp_lcd_panel_handle_t panel_handle)
{
    ESP_LOGI(TAG, "Configuring LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true)); /* true: horizontal */
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

// Initialize XPT2046 touch controller
static esp_lcd_touch_handle_t init_touch_controller(void)
{
    ESP_LOGI(TAG, "Initialize XPT2046 resistive touch controller");
    
    esp_lcd_panel_io_handle_t touch_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(TOUCH_CS);
    io_config.pclk_hz = TOUCH_CLOCK_HZ;
    
    esp_err_t ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)TOUCH_HOST, &io_config, &touch_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch panel IO: %s", esp_err_to_name(ret));
        return NULL;
    }

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_V_RES,
        .y_max = LCD_H_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,  // Can use TOUCH_IRQ if you want interrupt mode
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 1,
            .mirror_x = 1,
            .mirror_y = 1,
        },
    };

    esp_lcd_touch_handle_t touch_handle = NULL;
    ret = esp_lcd_touch_new_spi_xpt2046(touch_io_handle, &tp_cfg, &touch_handle);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize XPT2046: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Troubleshooting checklist:");
        ESP_LOGE(TAG, "  1. Verify touch CS pin: GPIO %d", TOUCH_CS);
        ESP_LOGE(TAG, "  2. Check SPI wiring (shared with LCD): MOSI=%d, MISO=12, CLK=%d", PIN_NUM_MOSI, PIN_NUM_CLK);
        ESP_LOGE(TAG, "  3. Ensure XPT2046 has power (3.3V)");
        return NULL;
    }
    
    ESP_LOGI(TAG, "XPT2046 touch controller initialized successfully");
    return touch_handle;
}

// Turn on the backlight
static void enable_backlight(void)
{
    ESP_LOGI(TAG, "Turning on backlight");
    gpio_set_level(PIN_NUM_BL, LCD_BK_LIGHT_ON);
}

// Create and start LVGL tick timer
static void init_lvgl_tick_timer(void)
{
    ESP_LOGI(TAG, "Creating LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
}

// Allocate LVGL draw buffers
static void allocate_lvgl_buffers(lv_color_t **buf1, lv_color_t **buf2)
{
    ESP_LOGI(TAG, "Allocate LVGL draw buffers");
    size_t buffer_size = LVGL_BUFFER_SIZE * sizeof(lv_color_t);
    *buf1 = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
    assert(*buf1);
    *buf2 = heap_caps_malloc(buffer_size, MALLOC_CAP_DMA);
    assert(*buf2);
}

// Create and configure LVGL display
static void init_lvgl_display(esp_lcd_panel_handle_t panel_handle, lv_color_t *buf1, lv_color_t *buf2)
{
    ESP_LOGI(TAG, "Creating LVGL display");
    size_t buffer_size = LVGL_BUFFER_SIZE * sizeof(lv_color_t);
    
    lvgl_disp = lv_display_create(LCD_H_RES, LCD_V_RES);
    lv_display_set_flush_cb(lvgl_disp, lvgl_flush_cb);
    lv_display_set_buffers(lvgl_disp, buf1, buf2, buffer_size, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_user_data(lvgl_disp, panel_handle);
}

// Initialize LVGL touch input device
static void init_lvgl_touch_input(esp_lcd_touch_handle_t touch_handle)
{
    ESP_LOGI(TAG, "Creating LVGL touch input device");
    
    lvgl_touch_indev = lv_indev_create();
    lv_indev_set_type(lvgl_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(lvgl_touch_indev, lvgl_touch_read_cb);
    lv_indev_set_user_data(lvgl_touch_indev, touch_handle);
}

// Button event handler
static void button_event_handler(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        // ESP_LOGI(TAG, "Button clicked!");
        
        // Change button label on click
        lv_obj_t *btn = lv_event_get_target(e);
        lv_obj_t *label = lv_obj_get_child(btn, 0);
        lv_label_set_text(label, "Clicked!");
        
        // Change button color
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x00AA00), LV_PART_MAIN);
    }
}

// Create background style
static void create_background(void)
{
    lv_obj_t *screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x003a57), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
}

// Create "Hello World" label
static void create_hello_label(void)
{
    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "Hello World");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(label);
}

// Create button with label and event handler
static void create_button(void)
{
    lv_obj_t *btn = lv_button_create(lv_screen_active());
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 70);
    lv_obj_add_event_cb(btn, button_event_handler, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Click Me!");
    lv_obj_center(btn_label);
}

// Create all UI elements
static void create_ui(void)
{
    ESP_LOGI(TAG, "Creating UI");
    create_background();
    create_hello_label();
    create_button();
}

// Main LVGL task loop
static void lvgl_task_loop(void)
{
    ESP_LOGI(TAG, "Starting LVGL task loop");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_timer_handler();
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-3248S035R LVGL Demo with XPT2046 Resistive Touch");
    ESP_LOGI(TAG, "LVGL version: %d.%d.%d", lv_version_major(), lv_version_minor(), lv_version_patch());

    // Initialize hardware components
    init_backlight_gpio();
    init_spi_bus();
    
    esp_lcd_panel_io_handle_t io_handle = init_panel_io();
    esp_lcd_panel_handle_t panel_handle = init_lcd_panel(io_handle);
    
    configure_lcd_panel(panel_handle);
    
    // Initialize touch controller (XPT2046 shares SPI with LCD)
    esp_lcd_touch_handle_t touch_handle = init_touch_controller();
    
    if (touch_handle == NULL) {
        ESP_LOGW(TAG, "Touch controller initialization failed - continuing without touch");
        ESP_LOGW(TAG, "Display will work but touch will not be available");
    }
    
    enable_backlight();

    // Initialize LVGL
    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();
    
    init_lvgl_tick_timer();
    
    lv_color_t *buf1 = NULL;
    lv_color_t *buf2 = NULL;
    allocate_lvgl_buffers(&buf1, &buf2);
    
    init_lvgl_display(panel_handle, buf1, buf2);
    
    // Only initialize touch input if touch controller was successfully initialized
    if (touch_handle != NULL) {
        init_lvgl_touch_input(touch_handle);
    }

    // Create UI
    create_ui();

    ESP_LOGI(TAG, "Display and touch initialized successfully!");

    // Run main loop
    lvgl_task_loop();
}
