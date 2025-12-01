idf_component_register(SRCS "my_lvgl_st7796_lcd.c"
                    INCLUDE_DIRS "."
                    REQUIRES lvgl esp_lcd esp_lcd_touch_xpt2046
                    PRIV_REQUIRES esp_timer
                  )
