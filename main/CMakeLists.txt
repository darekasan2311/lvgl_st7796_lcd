idf_component_register(SRCS "my_lvgl_st7796_lcd.c"
                    INCLUDE_DIRS "."
                    REQUIRES lvgl esp_lcd
                    PRIV_REQUIRES esp_timer
                  )
