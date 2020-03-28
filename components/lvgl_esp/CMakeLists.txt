set(SOURCES disp_spi.c ili9341.c tp_spi.c xpt2046.c st7735.c fonts/roboto_numbers_36.c)
idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS .
                       REQUIRES lvgl)
