/** 
 * ST7735 LCD driver, as an ESP-IDF component
 *
 * Copyright (C) 2016 Marian Hrinko.
 * Written by Marian Hrinko (mato.hrinko@gmail.com)
 * Original repostory: https://github.com/Matiasus/ST7735.
 *
 * Adaptations to ESP-IDF (c) 2020 Ivan Grokhotkov
 */

#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "st7735_defs.h"

#include <stdbool.h>

#include "lvgl/lvgl.h"


#define MAX_Y 160
#define MIN_Y 1
#define MIN_X 26

// max rows
#define MAX_X (MIN_X + 79)
// columns max counter
#define SIZE_X (MAX_X - MIN_X + 1)
// rows max counter
#define SIZE_Y (MAX_Y - MIN_Y + 1)

#define DISP_BUF_SIZE (SIZE_X * SIZE_Y)
#define ST7735_DC   CONFIG_LVGL_DISP_PIN_DC
#define ST7735_RST  CONFIG_LVGL_DISP_PIN_RST
#define ST7735_BCKL CONFIG_LVGL_DISP_PIN_BCKL

#define ST7735_ENABLE_BACKLIGHT_CONTROL CONFIG_LVGL_ENABLE_BACKLIGHT_CONTROL

#if CONFIG_LVGL_BACKLIGHT_ACTIVE_LVL
  #define ST7735_BCKL_ACTIVE_LVL 1
#else
  #define ST7735_BCKL_ACTIVE_LVL 0
#endif

// if text/images are backwards, try setting this to 1
#define ST7735_INVERT_DISPLAY CONFIG_LVGL_INVERT_DISPLAY

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

void st7735_init(void);
void st7735_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map);
void st7735_enable_backlight(bool backlight);


#ifdef __cplusplus
}
#endif
