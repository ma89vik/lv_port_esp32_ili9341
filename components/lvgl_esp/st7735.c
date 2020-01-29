/** 
 * ST7735 LCD driver, as an ESP-IDF component
 *
 * Copyright (C) 2016 Marian Hrinko.
 * Written by Marian Hrinko (mato.hrinko@gmail.com)
 * Original repostory: https://github.com/Matiasus/ST7735.
 *
 * Adaptations to ESP-IDF (c) 2020 Ivan Grokhotkov
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "driver/gpio.h"
#include "st7735.h"
#include "st7735_defs.h"

#include "disp_spi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

static char *TAG = "st7735";

static void st7735_commands(const uint8_t *commands);

static void st7735_send_command(uint8_t);
static void st7735_send_data8(uint8_t);
static void st7735_send_data16(uint16_t);
static void st7735_delay_ms(uint8_t);

static void st7735_logical_to_lcd(int *x, int* y);

/** @array Init command */
static const uint8_t st7735_init_commands[] = {
    // 11 commands in list:
    11,
    // Software reset
    //  no arguments
    //  delay
    SWRESET, 
        DELAY,  
          200,  // 200 ms delay
    // Out of sleep mode, 
    //  no arguments, 
    //  delay
    SLPOUT,
        DELAY,  
          200,  // 200 ms delay
    // Set color mode, 
    //  1 argument
    //  delay
    COLMOD, 
      1+DELAY,  
         0x05,  // 16-bit color
           10,  // 10 ms
    // Frame rate control, 
    //  3 arguments
    //  delay
    FRMCTR1,
      3+DELAY,  
         0x00,  // fastest refresh
         0x06,  // 6 lines front porch
         0x03,  // 3 lines back porch
           10,  // 10 ms delay
    // Inversion mode off
     INVOFF,
        DELAY,  
           10,
    // Memory access ctrl (directions), 
    //  1 argument
    //  no delay
     MADCTL, 
            1,
         // D7  D6  D5  D4  D3  D2  D1  D0
         // MY  MX  MV  ML RGB  MH   -   -
         // ------------------------------
         // ------------------------------
         // MV  MX  MY -> {MV (row / column exchange) MX (column address order), MY (row address order)}
         // ------------------------------
         //  0   0   0 -> begin left-up corner, end right-down corner 
         //               left-right (normal view) 
         //  0   0   1 -> begin left-down corner, end right-up corner 
         //               left-right (Y-mirror)
         //  0   1   0 -> begin right-up corner, end left-down corner 
         //               right-left (X-mirror)
         //  0   1   1 -> begin right-down corner, end left-up corner
         //               right-left (X-mirror, Y-mirror)
         //  1   0   0 -> begin left-up corner, end right-down corner
         //               up-down (X-Y exchange)  
         //  1   0   1 -> begin left-down corner, end right-up corner
         //               down-up (X-Y exchange, Y-mirror)
         //  1   1   0 -> begin right-up corner, end left-down corner 
         //               up-down (X-Y exchange, X-mirror)  
         //  1   1   1 -> begin right-down corner, end left-up corner
         //               down-up (X-Y exchange, X-mirror, Y-mirror)
         // ------------------------------
         //  ML: vertical refresh order 
         //      0 -> refresh top to bottom 
         //      1 -> refresh bottom to top
         // ------------------------------
         // RGB: filter panel
         //      0 -> RGB 
         //      1 -> BGR        
         // ------------------------------ 
         //  MH: horizontal refresh order 
         //      0 -> refresh left to right 
         //      1 -> refresh right to left
         // 0xA0 = 1010 0000  
         (BIT(7) | BIT(6) | BIT(4) | BIT(3)),
    // Display settings #5, 
    //  2 arguments 
    //  no delay
    DISSET5, 
            2,  
         0x15,  // 1 clk cycle nonoverlap, 2 cycle gate
                // rise, 3 cycle osc equalize
         0x02,  // Fix on VTL
    // Display inversion control, 
    //  1 argument
    //  no delay
     INVCTR,
            1,  
          0x0,  //     Line inversion
    // Magical unicorn dust, 
    //  16 arguments
    //  no delay
    GMCTRP1,
           16,
         0x09, 
         0x16, 
         0x09, 
         0x20,
         0x21, 
         0x1B,
         0x13,
         0x19,
         0x17,
         0x15,
         0x1E,
         0x2B,
         0x04,
         0x05,
         0x02,
         0x0E,
    // Sparkles and rainbows 
    //  16 arguments 
    //  delay
    GMCTRN1,
     16+DELAY,  
         0x0B,
         0x14,
         0x08,
         0x1E, 
         0x22,
         0x1D,
         0x18,
         0x1E,
         0x1B,
         0x1A,
         0x24,  
         0x2B,
         0x06,
         0x06,
         0x02,
         0x0F,
           10,  // 10 ms delay
    // Normal display on
    //  no arguments
    //  delay
      NORON, 
        DELAY, 
           10,  // 10 ms delay
/*
    // Main screen turn on
    //  no arguments
    //  delay
     DISPON,
        DELAY,  
          200   // 200 ms delay
*/
};




void st7735_enable_backlight(bool backlight)
{
    uint32_t tmp = 0;

    tmp = backlight ? 1 : 0;

    gpio_set_level(ST7735_BCKL, tmp);

}


void st7735_init(void)
{
  gpio_config_t pins_config = {
        .pin_bit_mask = BIT64(ST7735_RST) | BIT64(ST7735_BCKL) | BIT64(ST7735_DC),
        .mode = GPIO_MODE_OUTPUT
  };
  ESP_ERROR_CHECK(gpio_config(&pins_config));
  gpio_set_level(ST7735_RST, 1);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  // load list of commands
  st7735_commands(st7735_init_commands);

  ESP_LOGE(TAG, "st7735 initialized");
}

static void st7735_commands(const uint8_t *commands)
{
  uint8_t milliseconds;
  uint8_t numOfCommands;
  uint8_t numOfArguments;

  // number of commands
  numOfCommands = *(commands++);
  
  // loop through whole command list
  while (numOfCommands--) {
    // send command
    st7735_send_command(*(commands++));
    // read number of arguments
    numOfArguments = *(commands++);
    // check if delay set
    milliseconds = numOfArguments & DELAY;
    // remove delay flag
    numOfArguments &= ~DELAY;
    // loop through number of arguments
    while (numOfArguments--) {
      // send arguments
      st7735_send_data8(*(commands++));
    }
    // check if delay set
    if (milliseconds) {
      // value in milliseconds
      milliseconds = *(commands++);
      // delay
      st7735_delay_ms(milliseconds);
    }
  }
}


static void st7735_send_command(uint8_t data)
{
  while(disp_spi_is_busy()) {}
    disp_spi_send_cmd(data);
}

static void st7735_send_data8(uint8_t data)
{
  while(disp_spi_is_busy()) {}
    disp_spi_send_data(&data, 1);
}

static void st7735_send_data16(uint16_t data)
{    
  while(disp_spi_is_busy()) {}
  disp_spi_send_color(&data, 2);

}

uint8_t st7735_set_window(uint8_t x0, uint8_t x1, uint8_t y0, uint8_t y1)
{
  // check if coordinates is out of range
  if ((x0 > x1)     ||
      (x1 > MAX_X) ||
      (y0 > y1)     ||
      (y1 > MAX_Y)) { 
    // out of range
    ESP_LOGE(TAG, "set window out of range, x0=%d, x1=%d, y0=%d, y1=%d", x0, x1, y0, y1);
    return 0;
  } 

  if ( x0 < MIN_X ) {
    ESP_LOGE(TAG, "set window out of range, MIN, x0=%d, x1=%d, y0=%d, y1=%d", x0, x1, y0, y1);
    return 0;

  } 
  // column address set
  st7735_send_command(CASET);
  // start x position
  st7735_send_data8(0x00);
  // start x position
  st7735_send_data8(x0);
  // start x position
  st7735_send_data8(0x00);
  // end x position
  st7735_send_data8(x1);
  // row address set
  st7735_send_command(RASET);
  // start x position
  st7735_send_data8(0x00);
  // start y position
  st7735_send_data8(y0);
  // start x position
  st7735_send_data8(0x00);
  // end y position
  st7735_send_data8(y1);
  // success

  
  st7735_send_command(DISPON);
  return 1;
}

void st7735_update_screen(void)
{
  // display on
  st7735_send_command(DISPON);
}

void st7735_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
  st7735_set_window(area->x1 + MIN_X, area->x2 + MIN_X, area->y1 + MIN_Y, area->y2 + MIN_Y);

  // access to RAM
  st7735_send_command(RAMWR);


	uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

  for (int i = 0; i<size; i++){
      st7735_send_data16(color_map[i].full);

  }

}




static void st7735_delay_ms(uint8_t ms)
{
  usleep(100*ms); /* 10 times shorter delays also seem to work */
}
