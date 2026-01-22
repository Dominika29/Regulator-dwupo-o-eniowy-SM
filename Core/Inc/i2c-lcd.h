#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "stm32f7xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c4;

// ===== I2C addr =====
#define LCD_I2C_ADDR_7BIT   0x27
#define SLAVE_ADDRESS_LCD   (LCD_I2C_ADDR_7BIT << 1)   // 0x4E

// ===== LCD geometry =====
#define LCD_COLS 20
#define LCD_ROWS 4

// ===== DEBUG =====
#define LCD_DEBUG 1

#if LCD_DEBUG
extern volatile HAL_StatusTypeDef lcd_last_status;
extern volatile uint32_t lcd_last_error;
extern volatile uint32_t lcd_err_count;
#endif

// ===== MAP SELECTION =====
// 1) Najpierw testuj MAP_A. Jeśli nic -> MAP_B.
// Odkomentuj dokładnie jedną:
#define LCD_MAP_A
// #define LCD_MAP_B

// 2) Bardzo częsta różnica: gdzie są linie D4..D7 na PCF8574
// - wariant typowy: D4..D7 = P4..P7  (czyli nibble idzie w górne 4 bity)  -> LCD_DATA_SHIFT = 0
// - wariant rzadziej: D4..D7 = P0..P3  (nibble idzie w dolne 4 bity)      -> LCD_DATA_SHIFT = 4
//
// Zmień na 4, jeśli nadal masz tylko bloki.
#define LCD_DATA_SHIFT 0

// ===== API =====
void lcd_init(void);
void lcd_clear(void);
void lcd_put_cur(int row, int col);
void lcd_send_string(char *str);
void lcd_send_string_at(int row, int col, char *str);

#endif
