#include "i2c-lcd.h"

#if defined(LCD_MAP_A)
// Bardzo popularny PCF8574 backpack:
// P0=RS, P1=RW, P2=EN, P3=BL, P4..P7=D4..D7
#define PIN_RS 0x01
#define PIN_RW 0x02
#define PIN_EN 0x04
#define PIN_BL 0x08
#elif defined(LCD_MAP_B)
// Spotykany alternatywny:
// P4=RS, P5=RW, P6=EN, P3=BL (czasem), dane różnie
#define PIN_RS 0x10
#define PIN_RW 0x20
#define PIN_EN 0x40
#define PIN_BL 0x08
#else
#error "Select LCD_MAP_A or LCD_MAP_B"
#endif

#if LCD_DEBUG
volatile HAL_StatusTypeDef lcd_last_status = HAL_OK;
volatile uint32_t lcd_last_error = 0;
volatile uint32_t lcd_err_count = 0;
#endif

static uint8_t lcd_bl_mask = PIN_BL;

static void lcd_i2c_write(uint8_t data)
{
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c4, SLAVE_ADDRESS_LCD, &data, 1, 100);

#if LCD_DEBUG
    lcd_last_status = st;
    lcd_last_error  = HAL_I2C_GetError(&hi2c4);
    if (st != HAL_OK) {
        lcd_err_count++;
    }
#endif
}

static void lcd_pulse_enable(uint8_t data)
{
    lcd_i2c_write(data | PIN_EN);
    lcd_i2c_write(data & (uint8_t)~PIN_EN);
}

static void lcd_send_nibble(uint8_t nibble, uint8_t rs)
{
    // nibble to 4-bit (0..15)
    uint8_t data_bits = (uint8_t)((nibble & 0x0F) << (4 - LCD_DATA_SHIFT));
    // Wyjaśnienie:
    // LCD_DATA_SHIFT=0  -> nibble <<4  (trafiasz w P4..P7)
    // LCD_DATA_SHIFT=4  -> nibble <<0  (trafiasz w P0..P3)

    uint8_t ctrl = lcd_bl_mask | (rs ? PIN_RS : 0);
    uint8_t out  = (data_bits & 0xF0) | ctrl;   // bierzemy górną połówkę po przesunięciu

    lcd_pulse_enable(out);
}

static void lcd_send_byte(uint8_t byte, uint8_t rs)
{
    lcd_send_nibble((byte >> 4) & 0x0F, rs);
    lcd_send_nibble(byte & 0x0F, rs);
}

void lcd_init(void)
{
    HAL_Delay(50);

    // init sekwencja HD44780 w 4-bit przez expander
    // UWAGA: na początku wysyłamy tylko nible high (jak klasycznie)

    // 0x3,0x3,0x3
    lcd_send_nibble(0x03, 0);
    HAL_Delay(5);
    lcd_send_nibble(0x03, 0);
    HAL_Delay(5);
    lcd_send_nibble(0x03, 0);
    HAL_Delay(5);

    // 0x2 -> 4-bit
    lcd_send_nibble(0x02, 0);
    HAL_Delay(5);

    // Function set: 4-bit, 2-line/multi-line, 5x8
    lcd_send_byte(0x28, 0);
    HAL_Delay(1);

    // Display OFF
    lcd_send_byte(0x08, 0);
    HAL_Delay(1);

    // Clear
    lcd_send_byte(0x01, 0);
    HAL_Delay(2);

    // Entry mode
    lcd_send_byte(0x06, 0);
    HAL_Delay(1);

    // Display ON
    lcd_send_byte(0x0C, 0);
    HAL_Delay(1);
}

void lcd_clear(void)
{
    lcd_send_byte(0x01, 0);
    HAL_Delay(2);
}

void lcd_put_cur(int row, int col)
{
    if (col < 0) col = 0;
    if (col >= LCD_COLS) col = LCD_COLS - 1;
    if (row < 0) row = 0;
    if (row >= LCD_ROWS) row = LCD_ROWS - 1;

    uint8_t addr;
    switch (row)
    {
        case 0: addr = 0x80 + col; break;
        case 1: addr = 0xC0 + col; break;
        case 2: addr = 0x94 + col; break;
        case 3: addr = 0xD4 + col; break;
        default: addr = 0x80 + col; break;
    }
    lcd_send_byte(addr, 0);
}

void lcd_send_string(char *str)
{
    while (*str) {
        lcd_send_byte((uint8_t)*str++, 1);
    }
}

void lcd_send_string_at(int row, int col, char *str)
{
    lcd_put_cur(row, col);
    lcd_send_string(str);
}

