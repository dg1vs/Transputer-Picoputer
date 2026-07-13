#ifndef HELPER_OLED096_H
#define HELPER_OLED096_H

#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_CMD       1
#define I2C_DATA      0
#define I2C_REPEAT    1
#define I2C_NO_REPEAT 0

typedef struct _I2C_SLAVE_DESC
{
    i2c_inst_t *i2c;
    uint sda_pin;
    uint scl_pin;
    uint8_t slave_7bit_addr;
    uint baudrate;
} I2C_SLAVE_DESC;

void oled_setup(I2C_SLAVE_DESC *slave);
void oled_send_cmd(I2C_SLAVE_DESC *slave, int n, const unsigned char *data, int command, int repeat);
void oled_set_xy(I2C_SLAVE_DESC *slave, int x, int y);
void oled_set_pixel_xy(I2C_SLAVE_DESC *slave, int x, int y);
void oled_set_byte_xy(I2C_SLAVE_DESC *slave, int x, int y, int b);
void oled_gap(I2C_SLAVE_DESC *slave);
void oled_display_int(I2C_SLAVE_DESC *slave, long int n, int num_digits);
void oled_display_string(I2C_SLAVE_DESC *slave, const char *string);
void oled_clear_display(I2C_SLAVE_DESC *slave);
void oled_display_scaled_string_xy(I2C_SLAVE_DESC *slave, const char *string, int x, int y, int scale);

#endif /* HELPER_OLED096_H */
