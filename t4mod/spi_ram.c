#include "spi_ram.h"

#include <stdio.h>
#include "hardware/gpio.h"
#include "pico/binary_info.h"

#define SPI_RAM_CMD_READ   0x03u
#define SPI_RAM_CMD_WRITE  0x02u

static inline void spi_ram_cs_select(uint cs_pin)
{
    gpio_put(cs_pin, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void spi_ram_cs_deselect(uint cs_pin)
{
    asm volatile("nop \n nop \n nop");
    gpio_put(cs_pin, 1);
}

void spi_ram_init_default(void)
{
    spi_init(SPI_RAM_SPI, SPI_RAM_BAUDRATE);

    gpio_set_function(SPI_RAM_RX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RAM_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_RAM_TX_PIN, GPIO_FUNC_SPI);

    gpio_init(SPI_RAM_CSN_PIN);
    gpio_put(SPI_RAM_CSN_PIN, 1);
    gpio_set_dir(SPI_RAM_CSN_PIN, GPIO_OUT);

    bi_decl(bi_3pins_with_func(SPI_RAM_RX_PIN, SPI_RAM_TX_PIN, SPI_RAM_SCK_PIN, GPIO_FUNC_SPI));
    bi_decl(bi_1pin_with_name(SPI_RAM_CSN_PIN, "External SPI RAM CS"));
}

void spi_ram_read(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t *buf, size_t len)
{
    uint8_t cmdbuf[4] = {
        SPI_RAM_CMD_READ,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)addr
    };

    spi_ram_cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, sizeof(cmdbuf));
    spi_read_blocking(spi, 0, buf, len);
    spi_ram_cs_deselect(cs_pin);
}

void spi_ram_write(spi_inst_t *spi, uint cs_pin, uint32_t addr, const uint8_t *data, size_t len)
{
    uint8_t cmdbuf[4] = {
        SPI_RAM_CMD_WRITE,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)addr
    };

    spi_ram_cs_select(cs_pin);
    spi_write_blocking(spi, cmdbuf, sizeof(cmdbuf));
    spi_write_blocking(spi, data, len);
    spi_ram_cs_deselect(cs_pin);
}

void spi_ram_printbuf(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        printf("%02x", buf[i]);
        if ((i % 16u) == 15u) {
            printf("\n");
        } else {
            printf(" ");
        }
    }
    if ((len % 16u) != 0u) {
        printf("\n");
    }
}

/* Compatibility wrappers for the existing emulator code. */
void ram_read(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t *buf, size_t len)
{
    spi_ram_read(spi, cs_pin, addr, buf, len);
}

void ram_write(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t data[], size_t len)
{
    spi_ram_write(spi, cs_pin, addr, data, len);
}
