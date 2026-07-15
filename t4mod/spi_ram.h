#ifndef HELPER_SPI_RAM_H
#define HELPER_SPI_RAM_H

#include <stddef.h>
#include <stdint.h>
#include "hardware/spi.h"
// Todo add DEFINE
#include "pico_transputer_config.h"



/*
 * Compatibility names used by the SPI RAM implementation.  The actual board
 * assignment lives in pico2_pin_config.h.
 */
#define SPI_RAM_SPI       TP3_SPI_RAM_SPI
#define SPI_RAM_RX_PIN    TP3_SPI_RAM_RX_PIN
#define SPI_RAM_TX_PIN    TP3_SPI_RAM_TX_PIN
#define SPI_RAM_SCK_PIN   TP3_SPI_RAM_SCK_PIN
#define SPI_RAM_CSN_PIN   TP3_SPI_RAM_CSN_PIN
#define SPI_RAM_BAUDRATE  TP3_SPI_RAM_BAUDRATE

#define SPI_RAM_PAGE_SIZE 256u

void spi_ram_init_default(void);
void spi_ram_read(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t *buf, size_t len);
void spi_ram_write(spi_inst_t *spi, uint cs_pin, uint32_t addr, const uint8_t *data, size_t len);
void spi_ram_printbuf(const uint8_t *buf, size_t len);

/* Compatibility names used by the existing t4/p.c code. */
void ram_read(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t *buf, size_t len);
void ram_write(spi_inst_t *spi, uint cs_pin, uint32_t addr, uint8_t data[], size_t len);

#endif /* HELPER_SPI_RAM_H */
