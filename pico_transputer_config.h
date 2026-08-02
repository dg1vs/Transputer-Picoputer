#ifndef PICO_TRANSPUTER_CONFIG_H
#define PICO_TRANSPUTER_CONFIG_H

#include <stdint.h>
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/spi.h"

// Ascii-Art for huge text by
// https://patorjk.com/software/taag/#p=display&f=Big&t=Memory&x=none&v=4&h=4&w=80&we=false
// 
//  _____ _____ _   _         _____             __ _       
// |  __ \_   _| \ | |       / ____|           / _(_)      
// | |__) || | |  \| |______| |     ___  _ __ | |_ _  __ _ 
// |  ___/ | | | . ` |______| |    / _ \| '_ \|  _| |/ _` |
// | |    _| |_| |\  |      | |___| (_) | | | | | | | (_| |
// |_|   |_____|_| \_|       \_____\___/|_| |_|_| |_|\__, |
//                                                    __/ |
//                                                   |___/ 
// 
// Raspberry Pi Pico 2 pin assignment for transputer03.
// 
// Keep all board-level GPIO and peripheral choices in this file.  Driver
// headers should describe APIs, not select the pins used by this board.
// 
// Signal                 GPIO    Pico 2 physical pin
// --------------------------------------------------
// Console UART TX        GP0     1
// Console UART RX        GP1     2
// OLED I2C0 SDA          GP4     6
// OLED I2C0 SCL          GP5     7
// SPI RAM MISO/RX        GP12    16
// SPI RAM CS             GP13    17
// SPI RAM SCK            GP14    19
// SPI RAM MOSI/TX        GP15    20
// Transputer LinkIn      GP16    21
// Transputer LinkOut     GP17    22
// Transputer LinkClock   GP18    24    5 MHz
// Transputer Reset       GP19    25    active high
// ...
// On-board LED           GP25    internal


// stdio UART is enabled in CMakeLists.txt and uses the Pico 2 defaults. */
#define TP3_CONSOLE_UART_TX_PIN  PICO_DEFAULT_UART_TX_PIN // GP0
#define TP3_CONSOLE_UART_RX_PIN  PICO_DEFAULT_UART_RX_PIN // GP1

// On-board status LED
#define TP3_LED_PIN              PICO_DEFAULT_LED_PIN

// SSD1306 OLED display. GP4/GP5 are an I2C0 pin pair. 
#define TP3_OLED_I2C             i2c0
#define TP3_OLED_SDA_PIN         4u
#define TP3_OLED_SCL_PIN         5u
#define TP3_OLED_ADDRESS         0x3cu
#define TP3_OLED_BAUDRATE        400000u

// Transputer serial link implemented by PIO0. 
#define TP3_LINK_PIO             pio0
#define TP3_LINK_IN_PIN          16u
#define TP3_LINK_OUT_PIN         17u
#define TP3_LINK_CLOCK_PIN       18u


// The name is retained for compatibility, but the connected interface signal
// is notReset: GP19 low asserts reset and GP19 high releases it.
// TODO Change to Notreset???
#define TP3_RESET_PIN            19u

// Optional external SPI RAM on SPI1. 
#define TP3_SPI_RAM_SPI          spi1
#define TP3_SPI_RAM_RX_PIN       12u
#define TP3_SPI_RAM_CSN_PIN      13u
#define TP3_SPI_RAM_SCK_PIN      14u
#define TP3_SPI_RAM_TX_PIN       15u
#define TP3_SPI_RAM_BAUDRATE     1000000u


// Catch accidental GPIO reuse when this file is edited.  The Pico SDK uses
// GCC/Clang, where __builtin_popcountll() is a constant expression here.
#define TP3_GPIO_BIT(pin_)       (UINT64_C(1) << (pin_))
#define TP3_USED_GPIO_MASK                                                   \
    (TP3_GPIO_BIT(TP3_CONSOLE_UART_TX_PIN) |                                \
     TP3_GPIO_BIT(TP3_CONSOLE_UART_RX_PIN) |                                \
     TP3_GPIO_BIT(TP3_LED_PIN) |                                            \
     TP3_GPIO_BIT(TP3_OLED_SDA_PIN) |                                       \
     TP3_GPIO_BIT(TP3_OLED_SCL_PIN) |                                       \
     TP3_GPIO_BIT(TP3_LINK_IN_PIN) |                                        \
     TP3_GPIO_BIT(TP3_LINK_OUT_PIN) |                                       \
     TP3_GPIO_BIT(TP3_LINK_CLOCK_PIN) |                                     \
     TP3_GPIO_BIT(TP3_RESET_PIN) |                                          \
     TP3_GPIO_BIT(TP3_SPI_RAM_RX_PIN) |                                     \
     TP3_GPIO_BIT(TP3_SPI_RAM_CSN_PIN) |                                    \
     TP3_GPIO_BIT(TP3_SPI_RAM_SCK_PIN) |                                    \
     TP3_GPIO_BIT(TP3_SPI_RAM_TX_PIN))

_Static_assert(__builtin_popcountll(TP3_USED_GPIO_MASK) == 13,
               "pico2_pin_config.h assigns one GPIO to multiple functions");

// 
//  _      _       _           _____                     _         _____             __ _       
// | |    (_)     | |         / ____|                   | |       / ____|           / _(_)      
// | |     _ _ __ | | _______| (___  _ __   ___  ___  __| |______| |     ___  _ __ | |_ _  __ _ 
// | |    | | '_ \| |/ /______\___ \| '_ \ / _ \/ _ \/ _` |______| |    / _ \| '_ \|  _| |/ _` |
// | |____| | | | |   <       ____) | |_) |  __/  __/ (_| |      | |___| (_) | | | | | | | (_| |
// |______|_|_| |_|_|\_\     |_____/| .__/ \___|\___|\__,_|       \_____\___/|_| |_|_| |_|\__, |
//                                  | |                                                    __/ |
//                                  |_|                                                   |___/  
// 
// RP2350 system clock.
// 
// 160 MHz is required by the existing eight-PIO-cycles-per-bit
// implementation for a 20 Mbit/s link.
// Pico 2 is officially specified for 150 MHz, so this is technically an overclock.
// 
// 
// Link timing depends on selecting clk_sys before UART/PIO initialization.
// The 160-MHz clock gives exactly eight state-machine cycles per 20-Mbit/s bit.
// 
#define TP3_SYS_CLOCK_KHZ           160000u

// INMOS serial-link bit rate.
#define TP3_LINK_BIT_RATE_HZ        20000000u


// The current LinkIn and LinkOut PIO programs use eight state-machine
// cycles for each serial-link bit.
#define TP3_LINK_CYCLES_PER_BIT     8u

#define TP3_LINK_PIO_CLOCK_HZ       \
    (TP3_LINK_BIT_RATE_HZ * TP3_LINK_CYCLES_PER_BIT)


// The ClockOut of the picoputer is 5 MHz for both the 10 Mbit/s and 20 Mbit/s link modes.
#define TP3_C011_CLOCK_HZ           5000000u

_Static_assert(
    TP3_LINK_PIO_CLOCK_HZ <= TP3_SYS_CLOCK_KHZ * 1000u,
    "System clock is too low for configured link rate"
);

// 
//  __  __                                 
// |  \/  |                                
// | \  / | ___ _ __ ___   ___  _ __ _   _ 
// | |\/| |/ _ \ '_ ` _ \ / _ \| '__| | | |
// | |  | |  __/ | | | | | (_) | |  | |_| |
// |_|  |_|\___|_| |_| |_|\___/|_|   \__, |
//                                    __/ |
//                                   |___/ 
// 
// Select memory backend for the emulated transputer external memory.
// 
// 0 = use Pico/RP2040/RP2350 internal RAM array in t4/p.c
// 1 = use external SPI RAM through helper/spi_ram.c
// 
// Can also be overridden from CMake:
// target_compile_definitions(transputer03 PRIVATE TRANS_USE_SPI_RAM=1)
// 
#ifndef TRANS_USE_SPI_RAM
#define TRANS_USE_SPI_RAM 0
#endif

// Transputer memory map base values used by the current emulator port. 
#define TRANS_MEM_START       ((uint32_t)0x80000048u)
#define TRANS_CORE_SIZE       ((uint32_t)(2u * 1024u))
#define TRANS_EXT_MEM_START   ((uint32_t)0x80000800u)


// The current p.c memory access code uses address masking, so these sizes
// should be powers of two.
//
#define TRANS_PICO_RAM_SIZE   ((uint32_t)(64u * 1024u))
#define TRANS_SPI_RAM_SIZE    ((uint32_t)(2u * 1024u * 1024u))

#if TRANS_USE_SPI_RAM
    #define TRANS_EXT_MEM_SIZE    TRANS_SPI_RAM_SIZE
    #define TRANS_MEM_BACKEND_NAME "external SPI RAM"
#else
    #define TRANS_EXT_MEM_SIZE    TRANS_PICO_RAM_SIZE
    #define TRANS_MEM_BACKEND_NAME "Pico internal RAM"
#endif

#define TRANS_MEM_WORD_MASK   ((uint32_t)((TRANS_EXT_MEM_SIZE - 1u) & 0xFFFFFFFCu))
#define TRANS_MEM_BYTE_MASK   ((uint32_t)(TRANS_EXT_MEM_SIZE - 1u))

#endif /* PICO_TRANSPUTER_CONFIG_H */
