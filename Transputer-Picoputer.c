#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"

#include "arithmetic.h"
#include "pico_transputer_config.h"
#include "processor.h"
#include "server.h"

#include "oled096.h"

#include "picoputer.pio.h"

#if TRANS_USE_SPI_RAM
#include "spi_ram.h"
#endif

#define DEBUG_STOP            \
  do {                        \
    volatile int x = 1;       \
    while (x) {               \
    }                         \
  } while (0)

#define MEM_BYTE_MASK 0x0000ffff

static I2C_SLAVE_DESC oled0 = {
    .i2c = TP3_OLED_I2C,
    .sda_pin = TP3_OLED_SDA_PIN,
    .scl_pin = TP3_OLED_SCL_PIN,
    .slave_7bit_addr = TP3_OLED_ADDRESS,
    .baudrate = TP3_OLED_BAUDRATE,
};

extern unsigned long WPtr;
extern unsigned long ProcPriority;

extern void server_simkey(void);

void link_in_booting_ack(int i);
void link_in_booting_data(int i, int data);

static void prepare_boot_mode(void);

#if 0
/* Write a byte to memory. */
inline void writebyte (unsigned long ptr, unsigned char value)
{
	/* Write byte, ensuring memory reference is in range. */
	mem[(ptr & MEM_BYTE_MASK)]   = value;
}
#endif

int analyse = false;
int copy = false;
int exitonerror = false;
int peeksize = 8;

int membits = 0;

uint32_t profile[10];
int profiling = false;
int tracing = 0;
int emudebug = false;
int memdebug = false;
int memnotinit = false;
int msgdebug = false;
char *dbgtrigger = NULL;
int usetvs = false;

uint clock_offset;
uint linkout_offset;
uint linkin_offset;

/*
 * External reset integration
 * --------------------------
 * The board-level signal is notReset: low asserts reset, high releases it.
 * The GPIO interrupt only latches this flag.  All processor, server and PIO
 * state changes are deliberately performed later in normal execution context.
 */
volatile int transputer_reset_requested = 0;

static void transputer_reset_gpio_irq(uint gpio, uint32_t events) {
  (void)events;

  if (gpio == TP3_RESET_PIN) {
    transputer_reset_requested = 1;
  }
}

// TODO check T800 and memory start
void init_transputer(void) {
  Txxx = 800;

  CoreSize = TRANS_CORE_SIZE;
  MemStart = TRANS_MEM_START;
  ExtMemStart = TRANS_EXT_MEM_START;

#if TRANS_USE_SPI_RAM
  spi_ram_init_default();
#endif

  printf("Transputer memory backend: %s\n", TRANS_MEM_BACKEND_NAME);
  printf("Core RAM: %lu bytes, external emulated RAM: %lu bytes\n",
         (unsigned long)TRANS_CORE_SIZE, (unsigned long)TRANS_EXT_MEM_SIZE);

  while ((WPtr & 0x00000003) != 0x00000000)
    WPtr++;

  ProcPriority = LoPriority;

  // Start the transputer going
  // mainloop();
}

////////////////////////////////////////////////////////////////////////////////
//
// Boot sequence handlers
//
////////////////////////////////////////////////////////////////////////////////

int boot_write_index = 0;
int boot_length_remaining = 0;
int boot_link = -1;
int booting = 0;
int boot_done = 0;

void link_in_boot_load(int i, int data) {
  // Another byte has been received, load into memory
  // printf("\nWriting %02X to %08X", data, MemStart + boot_write_index);

  writebyte_int((MemStart + boot_write_index), data);

  // Update
  WPtr = MemStart + boot_write_index;

  // Point to next byte
  boot_write_index++;

  // One more byte done
  boot_length_remaining--;

  if (boot_length_remaining == 0) {
    // we are done
    boot_done = 1;
  }
}

// First data byte received, this is the length

/*
 * Link bootstrap protocol
 * -----------------------
 * The first data byte is the number of bootstrap bytes to load.  The receive
 * handlers are switched to the memory loader before the length-byte ACK is
 * sent, because the host may start the next 20-Mbit/s packet immediately after
 * observing that ACK.
 */
void link_in_boot_start_data(int i, int data) {

  // We have a length byte on a link, start loading data into memory
  boot_link = i;

  printf("Boot start data (length):%02X\n", data);

  if (data >= 2) {
    boot_length_remaining = data;

    // Switch to the memory loader handler from now on
    booting = 1;

    // printf("Booting, moving to download handlers...\n");
    link_in_data_fp = link_in_booting_data;
    link_in_ack_fp = link_in_booting_ack;

    // ACK the byte
    send_ack_to_link(i);

    /* One compact diagnostic line per bootstrap. */
    printf("B:%02X\n", data);
  } else {
    // This is a debug peek or poke
    // Handle later maybe
  }
}

void link_in_boot_start_ack(int i) {
  // This shouldn't happen, just ignore it
}

//--------------------------------------------------------------------------------
// We are now loading bytes into memory

void link_in_booting_data(int i, int data) {
  // We only deal with the link that we ar ebooting from
  if (i == boot_link) {
    // Store byte in memory
    link_in_boot_load(i, data);

    // ACK the byte
    send_ack_to_link(i);
  } else {
    // Ignore
  }
}

void link_in_booting_ack(int i) {}

////////////////////////////////////////////////////////////////////////////////
//
// Startup the Picoputer
//
////////////////////////////////////////////////////////////////////////////////

unsigned char byte_int(uint32_t ptr);
INLINE void writebyte_int(uint32_t ptr, unsigned char value);

/*
 * Reset service for the simulated transputer
 * ------------------------------------------
 * Reset stops and flushes the two link state machines, holds LinkOut at its
 * inactive low level, clears processor/link runtime state, and waits until the
 * active-low notReset input is released.  The PIO programs remain resident;
 * their saved offsets are used to restart the state machines.  The separate
 * 5-MHz C011 clock state machine continues to run, and emulated RAM is
 * retained.
 */
static void service_transputer_reset(PIO pio, uint linkout_sm,
                                     uint linkout_offset, uint linkin_sm,
                                     uint linkin_offset) {
  printf("RESET+\n");

  /*
   * Stop both serial link engines.
   */
  pio_sm_set_enabled(pio, linkout_sm, false);
  pio_sm_set_enabled(pio, linkin_sm, false);

  /*
   * Discard incomplete packets and acknowledgements.
   */
  pio_sm_clear_fifos(pio, linkout_sm);
  pio_sm_clear_fifos(pio, linkin_sm);

  /*
   * Hold LinkOut at the inactive low level while Reset is active.
   */
  gpio_init(TP3_LINK_OUT_PIN);
  gpio_put(TP3_LINK_OUT_PIN, 0);
  gpio_set_dir(TP3_LINK_OUT_PIN, GPIO_OUT);

  /*
   * Ignore LinkIn while Reset is active.
   */
  gpio_init(TP3_LINK_IN_PIN);
  gpio_set_dir(TP3_LINK_IN_PIN, GPIO_IN);
  gpio_pull_down(TP3_LINK_IN_PIN);

  processor_reset_runtime();
  server_reset_runtime();

  /*
   * Reset input is active low at the Pico interface.
   * Remain in reset until the signal returns high.
   */
  while (!gpio_get(TP3_RESET_PIN)) {
    tight_loop_contents();
  }

  /*
   * Clear the latched request before enabling the links again.
   */
  transputer_reset_requested = 0;

  /*
   * Restart both state machines at the start of their programs.
   * The PIO programs themselves remain loaded in instruction memory.
   */
  picoputerlinkout_program_init(pio, linkout_sm, linkout_offset,
                                TP3_LINK_OUT_PIN);

  picoputerlinkin_program_init(pio, linkin_sm, linkin_offset, TP3_LINK_IN_PIN);

  printf("RESET-\n");
}

/*
 * The system clock must be selected before stdio_init_all().  UART baud-rate
 * divisors are calculated during stdio initialization; changing clk_sys
 * afterwards produces unreadable serial output.
 */
int main() {
  set_sys_clock_khz(TP3_SYS_CLOCK_KHZ, true);

  stdio_init_all();

  char line[24];

  uint linkout_sm;
  PIO linkout_pio;

  /* INMOS links are inactive at logic low. Hold LinkOut low immediately. */
  gpio_init(TP3_LINK_OUT_PIN);
  gpio_put(TP3_LINK_OUT_PIN, 0);
  gpio_set_dir(TP3_LINK_OUT_PIN, GPIO_OUT);

  bi_decl(bi_program_description("This is a test binary."));
  bi_decl(bi_1pin_with_name(TP3_LED_PIN, "On-board LED"));

  printf("\nPicoputer\n");

  oled_setup(&oled0);

  oled_set_xy(&oled0, 0, 0);
  oled_display_string(&oled0, "Picoputer");

  oled_set_xy(&oled0, 0, 14);
  oled_display_string(&oled0, "I2C OLED Display");

  uint32_t sys_clock_hz = clock_get_hz(clk_sys);
  snprintf(line, sizeof(line), "%lu Hz", (unsigned long)sys_clock_hz);
  oled_set_xy(&oled0, 0, 21);
  oled_display_string(&oled0, line);

  printf("clk_sys: %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

  printf("\nPicoputer GPIO init\n");

  gpio_init(TP3_LED_PIN);
  gpio_set_dir(TP3_LED_PIN, GPIO_OUT);
  gpio_put(TP3_LED_PIN, 0);

  /*
   * The generated PIO init helpers configure GPIO function and direction.
   * Do not assign GPIO_FUNC_PIO0/PIO1 manually here: TP3_LINK_PIO is the
   * single source of truth for the selected PIO instance.
   */

  /*
   * The external interface exposes notReset, not an active-high Reset.
   * Use a pull-up for the released state and latch the falling edge as reset
   * assertion.  The level check also covers reset already being active when
   * the interrupt is enabled.
   */
  gpio_init(TP3_RESET_PIN);
  gpio_set_dir(TP3_RESET_PIN, GPIO_IN);
  gpio_pull_up(TP3_RESET_PIN);

  gpio_set_irq_enabled_with_callback(TP3_RESET_PIN, GPIO_IRQ_EDGE_FALL, true,
                                     transputer_reset_gpio_irq);

  /*
   * Also handle the case where Reset was already low when the
   * interrupt was enabled.
   */
  printf("Reset input initial level: %u\n", (unsigned)gpio_get(TP3_RESET_PIN));

  if (!gpio_get(TP3_RESET_PIN)) {
    printf("Reset active at startup\n");
    transputer_reset_requested = 1;
  }

#if 1

  // Initialise the link server
  server_init();

  // PIO stuff
  // todo get free sm
  PIO pio = TP3_LINK_PIO;

  // exit(0);
  //  Choose which PIO instance to use (there are two instances)

  // Our assembled program needs to be loaded into this PIO's instruction
  // memory. This SDK function will find a location (offset) in the
  // instruction memory where there is enough space for our program. We need
  // to remember this location!
  /*
   * Keep each program offset for the lifetime of the firmware.  Reset only
   * reinitializes LinkIn and LinkOut state machines; it does not reload PIO
   * instruction memory or disturb the continuously running C011 clock.
   */
  clock_offset = pio_add_program(pio, &picoputerclk_program);

  // Find a free state machine on our chosen PIO (erroring if there are
  // none). Configure it to run our program, and start it, using the
  // helper function we included in our .pio file.
  uint sm = pio_claim_unused_sm(pio, true);
  picoputerclk_program_init(pio, sm, clock_offset, TP3_LINK_CLOCK_PIN);

  //--------------------------------------------------------------------------------
  // Set up link out state machine

  sm = pio_claim_unused_sm(pio, true);

  linkout_sm = sm;
  linkout_pio = pio;

  linkout_offset = pio_add_program(pio, &picoputerlinkout_program);
  picoputerlinkout_program_init(pio, sm, linkout_offset, TP3_LINK_OUT_PIN);

  // Tie state machine to link
  server_linkout_init(0, pio, sm);

  // The link out should now be running. Send a byte to the link. The IMSC011
  // should see that byte, and present it on its Q outputs. It should then take
  // QVALID active, which the Mega monitor should see.

  sm = pio_claim_unused_sm(pio, true);
  uint linkin_sm = sm;
  PIO linkin_pio = pio;
  linkin_offset = pio_add_program(pio, &picoputerlinkin_program);
  picoputerlinkin_program_init(pio, sm, linkin_offset, TP3_LINK_IN_PIN);

  // Tie state machine to link
  server_linkin_init(0, pio, sm);

  //--------------------------------------------------------------------------------
  //
  // Set up link in state machine
  //

  int qval = 0;
  uint32_t data;

  //--------------------------------------------------------------------------------
  // Initialise transputer emulator
  //

  init_transputer();

  // Set up a keypress to kick the in intruction
  server_simkey();

#define LOOPBACK 0

  // Wait for things to settle (link adapter clock etc)
  sleep_ms(500);

  // We sit in a bootstrap mode, waiting for data over the link
  // That data is a standard transputer 'boot from link' stream.
  //
  // Wait for a byte on a link (just link 0 for now)
  // If byte >= 2 then load byte number of bytes into memory from
  // the link
  // If <2 then debug features are handled

  // Wait for a byte from the link
  // Set up data and ack handlers

  //    link_in_data_fp = link_in_boot_start_data;
  //    link_in_ack_fp = link_in_boot_start_ack;
  //
  //    printf("Entering boot loop...\n");
  //
  //    while (!boot_done)
  //    {
  //        // Process links
  //        bootloop();
  //    }
  //
  //    printf("Entering main loop...\n");

  /*
   * Top-level simulated-transputer lifecycle:
   *
   *   BOOT -> RUN -> external notReset -> reset service -> BOOT
   *
   * This loop permits repeated iserver bootstrap sessions without rebooting the
   * RP2350.  Both the bootstrap loop and processor core observe the latched
   * reset request so reset can interrupt either state.
   */
  while (1) {
    prepare_boot_mode();

    printf("BOOT\n");

    /*
     * Wait for a bootstrap, but remain responsive to external Reset.
     */
    while (!boot_done && !transputer_reset_requested) {
      bootloop();
    }

    if (transputer_reset_requested) {
      service_transputer_reset(pio, linkout_sm, linkout_offset, linkin_sm,
                               linkin_offset);

      continue;
    }

    printf("RUN:%02X\n", boot_write_index);

    /*
     * mainloop() normally remains here while the program runs.
     * It returns when the Reset flag is detected, or for an existing
     * emulator exit condition.
     */
    while (!transputer_reset_requested) {
      mainloop();

      if (!transputer_reset_requested) {
        /*
         * Preserve the existing behavior if mainloop() returns for
         * a reason other than external Reset.
         */
        linkloop();
      }
    }

    service_transputer_reset(pio, linkout_sm, linkout_offset, linkin_sm,
                             linkin_offset);
  }

  //    while (1)
  //    {
  //        char line[80];
  //
  //        //----------------------------------------
  //        // Run the transputer
  // #if !LOOPBACK
  //        mainloop();
  //
  //        // And the links, the server is now on whatever is on the other
  //        // end of the links
  //
  //        linkloop();
  // #endif
  //        //----------------------------------------
  // #if LOOPBACK
  //        qval++;
  //
  //        if (data = picoputerlinkin_get(linkin_pio, linkin_sm))
  //        {
  //            // We have data. The external LinkIn signal is not inverted.
  //            data >>= 22;
  //
  //            sprintf(line, "\ndata= %08X", data);
  //            printf(line);
  //
  //            if (data == 0)
  //            {
  //                // This is an ACK
  //                printf("\nACK");
  //            }
  //            else
  //            {
  //                // Data packet
  //                // Remove second stop bit in LSB
  //                data >>= 1;
  //
  //                // Mask out data, just in case
  //                data &= 0xff;
  //
  //                printf("\nDATA:%02X", data);
  //            }
  //
  //            // ACK: PIO generates H; a zero payload produces the following
  //            L. picoputerlinkout_program_putc(linkout_pio, linkout_sm,
  //            0x000); sleep_ms(10);
  //            // Data: second start bit in bit 0, D0..D7 in bits 1..8, stop=0.
  //            picoputerlinkout_program_putc(linkout_pio,
  //                                           linkout_sm,
  //                                           0x001 | ((data & 0xff) << 1));
  //        }
  // #endif
  //    }

  // And done for now
  exit(0);
#if 0
  while (true)
    {
      // Blink
      pio_sm_put_blocking(pio, sm, 1);
      sleep_ms(100);
      // Blonk
      pio_sm_put_blocking(pio, sm, 0);
      sleep_ms(100);
    }
#endif
#endif

  ////////////////////////////////////////////////////////////////////////////////

  // Our assembled program needs to be loaded into this PIO's instruction
  // memory. This SDK function will find a location (offset) in the
  // instruction memory where there is enough space for our program. We need
  // to remember this location!
  uint offsetpclk = pio_add_program(pio, &picoputerclk_program);

  // Find a free state machine on our chosen PIO (erroring if there are
  // none). Configure it to run our program, and start it, using the
  // helper function we included in our .pio file.
  uint smclk = pio_claim_unused_sm(pio, true);

  picoputerclk_program_init(pio, smclk, offsetpclk, TP3_LINK_CLOCK_PIN);
  // The state machine is now running. Any value we push to its TX FIFO will

  ////////////////////////////////////////////////////////////////////////////////

  int count = 0;

  // sd_test();

  while (1) {
    char line[80];

    sprintf(line, "Loop %d", count);
    gpio_put(TP3_LED_PIN, 0);

    sleep_ms(1000);
    gpio_put(TP3_LED_PIN, 1);

    puts(line);
    sleep_ms(500);

    oled_set_xy(&oled0, 0, 35);
    oled_display_string(&oled0, line);

    count++;
  }
}

/*
 * Restore only bootstrap bookkeeping and callbacks.  Processor registers,
 * server transfer state and PIO FIFOs are reset by their dedicated reset
 * helpers before this state is entered after an external reset.
 */
static void prepare_boot_mode(void) {
  boot_write_index = 0;
  boot_length_remaining = 0;
  boot_link = -1;
  booting = 0;
  boot_done = 0;

  link_in_data_fp = link_in_boot_start_data;
  link_in_ack_fp = link_in_boot_start_ack;
}