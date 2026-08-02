#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/binary_info.h"   // IWYU pragma: keep    --> suppress the vsc warning
#include "pico/stdlib.h"

#include "arithmetic.h"
#include "pico_transputer_config.h"
#include "processor.h"
#include "server.h"

#include "oled096.h"
#include "picoputer.pio.h"

#if TRANS_USE_SPI_RAM
  #include "spi_ram.h"
#endif

#define BOOT_DEBUG_POKE 0
#define BOOT_DEBUG_PEEK 1
#define BOOT_DEBUG_WORD_BYTES 4u

// States used while decoding the special bootstrap PEEK and POKE commands.
typedef enum {
  BOOT_DEBUG_IDLE = 0,
  BOOT_DEBUG_RECEIVE_ADDRESS,
  BOOT_DEBUG_RECEIVE_POKE_VALUE,
  BOOT_DEBUG_SEND_PEEK_VALUE
} boot_debug_state_t;

// Descriptor for the SSD1306 OLED used to display basic Picoputer status information.
static I2C_SLAVE_DESC status_display = {
    .i2c = TP3_OLED_I2C,
    .sda_pin = TP3_OLED_SDA_PIN,
    .scl_pin = TP3_OLED_SCL_PIN,
    .slave_7bit_addr = TP3_OLED_ADDRESS,
    .baudrate = TP3_OLED_BAUDRATE,
};

// T4 emulator workspace pointer; defined by the processor core and shared with this front end.
extern uint32_t WPtr;
// T4 emulator process-priority register; defined by the processor core and initialized here.
extern uint32_t ProcPriority;

// Legacy T4 command-line compatibility flag controlling analysis mode.
int analyse = false;
// Legacy T4 command-line compatibility flag controlling copy mode.
int copy = false;
// Legacy T4 command-line compatibility flag requesting termination after an emulator error.
int exitonerror = false;
// Legacy T4 setting selecting the number of bytes shown by a memory peek operation.
int peeksize = 8;
// Legacy T4 setting describing the configured memory-address width.
int membits = 0;
// Legacy T4 profiling counters; retained here because the emulator expects this global symbol.
int profile[10];
// Legacy T4 flag enabling collection of profiling information.
int profiling = false;
// Legacy T4 trace-control value used by the emulator's diagnostic output.
int tracing = 0;
// Legacy T4 flag enabling emulator-level debugging output.
int emudebug = false;
// Legacy T4 flag enabling memory-access debugging output.
int memdebug = false;
// Legacy T4 flag controlling diagnostics for accesses to uninitialized memory.
int memnotinit = false;
// Legacy T4 flag enabling emulator message/debug output.
int msgdebug = false;
// Legacy T4 flag controlling the emulator's test-vector support.
int usetvs = false;
// Legacy T4 optional debug-trigger string; NULL means that no trigger is configured.
char *dbgtrigger = NULL;


// Byte offset within emulated memory at which the next bootstrap byte will be stored.
static uint32_t bootstrap_write_offset = 0;
// Number of bootstrap bytes still expected after the one-byte bootstrap length field.
static uint8_t bootstrap_bytes_remaining = 0;
// Link currently used for bootstrap traffic; -1 means that no link has been selected yet.
static int8_t bootstrap_link = -1;
// Becomes true after the complete bootstrap image has been written to emulated memory.
static bool bootstrap_complete = false;
// 32-bit value being received by POKE or returned by PEEK.
static uint32_t boot_debug_word = 0;
// Current state of the bootstrap PEEK/POKE transaction state machine.
static boot_debug_state_t boot_debug_state = BOOT_DEBUG_IDLE;
// Current debug-bootstrap command: POKE, PEEK, or -1 when no command is active.
static int8_t boot_debug_command = -1;
// Little-endian 32-bit address assembled from incoming bootstrap debug bytes.
static uint32_t boot_debug_address = 0;
// Byte position, from 0 to 3, while assembling or transmitting a 32-bit debug word.
static uint8_t  boot_debug_byte_index = 0;
// PIO instance used by LinkOut; saved globally because the PEEK reply path also transmits data.
static PIO link_out_pio;
// PIO state-machine number assigned to LinkOut and reused by the bootstrap PEEK reply handler.
static uint link_out_sm;
// Latched request from the active-low external notReset signal; serviced outside interrupt context.
volatile bool transputer_reset_requested = false;
// Latched request from the active-low external notAnalyse; just a plaseholder.
volatile bool transputer_analyse_requested = false;

extern void server_simkey(void);

void link_in_bootstrap_start(int i, int data);
void link_in_bootstrap_start_ack(int i);
void link_in_bootstrap_ack(int i);
void link_in_bootstrap_data(int i, int data);


static void link_in_debug_data(int i, int data);
static void link_in_peek_ack(int i);
static void reset_boot_debug_state(void);
static void send_next_peek_reply_byte(void);
static void reset_bootstrap_state(void);


// GPIO interrupt callback for external Transputer control signals.
// The Pico SDK does not install an independent callback per GPIO with this function. 
// There is only one default GPIO IRQ callback per core. 
// gpio_set_irq_enabled_with_callback() enables the specified GPIO and 
// also replaces the callback for the whole core. 
// Raspberry Pi documents this explicitly: all GPIOs using this mechanism on one core share a single callback.
//
// --> notReset and notAnalyse must share this handler. The interrupt handler only latches requests; 
// emulator state changes are performed outside interrupt context.
static void transputer_control_gpio_irq_handler(uint gpio, uint32_t events) {
  (void)events;

  if (gpio == TP3_RESET_PIN) {
    transputer_reset_requested = true;
    return;
  }

  if (gpio == TP3_NOT_ANALYSE_PIN) {
    transputer_analyse_requested = true;
        printf("Something really gone wrong\n");
  }
}



// Initializes the T4 emulator configuration that depends on the Pico hardware and project settings.
// It selects the emulated memory layout, optionally initializes external SPI RAM, and reports the active backend.
// The routine also aligns the workspace pointer and establishes the initial low-priority processor state.
static void initialize_t4_emulator(void) {
  Txxx = 800;
  CoreSize = TRANS_CORE_SIZE;
  MemStart = TRANS_MEM_START;
  ExtMemStart = TRANS_EXT_MEM_START;

#if TRANS_USE_SPI_RAM
  spi_ram_init_default();
#endif

  printf("Transputer memory backend: %s\n", TRANS_MEM_BACKEND_NAME);
  // KS double check printf("Core RAM: %lu bytes, external emulated RAM: %lu bytes\n",
  printf("Core RAM: %" PRIu32 " bytes, external emulated RAM: %" PRIu32 " bytes\n",
         (uint32_t)TRANS_CORE_SIZE, 
         (uint32_t)TRANS_EXT_MEM_SIZE);

  while ((WPtr & 0x00000003) != 0x00000000)
    WPtr++;

  ProcPriority = LoPriority;
}



// Finishes the current bootstrap PEEK or POKE transaction and returns to normal bootstrap command decoding.
// All accumulated debug-command state is cleared so that a following transaction starts from a known condition.
// The normal bootstrap data and ACK callbacks are restored only after the current debug operation is complete.
static void reset_boot_debug_state(void) {
  boot_debug_state = BOOT_DEBUG_IDLE;
  boot_debug_command = -1;
  boot_debug_address = 0;
  boot_debug_word = 0;
  boot_debug_byte_index = 0;
  bootstrap_link = -1;

  link_in_data_fp = link_in_bootstrap_start;
  link_in_ack_fp = link_in_bootstrap_start_ack;
}


// Queues one byte of a 32-bit PEEK result for transmission through the INMOS LinkOut PIO state machine.
// The byte is encoded as payload bits expected by the LinkOut PIO program; framing is generated by PIO.
// PEEK replies are ACK-clocked, so this routine sends only the current byte and the ACK handler advances to the next.
static void send_next_peek_reply_byte(void) {
    const uint8_t reply_byte = (uint8_t)((boot_debug_word >> (boot_debug_byte_index * 8u)) & 0xffu);
    const int encoded_packet = 1 | ((int)reply_byte << 1);
    picoputerlinkout_program_putc(link_out_pio, link_out_sm, encoded_packet);
}


// Receives the data bytes that follow a bootstrap PEEK or POKE command on the selected INMOS link.
// Address and value words are reconstructed little-endian, matching the byte order used by a real transputer link.
// The routine performs the requested memory access and controls the ACK sequence or starts the PEEK reply transfer.
static void link_in_debug_data(int i, int data) {
  uint32_t received_byte;

  if (i != bootstrap_link) {
    return;
  }

  received_byte = (uint32_t)data & 0xffu;

  if (boot_debug_state == BOOT_DEBUG_RECEIVE_ADDRESS) {
    boot_debug_address |= received_byte << (boot_debug_byte_index * 8u);
    boot_debug_byte_index++;

    if (boot_debug_byte_index < BOOT_DEBUG_WORD_BYTES) {
      send_ack_to_link(i);
      return;
    }
    
    boot_debug_byte_index = 0;

    if (boot_debug_command == BOOT_DEBUG_POKE) {
      boot_debug_state = BOOT_DEBUG_RECEIVE_POKE_VALUE;
      send_ack_to_link(i);
      return;
    }

    if (boot_debug_command == BOOT_DEBUG_PEEK) {
      
      // Read the complete word before acknowledging the final address byte. 
      // The ACK and the first reply byte are then queued in that order on LinkOut.
      boot_debug_word = word_int(boot_debug_address);
      boot_debug_state = BOOT_DEBUG_SEND_PEEK_VALUE;
      send_ack_to_link(i);
      send_next_peek_reply_byte();
    } else {
      // data < 2 guarantees that this should never be reached.
      reset_boot_debug_state();
      send_ack_to_link(i);
    }
    return;
  }
  
  if (boot_debug_state == BOOT_DEBUG_RECEIVE_POKE_VALUE) {
    boot_debug_word |= received_byte << (boot_debug_byte_index * 8u);
    boot_debug_byte_index++;

    if (boot_debug_byte_index == BOOT_DEBUG_WORD_BYTES) {
      /*
       * Commit the value and restore the command decoder before the
       * final ACK. The host may send another debug command immediately.
       */
      writeword_int(boot_debug_address, boot_debug_word);
      reset_boot_debug_state();
    }

    send_ack_to_link(i);
  }
}


// Handles acknowledgements received while a four-byte bootstrap PEEK result is being transmitted.
// Each valid ACK advances the reply byte index and causes the next byte of the 32-bit value to be queued.
// After the fourth byte has been acknowledged, the debug transaction is finished and normal boot decoding resumes.
static void link_in_peek_ack(int i) {
  if ((i != bootstrap_link) || (boot_debug_state != BOOT_DEBUG_SEND_PEEK_VALUE)) {
    return;
  }

  boot_debug_byte_index++;

  if (boot_debug_byte_index < BOOT_DEBUG_WORD_BYTES) {
    send_next_peek_reply_byte();
  } else {
    reset_boot_debug_state();
  }
}


// Stores one received bootstrap byte into the emulated Transputer memory at the current download position.
// The workspace pointer and write index are advanced after every byte while the remaining bootstrap length is counted down.
// When the final byte has been stored, boot_done is asserted so the top-level lifecycle can leave the bootstrap loop.
void store_bootstrap_byte(int data) {
    
    hard_assert(data >= 0);
    hard_assert(data <= UINT8_MAX);

    // Another byte has been received, load into memory
    writebyte_int(MemStart + bootstrap_write_offset, (uint8_t)data);

    // Update
    WPtr = MemStart + bootstrap_write_offset;

    // Point to next byte
    bootstrap_write_offset++;

    // One more byte done
    bootstrap_bytes_remaining--;

    if (bootstrap_bytes_remaining == 0) {
        bootstrap_complete = true;
    }
}


// Decodes the first byte received while the emulator is waiting for a bootstrap command or bootstrap image.
// Values of two or more are treated as the bootstrap length; values zero and one select the POKE and PEEK commands.
// Callback handlers are switched before the ACK is sent so the 20-Mbit/s host may immediately transmit the next packet.
void link_in_bootstrap_start(int i, int data) {

  // We have a length byte on a link, start loading data into memory
  bootstrap_link = (int8_t)i;

  printf("Boot start data (length):%02X\n", data);

  if (data >= 2) {
    hard_assert(data >= 0);
    hard_assert(data <= UINT8_MAX);
    bootstrap_bytes_remaining = (uint8_t)data;

    // printf("Booting, moving to download handlers...\n");
    link_in_data_fp = link_in_bootstrap_data;
    link_in_ack_fp = link_in_bootstrap_ack;

    // ACK the byte
    send_ack_to_link(i);

    /* One compact diagnostic line per bootstrap. */
    printf("B:%02X\n", data);
  } else {
    
    // Standard transputer debug bootstrap commands:
    // 0: POKE, followed by address word and value word
    // 1: PEEK, followed by address word; return one value word
    boot_debug_command = (int8_t)data;
    boot_debug_state = BOOT_DEBUG_RECEIVE_ADDRESS;
    boot_debug_address = 0;
    boot_debug_word = 0;
    boot_debug_byte_index = 0;

    // Install the transaction handlers before ACKing the command byte.
    link_in_data_fp = link_in_debug_data;
    link_in_ack_fp = link_in_peek_ack;
    send_ack_to_link(i);
  }
}


// Handles an ACK received while the bootstrap decoder is still waiting for its first data byte.
// Such an ACK does not belong to a valid transaction in this state, so it is intentionally ignored.
// The link-number parameter is retained because this function must match the common ACK callback interface.
void link_in_bootstrap_start_ack(int i) {
  (void)i;
  // This shouldn't happen, just ignore it
}


// Processes data bytes while a normal bootstrap image is actively being downloaded into emulated memory.
// Only traffic from the link that supplied the bootstrap length byte is accepted; data from other links is ignored.
// Each accepted byte is stored through store_bootstrap_byte() and then acknowledged back to the transmitting link.
void link_in_bootstrap_data(int i, int data) {
  // We only deal with the link that we are booting from
  if (i == bootstrap_link) {
    // Store byte in memory
    store_bootstrap_byte(data);

    // ACK the byte
    send_ack_to_link(i);
  } else {
    // Ignore
  }
}


// Provides the ACK callback required while the emulator is receiving a normal bootstrap image.
// Incoming ACK packets are not meaningful for this receive-only phase, so no bootstrap state is changed here.
// The link-number parameter is intentionally unused but retained to satisfy the shared ACK callback signature.
void link_in_bootstrap_ack(int i) {
  (void)i;
}


// Legacy T4 internal-memory access helpers used by the bootstrap/debug code in this translation unit.
// byte_int() reads one byte from an emulated address; writebyte_int() stores one byte at an emulated address.
// Their implementations belong to the emulator memory layer, while these declarations make the interface explicit here.
unsigned char byte_int(uint32_t ptr);
INLINE void writebyte_int(uint32_t ptr, unsigned char value);


// Services an externally requested Transputer reset outside interrupt context and returns the link hardware to boot state.
// LinkIn and LinkOut PIO state machines are stopped and flushed while the pins are held at their inactive reset levels.
// After notReset is released, processor/server runtime state is cleared and both link PIO state machines are reinitialized.
static void service_transputer_reset(PIO pio, uint linkout_sm, uint linkout_offset, 
                                     uint linkin_sm, uint linkin_offset) {

  printf("RESET+\n");

  // Stop both serial link engines.
  pio_sm_set_enabled(pio, linkout_sm, false);
  pio_sm_set_enabled(pio, linkin_sm, false);
  
  // Discard incomplete packets and acknowledgements.
  pio_sm_clear_fifos(pio, linkout_sm);
  pio_sm_clear_fifos(pio, linkin_sm);

    // Hold LinkOut at the inactive low level while Reset is active.
  gpio_init(TP3_LINK_OUT_PIN);
  gpio_put(TP3_LINK_OUT_PIN, 0);
  gpio_set_dir(TP3_LINK_OUT_PIN, GPIO_OUT);

  //Ignore LinkIn while Reset is active.
  gpio_init(TP3_LINK_IN_PIN);
  gpio_set_dir(TP3_LINK_IN_PIN, GPIO_IN);
  gpio_pull_down(TP3_LINK_IN_PIN);

  processor_reset_runtime();
  server_reset_runtime();

  // Reset input is active low at the Pico interface.
  // Remain in reset until the signal returns high.
  while (!gpio_get(TP3_RESET_PIN)) {
    tight_loop_contents();
  }

  // Clear the latched request before enabling the links again.
  transputer_reset_requested = false;
  // Restart both state machines at the start of their programs.
  // The PIO programs themselves remain loaded in instruction memory.
  picoputerlinkout_program_init(pio, linkout_sm, linkout_offset, TP3_LINK_OUT_PIN);
  picoputerlinkin_program_init(pio, linkin_sm, linkin_offset, TP3_LINK_IN_PIN);

  printf("RESET-\n");
}


// Restores the software bookkeeping needed to accept a fresh bootstrap after startup or an external reset.
// Bootstrap counters, link selection, and completion flags are returned to their idle values before reception begins.
// Processor registers, server transfer state, and PIO FIFOs are intentionally handled by their dedicated reset routines.
static void reset_bootstrap_state(void) {
  bootstrap_write_offset = 0;
  bootstrap_bytes_remaining = 0;
  bootstrap_link = -1;
  bootstrap_complete = false;

  reset_boot_debug_state();
}


// Configures the Pico hardware, PIO link engines, user interface, and T4 emulator before entering the runtime lifecycle.
// The main lifecycle alternates between waiting for a bootstrap image, executing the emulated Transputer, and servicing reset.
// External notReset requests can interrupt both BOOT and RUN states so repeated iserver sessions do not require a Pico reboot.
int main(void) {

  // The system clock must be selected before stdio_init_all()! UART baud-rate divisors are calculated during stdio initialization; 
  // changing clk_sys afterwards produces unreadable serial output!
  // Anyway set the pico speed to 160 MHz
  set_sys_clock_khz(TP3_SYS_CLOCK_KHZ, true);

  stdio_init_all();

  bi_decl(bi_program_description("This is a test binary."));
  bi_decl(bi_1pin_with_name(TP3_LED_PIN, "On-board LED"));

  printf("\nPicoputer\n");

  oled_setup(&status_display);
  oled_set_xy(&status_display, 0, 0);
  oled_display_string(&status_display, "Picoputer");
  oled_set_xy(&status_display, 0, 14);
  oled_display_string(&status_display, "I2C OLED Display");
  oled_set_xy(&status_display, 0, 28);
  oled_display_string(&status_display, "0008");

  const uint32_t system_clock_hz = clock_get_hz(clk_sys);
  printf("clk_sys: %lu Hz\n", (unsigned long)system_clock_hz);
  printf("\nPicoputer GPIO init\n");

  // Default LED
  gpio_init(TP3_LED_PIN);
  gpio_set_dir(TP3_LED_PIN, GPIO_OUT);
  gpio_put(TP3_LED_PIN, 0);

  // INMOS links are inactive at logic low. Hold LinkOut low immediately. 
  gpio_init(TP3_LINK_OUT_PIN);
  gpio_put(TP3_LINK_OUT_PIN, 0);
  gpio_set_dir(TP3_LINK_OUT_PIN, GPIO_OUT);

  // The simulator exposes a TRAM notReset, not an active-high Transputer Reset.
  gpio_init(TP3_RESET_PIN);
  gpio_set_dir(TP3_RESET_PIN, GPIO_IN);
  gpio_pull_up(TP3_RESET_PIN);

  // setup irq for notReset
  gpio_set_irq_enabled_with_callback(TP3_RESET_PIN, GPIO_IRQ_EDGE_FALL, true, transputer_control_gpio_irq_handler);

  // edge case -- handle the case where Reset was already low when the interrupt was enabled
  printf("Reset input initial level: %u\n", (unsigned)gpio_get(TP3_RESET_PIN));

  if (!gpio_get(TP3_RESET_PIN)) {
    printf("Reset active at startup\n");
    transputer_reset_requested = true;
  }

  // The simulator exposes a TRAM notAnalse
  gpio_init(TP3_NOT_ANALYSE_PIN);
  gpio_set_dir(TP3_NOT_ANALYSE_PIN, GPIO_IN);
  gpio_pull_up(TP3_NOT_ANALYSE_PIN);

  // setup irq for notReset
  gpio_set_irq_enabled_with_callback(TP3_NOT_ANALYSE_PIN, GPIO_IRQ_EDGE_FALL, true, transputer_control_gpio_irq_handler);

  // edge case -- handle the case where Analyse was already low when the interrupt was enabled
  printf("Analyse input initial level: %u\n", (unsigned)gpio_get(TP3_NOT_ANALYSE_PIN));
  // Todo the same hadling as for reset?

  // Initialise the link server
  server_init();

  // PIO stuff
  PIO link_pio = TP3_LINK_PIO;
  // With all warnings on we need a convesersion and an assert
  // pio_add_program() can return a negative error, and pio_claim_unused_sm() can return -1 when required == false. 
  // Once successful, however, the offset and state-machine number are non-negative hardware indices.
  int pio_temp_result;
  
  // Our assembled program needs to be loaded into this PIO's instruction memory. 
  // This SDK function will find a location (offset) in the instruction memory where there 
  // is enough space for our program. We need to remember this location!
  pio_temp_result = pio_add_program(link_pio, &picoputerclk_program);
  hard_assert(pio_temp_result >= 0);
  const uint link_clock_program_offset = (uint)pio_temp_result;
  // Find a free state machine on our chosen PIO (erroring if there are  none). 
  // Configure it to run our program, and start it, using the helper function we included in our .pio file.
  pio_temp_result = pio_claim_unused_sm(link_pio, true);
  hard_assert(pio_temp_result >= 0);
  const uint link_clock_sm = (uint)pio_temp_result;
  
  picoputerclk_program_init(link_pio, link_clock_sm, link_clock_program_offset, TP3_LINK_CLOCK_PIN);

  // LinkOut
  pio_temp_result = pio_add_program(link_pio, &picoputerlinkout_program);
  hard_assert(pio_temp_result >= 0);
  const uint link_out_program_offset = (uint)pio_temp_result;
  
  pio_temp_result = pio_claim_unused_sm(link_pio, true);
  hard_assert(pio_temp_result >= 0);
  link_out_sm = (uint)pio_temp_result;
  
  link_out_pio = link_pio;
  picoputerlinkout_program_init(link_pio, link_out_sm, link_out_program_offset, TP3_LINK_OUT_PIN);
  // Tie state machine to link
  server_linkout_init(0, link_pio, link_out_sm);

  // LinkIn
  pio_temp_result = pio_add_program(link_pio, &picoputerlinkin_program);
  hard_assert(pio_temp_result >= 0);
  const uint link_in_program_offset = (uint)pio_temp_result;
  
  pio_temp_result = pio_claim_unused_sm(link_pio, true);
  hard_assert(pio_temp_result >= 0);
  const uint link_in_sm = (uint)pio_temp_result;
  
  picoputerlinkin_program_init(link_pio, link_in_sm, link_in_program_offset, TP3_LINK_IN_PIN);
  // Tie state machine to link
  server_linkin_init(0, link_pio, link_in_sm);

  // Initialise transputer emulator
  initialize_t4_emulator();

  // Set up a keypress to kick the in intruction
  server_simkey();

  // Wait for things to settle (link adapter clock etc)
  sleep_ms(500);
  

  // Top-level simulated-transputer lifecycle:
  // BOOT -> RUN -> external notReset -> reset service -> BOOT
  // This loop permits repeated iserver bootstrap sessions without rebooting the
  // RP2350.  Both the bootstrap loop and processor core observe the latched
  // reset request so reset can interrupt either state.
  
  while (true) {
    reset_bootstrap_state();

    printf("BOOT\n");
    
    // Wait for a bootstrap, but remain responsive to external Reset.
    while (!bootstrap_complete && !transputer_reset_requested) {
      bootloop();
    }

    if (transputer_reset_requested) {
      service_transputer_reset(link_pio, link_out_sm, link_out_program_offset, link_in_sm,
                               link_in_program_offset);

      continue;
    }

    
    printf("RUN:%02" PRIX32 "\n", bootstrap_write_offset);
    
    // mainloop() normally remains here while the program runs.
    // It returns when the Reset flag is detected, or for an existing
    // emulator exit condition.
    while (!transputer_reset_requested) {
      mainloop();

      if (!transputer_reset_requested) {
        // Preserve the existing behavior if mainloop() returns for a reason other than external Reset.
        linkloop();
      }
    }

    service_transputer_reset(link_pio, link_out_sm, link_out_program_offset, link_in_sm,
                             link_in_program_offset);
  }

  // And done for now
  exit(0);

}
