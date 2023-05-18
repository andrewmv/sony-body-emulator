/**
 * 2023/04 Andrew Villeneuve
 * Emulate a Sony camera body to a multi-interface shoe flash 
 * 
 * Pins: 
 * DATA on GPIO 0, 
 * CLOCK on GPIO 1, 
 * TRIG on GPIO 2
 * 
 * UART CLI:
 * Flash state changes are shown with F:
 * Body state changes are shown with B:
 * Commands:
 * r - toggle half-press
 * f - fire flash with current settings
 * pXX - set pre-flash power level to XX (00 - FF)
 * eXX - set exposure flash power level to XX (00 - FF)
 */

#include <stdio.h>
#include "body_emu.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "rx-miso.pio.h"
#include "tx-mosi.pio.h"

// Callback for RX DMA control chain (packet fully received)
void dma_callback() {
    dma_channel_acknowledge_irq0(miso_dma_chan);
    // Stop the PIO from trying to clock in any more data
    pio_sm_set_enabled(miso_pio, miso_sm, false);
    // Compare new rx'd packet to old one, and print it if they differ
    if (memcmp(old_flash_packet, new_flash_packet, flash_packet_length) != 0) {
        memcpy(old_flash_packet, new_flash_packet, flash_packet_length);
        flash_packet_updated = true;
    }
}

void uart_rx_callback() {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        uart_cmd_buffer[uart_cmd_buffer_pos] = ch;
        uart_cmd_buffer_pos++;
        if ((uart_cmd_buffer_pos >= uart_cmd_buffer_length) || (ch == '\r')) {
            // Enter was pressed
            // Treat overflow as Enter for simplicity
            process_uart_cmd();
            uart_cmd_buffer_pos = 0;
        } else {
            // Echo
            uart_putc(uart0, ch);
        }
    }
}

int64_t alarm_callback(alarm_id_t id, void *user_data) {
    state = STATE_READY_TIMEOUT;
    return 0;
}

// Helper functions for converting hex strings to bytes
uint8_t nibble(uint8_t c) {
	if (c >= '0' && c <= '9')
		return c - '0';

	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;

	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	return 0;  // Not a valid hexadecimal character
}

uint8_t strToByte(uint8_t highNibble, uint8_t lowNibble) {
	return (nibble(highNibble) << 4) + nibble(lowNibble);
}

void process_uart_cmd() {
    uint8_t cmd = uart_cmd_buffer[0];
    if (cmd == 'r') {
        // Toggle ready (shutter release half pressed)
        if (state == STATE_STANDBY) {
            state = STATE_READY;
            memcpy(body_packet, body_packet_ready, body_packet_length);
        } else if (state == STATE_READY) {
            state = STATE_STANDBY;
            memcpy(body_packet, body_packet_standby, body_packet_length);
        }
    } else if (cmd == 'p') {
        // Set pre-flash power level
        body_packet_pf[9] = strToByte(uart_cmd_buffer[1], uart_cmd_buffer[2]);
    } else if (cmd == 'e') {
        // Set adjusted exposure flash power level
        body_packet_ef[9] = strToByte(uart_cmd_buffer[1], uart_cmd_buffer[2]);
    } else if (cmd == 'f') {
        // Execute a flash metering transaction next cycle
        state = STATE_METERING_PF;
    } else if (cmd == 's') {
        // Set an arbitrary byte in the body data
        u_int8_t updatepos = (((uart_cmd_buffer[1] - '0') * 10) + uart_cmd_buffer[2] - '0');
        u_int8_t newbyte = strToByte(uart_cmd_buffer[3], uart_cmd_buffer[4]);
        body_packet[updatepos] = newbyte;
        body_packet_standby[updatepos] = newbyte;
        body_packet_ready[updatepos] = newbyte;
        body_packet_pf[updatepos] = newbyte;
        body_packet_ef[updatepos] = newbyte;
    }
    body_packet_updated = true;
}

void print_body_state(uint8_t* packet) {
    uart_puts(uart0, "\r\nB: ");
    for (int i = 0; i < body_packet_length; i++) {
        printf("%02X ", packet[i]);
    }
    uart_puts(uart0, " > ");
}

void print_flash_state() {
    uart_puts(uart0, "\r\nF: ");
    for (int i = 0; i < flash_packet_length; i++) {
        printf("%02X ", old_flash_packet[i]);
    }
    uart_puts(uart0, " > ");
}

// Take over control of the CLK GPIO and assert a pulse of time_us
void assert_clk(uint16_t time_us) {
    gpio_init(CLK);
    gpio_set_dir(CLK, GPIO_OUT);
    gpio_put(CLK, 1);
    sleep_us(time_us);
    gpio_put(CLK, 0);
}

// Pull TRIG low for 15ms to fire the flash
void assert_trig() {
    gpio_put(TRIG, 0);
    gpio_set_dir(TRIG, GPIO_OUT);
    sleep_ms(15);
    gpio_set_dir(TRIG, GPIO_IN);
}

// Start a simulation of a TTL metered flash
void simulate_ttl_flash() {
    // Send pre-flash metering initialization packet
    start_mosi_tx(body_packet_pf);
    uart_puts(uart0, "\r\nPF:");
    print_body_state(body_packet_pf);

    // Wait for packet to finish sending, then another 2.5ms
    sleep_us(4500);

    // Arm the pre-flash
    assert_clk(FLASH_READY_US);

    // Signal the pre-flash to strobe - a 90us clock pulse asserted by the body
    sleep_ms(30);
    assert_clk(MISO_INIT_US);

    // Spend ~50ms pretending to do TTL calculations
    state = STATE_METERING_EF;
    sleep_ms(50);

    // Send an adjusted exposure flash initialization packet
    start_mosi_tx(body_packet_ef);
    uart_puts(uart0, "\r\nEF:");
    print_body_state(body_packet_ef);
    sleep_us(4000 + 2500);

    // Arm the exposure flash
    assert_clk(FLASH_READY_US);

    // Trigger the exposure flash by pulling TRIG low
    sleep_ms(30);
    assert_trig();
}

// Configure the PIOs and DMA for a flash-to-body transfer
void start_miso_rx() {
    // Assert start pulse
    assert_clk(MISO_INIT_US);
    // Add 412us to the built-in 128us built-in delay between the start pulse and the first bit to get to 540us
    sleep_us(412);

    // Hand GPIO control to PIO
    pio_gpio_init(miso_pio, CLK);
    pio_gpio_init(miso_pio, DATA);

    // Restart and Enable PIO SM (sets ISR shift counter to Empty)
    uint32_t restart_mask = (1u << PIO_CTRL_SM_RESTART_LSB << miso_sm);
    restart_mask |= (1u << PIO_CTRL_SM_ENABLE_LSB << miso_sm);
    miso_pio->ctrl = restart_mask;

    // Force SM to beginning of program
    pio_sm_exec_wait_blocking(miso_pio, miso_sm, pio_encode_jmp(miso_offset)); 

    // Start DMA to empty RX FIFO
    dma_channel_set_write_addr(miso_dma_chan, new_flash_packet, true);      
}

// Configure the PIOs and DMA for a body-to-flash transfer
void start_mosi_tx(const u_int8_t *data) {
    // Assert start pulse
    assert_clk(MOSI_INIT_US);
    pio_gpio_init(mosi_pio, CLK);
    // Add 192us to the built-in 128us built-in delay between the start pulse and the first bit to get to 320us
    // Then add a bodge number
    sleep_us(192 - 16);

    // Attach DATA pin function to TX PIO and set direction
    pio_gpio_init(mosi_pio, DATA);
    pio_sm_set_consecutive_pindirs(mosi_pio, mosi_sm, DATA, 2, true);
    gpio_set_drive_strength(DATA, GPIO_DRIVE_STRENGTH_12MA);

    // Restart and Enable PIO SM (sets OSR shift counter to Empty)
    uint32_t restart_mask = (1u << PIO_CTRL_SM_RESTART_LSB << mosi_sm);
    restart_mask |= (1u << PIO_CTRL_SM_ENABLE_LSB << mosi_sm);
    mosi_pio->ctrl = restart_mask;

    // Force SM to beginning of program
    pio_sm_exec_wait_blocking(mosi_pio, mosi_sm, pio_encode_jmp(mosi_offset));

    // Start DMA to fill RX FIFO
    dma_channel_set_read_addr(mosi_dma_chan, data, true);
}

// Configure DMA to feed data to the PIO state machine for
// shifting into the flash (TX)
void mosi_dma_setup(PIO pio, uint sm, uint dma_chan) {
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    // We don't want DMA IRQs from TX transfers
    channel_config_set_irq_quiet(&c, true);

    dma_channel_configure(
        dma_chan,           // The channel to configure
        &c,                 // Configuration struct
        &pio->txf[sm],      // Destination pointer - PIO TX FIFO
        NULL,               // Source - will set later
        body_packet_length, // Transfer count (size of source array)
        false               // Don't start yet
    );
}

// Configure DMA to read data shifted out of the body from
// the PIO state machine (RX)
void miso_dma_setup(PIO pio, uint sm, uint dma_chan) {
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    const volatile void *rx_fifo_addr = (io_rw_8*)&pio->rxf[sm] + 3;

    dma_channel_configure(
        dma_chan,           // The channel to configure
        &c,                 // Configuration struct
        NULL,               // Destination - will set later
        rx_fifo_addr,       // Source - leftmost octet of PIO RX FIFO
        flash_packet_length, // Transfer count (size of source array)
        false               // Don't start yet
    );

    // Raise IRQ line 0 when the DMA transfer finishes
    dma_channel_set_irq0_enabled(dma_chan, true);
}

int main() {
    // Initialize outgoing packet
    memcpy(body_packet, body_packet_standby, body_packet_length);

    // Initialize last read packet to a reasonable value
    for (int i = 0; i < flash_packet_length; i++) {
        old_flash_packet[i] = 0;
    }

    // Initialize command buffer
    for (int i = 0; i < uart_cmd_buffer_length; i++) {
        uart_cmd_buffer[i] = 0x00;
    }

    // Setup Serial
    stdio_init_all();
    printf("Body Emulator Ready\n");
    irq_set_exclusive_handler(UART0_IRQ, uart_rx_callback);
    irq_set_enabled(UART0_IRQ, true);
    uart_set_irq_enables(uart0, true, false);

    // Setup GPIO
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    gpio_init(TRIG);
    gpio_set_dir(TRIG, GPIO_IN);

    // Setup PIO State Machine
    miso_offset = pio_add_program(miso_pio, &rx_miso_program);
    rx_miso_program_init(miso_pio, miso_sm, miso_offset, CLK, DATA);

    mosi_offset = pio_add_program(mosi_pio, &tx_mosi_program);
    tx_mosi_program_init(mosi_pio, mosi_sm, mosi_offset, CLK, DATA);

    // Setup DMA
    miso_dma_chan = dma_claim_unused_channel(true);
    miso_dma_setup(miso_pio, miso_sm, miso_dma_chan);

    mosi_dma_chan = dma_claim_unused_channel(true);
    mosi_dma_setup(mosi_pio, mosi_sm, mosi_dma_chan);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_callback);
    irq_set_enabled(DMA_IRQ_0, true);

    while(true) {
        // If anything changed last loop, print it to the console
        if (body_packet_updated) {
            print_body_state(body_packet);
            body_packet_updated = false;
        }

        if (flash_packet_updated) {
            print_flash_state();
            flash_packet_updated = false;
        }

        // Transmit a body packet consistent with our current mode and state
        start_mosi_tx(body_packet);
        sleep_ms(PACKET_INTERVAL_MS);

        // Read a packet out from the flash
        start_miso_rx();
        sleep_ms(PACKET_INTERVAL_MS);

        // Simulate post-exposure delay
        if (state == STATE_RECHARGE) {
            sleep_ms(200);
            state = STATE_STANDBY;
        }

        // Stop normal polling to do a TTL flash metering transaction
        if (state == STATE_METERING_PF) {
            sleep_ms(8);    // We need a double interval before the PF Init packet
            simulate_ttl_flash(0, 0);
            sleep_ms(20);
            state = STATE_RECHARGE;
        }
    }
}