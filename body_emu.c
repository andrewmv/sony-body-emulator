/**
 * 2023/04 Andrew Villeneuve
 * Emulate a Sony camera body to a multi-interface shoe flash 
 * 
 * Pins: 
 * DATA on GPIO 0, 
 * CLOCK on GPIO 1, 
 * TRIG on GPIO 2
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
        for (int i = 0; i < flash_packet_length; i++) {
            printf("%02X ", new_flash_packet[i]);
        }
        printf("\n");
    }
}

void uart_rx_callback() {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        if (uart_is_writable(uart0)) {
            if (ch == '\r') {
                uart_puts(uart0, "\r\n");
            } else {
                uart_putc(uart0, ch);
            }
        }
    }
}

// Take over control of the CLK GPIO and assert a pulse of time_us
void assert_clk(uint8_t time_us) {
    gpio_init(CLK);
    gpio_set_dir(CLK, GPIO_OUT);
    gpio_put(CLK, 1);
    sleep_us(time_us);
    gpio_put(CLK, 0);
}

// Pull TRIG low for 15ms to fire the flash
void assert_trig() {
    gpio_put(TRIG, 0);
    sleep_ms(15);
    gpio_put(TRIG, 1);
}

// Take over control of the CLK GPIO and block until a 90us clock pulse is received
void wait_for_flash_ready() {
    gpio_init(CLK);
    gpio_set_dir(CLK, GPIO_IN);
    absolute_time_t waitstart = get_absolute_time();
    absolute_time_t waitend = waitstart;
    while((gpio_get(CLK) == 0) && (absolute_time_diff_us(waitend, waitstart) < TIMEOUT_PF_READY_US)) {
        waitend = get_absolute_time();
    }
}

// Start a simulation of a TTL metered flash
// pf_power: The power of the pre-flash metering strobe (MOSI byte 10)
// fe_power: The power of the final exposure flash, calculated through-the-lens based on the pre-flash
void simulate_ttl_flash(uint8_t pf_power, uint8_t ef_power) {
    // Send pre-flash metering initialization packet
    start_mosi_tx(body_packet_pf);

    state = STATE_METERING_PF;
    // Wait for the flash to send back a READY frame  - a 90us clock pulse asserted by the flash
    wait_for_flash_ready();

    // Signal the pre-flash to strobe - a 90us clock pulse asserted by the body
    assert_clk(MISO_INIT_US);

    // Spend ~50ms pretending to do TTL calculations
    sleep_ms(50);

    // Send an adjusted exposure flash initialization packet
    start_mosi_tx(body_packet_ef);

    // Wait for the flash to send back another READY frame
    wait_for_flash_ready();

    // Trigger the exposure flash by pulling TRIG low
    assert_trig();
}

// Configure the PIOs and DMA for a flash-to-body transfer
void start_miso_rx() {
    // Track what state we're in (not using this yet)
    state = STATE_RX_MISO;

    // Assert start pulse
    assert_clk(MISO_INIT_US);

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
    // Track what state we're in (not using this yet)
    state = STATE_TX_MOSI;
    
    // Assert start pulse
    assert_clk(MOSI_INIT_US);
    pio_gpio_init(mosi_pio, CLK);

    // Attach DATA pin function to TX PIO and set direction
    pio_gpio_init(mosi_pio, DATA);
    pio_sm_set_consecutive_pindirs(mosi_pio, mosi_sm, DATA, 2, true);

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
    // Initialize last read packet to a reasonable value
    for (int i = 0; i < flash_packet_length; i++) {
        old_flash_packet[i] = 0;
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
    gpio_set_dir(TRIG, GPIO_OUT);
    gpio_pull_up(TRIG);

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
        sleep_ms(15);
        start_mosi_tx(body_packet);
        sleep_ms(15);
        start_miso_rx();
    }
}
