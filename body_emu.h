/**
 * 2023/04 Andrew Villeneuve
 * Emulate a Sony camera body to a multi-interface shoe flash 
 * 
 * Pins: 
 * DATA on GPIO 0, 
 * CLOCK on GPIO 1,
 * TRIG on GPI 2
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"

// Define GPIOs
// Data must be a lower pin number than clock for the PIOs to map correctly
#define DATA 2
#define CLK 3
#define TRIG 4
#define F1 4

// Define timing constants
#define CLOCK_US 6
#define MISO_INIT_US 90
#define MOSI_INIT_US 165
#define FLASH_READY_US 260
// Time between the start of one packet and the start of the next
#define PACKET_INTERVAL_MS 16

// Define machine states
#define STATE_STANDBY 0
#define STATE_READY 2
#define STATE_METERING_PF 3
#define STATE_METERING_EF 4
#define STATE_RECHARGE 5
#define STATE_READY_TIMEOUT 6

// Define timeouts

// Maximum time between end of PF init packet from body and PF ready signal from flash
#define TIMEOUT_PF_READY_MS 10

// Maximum time between end of EF exposure correction packet from body and EF ready signal from flash
#define TIMEOUT_EF_READY_MS 10

#define flash_packet_length 26
uint8_t old_flash_packet[flash_packet_length];
uint8_t new_flash_packet[flash_packet_length];
bool flash_packet_updated = false;

// This must also be seperately defined as a constant in tx-mosi.pio
#define body_packet_length 14

bool body_packet_updated = false;

uint8_t body_packet[body_packet_length];

// State to report to flash when idle
uint8_t body_packet_standby[] = {
    0x28,0x9C,0x2B,0xFF,0xAE,0x7D,0x80,0x40,0x6E,0x00,0xE3,0x75,0x80,0x00
};

// State that represents shutter button half-press
uint8_t body_packet_ready[] = {
    0x28,0x9C,0x2B,0xFF,0xAE,0x7D,0x80,0x40,0x6A,0x00,0xE3,0x75,0x80,0x00
};

// Pre-flash initialization packet
uint8_t body_packet_pf[] = {
    0x28,0x9C,0x2B,0xFF,0xAE,0x7D,0x80,0x40,0x4A,0xFC,0xC0,0x75,0xF8,0x78
};

// Exposure flash correction packet
uint8_t body_packet_ef[] = {
    0x28,0x9C,0x2B,0xFF,0xAE,0x7D,0x80,0xC0,0x7A,0xDF,0xC0,0x75,0xBF,0x3F 
};

volatile absolute_time_t risetime;
volatile uint8_t state = STATE_STANDBY;

#define uart_cmd_buffer_length 32
uint8_t uart_cmd_buffer_pos = 0;
uint8_t uart_cmd_buffer[uart_cmd_buffer_length];

// DMA and PIO variables
const PIO miso_pio = pio0;
const uint8_t miso_sm = 0;
uint8_t miso_dma_chan;
uint8_t miso_offset;

const PIO mosi_pio = pio1;
const uint8_t mosi_irq = PIO1_IRQ_0;
const uint8_t mosi_sm = 0;
uint8_t mosi_dma_chan;
uint8_t mosi_offset;

void process_uart_cmd();
void miso_dma_setup(PIO pio, uint sm, uint dma_chan);
void start_miso_rx();
void start_mosi_tx(const uint8_t *data);
