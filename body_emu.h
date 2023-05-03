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
#define MISO_INIT_US 80
#define MOSI_INIT_US 150
#define FLASH_READY_US 260

// Define machine states
#define STATE_IDLE 0
#define STATE_RX_MISO 1
#define STATE_TX_MOSI 2
#define STATE_METERING_PF 3
#define STATE_METERING_EF 4
#define STATE_RECHARGE 5

#define flash_packet_length 26
uint8_t old_flash_packet[flash_packet_length];
uint8_t new_flash_packet[flash_packet_length];

#define body_packet_length 5
const uint8_t body_packet[] = {
    0x12, 0x34, 0x56, 0x78, 0x9a
};

volatile absolute_time_t risetime;
volatile uint8_t state = STATE_IDLE;

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

void miso_dma_setup(PIO pio, uint sm, uint dma_chan);
void generate_clock_byte();
void generate_clock_multibyte(int count);
void start_miso_rx();
void start_mosi_tx();

// Functions used for generating testing a testing clock when TESTCLOCK is defined
void generate_miso_packet_clock();
void generate_mosi_packet_clock();
void generate_clock_byte();
void generate_clock_multibyte(int count);