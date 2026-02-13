// bq76972_spi.c
#include "bq76972_spi.h"
#include "spi_driver.h"
#include "pico/stdlib.h"
#include "config.h"
#include <stdio.h>

// Each SPI transaction is exactly 2 bytes (no CRC mode).
// Byte 0: [R/W | 7-bit address]  R=0, W=1
// Byte 1: data (write) or don't-care (read)
// The device returns the PREVIOUS transaction's echo on MISO.

static uint8_t last_rx[2]; // stores MISO from most recent transaction

static void bq76972_transaction(uint8_t b0, uint8_t b1) {
    uint8_t tx[2] = {b0, b1};
    gpio_put(SPI_CONTROLLER_PIN_CSB, 1); // Select device
    spi_controller_wr_blocking(tx, last_rx, 2);
    gpio_put(SPI_CONTROLLER_PIN_CSB, 0); // Deselect device
}

void bq76972_write_reg(uint8_t addr, uint8_t data) {
    // Byte 0: bit7=1 (write) | 7-bit address
    bq76972_transaction(0x80 | (addr & 0x7F), data);
    sleep_us(50);
}

uint8_t bq76972_read_reg(uint8_t addr) {
    // Transaction 1: request the read
    bq76972_transaction(addr & 0x7F, 0xFF);
    sleep_us(50);

    // Transaction 2: clock out the response (send dummy)
    bq76972_transaction(0xFF, 0xFF);
    sleep_us(50);

    // last_rx[1] now contains the data byte
    return last_rx[1];
}

bool bq76972_wake(void) {
    // Send dummy reads to wake the HFO oscillator
    for (int i = 0; i < 20; i++) {
        uint8_t val = bq76972_read_reg(0x00);
        if (val != 0xFF) {
            return true;
        }
        sleep_us(200);
    }
    return false;
}

bool bq76972_read_device_number(uint16_t *dev_num) {
    // 1. Write subcommand 0x0001 to addresses 0x3E (low) and 0x3F (high)
    bq76972_write_reg(0x3E, 0x01);  // low byte of subcommand
    bq76972_write_reg(0x3F, 0x00);  // high byte of subcommand

    // 2. Wait for device to process (~400 us for DEVICE_NUMBER)
    sleep_us(500);

    // 3. Poll 0x3E/0x3F until they echo back the subcommand
    for (int i = 0; i < 20; i++) {
        uint8_t lo = bq76972_read_reg(0x3E);
        uint8_t hi = bq76972_read_reg(0x3F);
        if (lo == 0x01 && hi == 0x00) {
            break;
        }
        if (i == 19) {
            printf("Subcommand did not complete\n");
            return false;
        }
        sleep_us(200);
    }

    // 4. Read 2-byte result from transfer buffer (0x40, 0x41)
    uint8_t b0 = bq76972_read_reg(0x40);
    uint8_t b1 = bq76972_read_reg(0x41);

    *dev_num = (uint16_t)b0 | ((uint16_t)b1 << 8);
    return true;
}