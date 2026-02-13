// main.c
#include <stdio.h>
#include "pico/stdlib.h"
#include "spi_driver.h"
#include "bq76972_spi.h"
#include "config.h"
// static uint8_t crc8(const uint8_t *data, size_t len) {
//     uint8_t crc = 0x00;
//     for (size_t i = 0; i < len; i++) {
//         crc ^= data[i];
//         for (int j = 0; j < 8; j++) {
//             if (crc & 0x80)
//                 crc = (crc << 1) ^ 0x07;
//             else
//                 crc <<= 1;
//         }
//     }
//     return crc;
// }

// void bq_write_reg(uint8_t addr, uint8_t data) {
//     uint8_t tx[3], rx[3];
//     tx[0] = 0x80 | (addr & 0x7F);  // write bit set
//     tx[1] = data;
//     tx[2] = crc8(tx, 2);
//     spi_controller_wr_blocking(tx, rx, 3);
//     sleep_us(100);
// }

// uint8_t bq_read_reg(uint8_t addr) {
//     uint8_t tx[3], rx[3];

//     // Transaction 1: request read
//     tx[0] = addr & 0x7F;  // read bit = 0
//     tx[1] = 0xFF;
//     tx[2] = crc8(tx, 2);
//     spi_controller_wr_blocking(tx, rx, 3);
//     sleep_us(100);

//     // Transaction 2: clock out response
//     tx[0] = 0xFF;
//     tx[1] = 0xFF;
//     tx[2] = 0xFF;
//     spi_controller_wr_blocking(tx, rx, 3);
//     sleep_us(100);

//     // Verify CRC on response
//     uint8_t expected_crc = crc8(rx, 2);
//     if (rx[2] != expected_crc) {
//         printf("CRC mismatch: got %02X, expected %02X\n", rx[2], expected_crc);
//     }

//     return rx[1];  // data byte
// }

void bq_write_reg(uint8_t addr, uint8_t data) {
    uint8_t tx[2], rx[2];
    tx[0] = 0x80 | (addr & 0x7F);
    tx[1] = data;
    gpio_put(SPI_CONTROLLER_PIN_CSB, 0); // Select device
    sleep_us(50);
    spi_controller_wr_blocking(tx, rx, 2);
    gpio_put(SPI_CONTROLLER_PIN_CSB, 1); // Deselect device
    printf("  W [%02X]=%02X  (rx: %02X %02X)\n", addr, data, rx[0], rx[1]);
    sleep_us(100);
}

uint8_t bq_read_reg(uint8_t addr) {
    uint8_t tx[2], rx[2];

    // Transaction 1: request
    tx[0] = addr & 0x7F;
    tx[1] = 0xFF;
    gpio_put(SPI_CONTROLLER_PIN_CSB, 0); // Select device
    spi_controller_wr_blocking(tx, rx, 2);
    gpio_put(SPI_CONTROLLER_PIN_CSB, 1); // Deselect device
    printf("  R1[%02X]     (rx: %02X %02X)\n", addr, rx[0], rx[1]);
    sleep_us(100);

    // Transaction 2: get response
    tx[0] = 0xFF;
    tx[1] = 0xFF;
    gpio_put(SPI_CONTROLLER_PIN_CSB, 0); // Select device
    spi_controller_wr_blocking(tx, rx, 2);
    gpio_put(SPI_CONTROLLER_PIN_CSB, 1); // Deselect device
    printf("  R2[%02X]     (rx: %02X %02X)\n", addr, rx[0], rx[1]);
    sleep_us(100);

    return rx[1];
}


int main() {
    stdio_init_all();
    sleep_ms(10000);  // wait for USB serial

    
    spi_controller_init();

    
    // In main():
    printf("=== Wake ===\n");
    for (int i = 0; i < 5; i++) {
        bq_read_reg(0x00);
        sleep_ms(5);
    }

    printf("\n=== Write subcommand 0x0001 ===\n");
    bq_write_reg(0x3E, 0x01);
    bq_write_reg(0x3F, 0x00);

    sleep_ms(2);

    printf("\n=== Poll 0x3E/0x3F ===\n");
    for (int i = 0; i < 10; i++) {
        uint8_t lo = bq_read_reg(0x3E);
        uint8_t hi = bq_read_reg(0x3F);
        printf("  -> lo=%02X hi=%02X\n", lo, hi);
        if (lo == 0x01 && hi == 0x00) {
            printf("  Subcommand complete!\n");
            break;
        }
        sleep_ms(2);
    }

    printf("\n=== Read buffer ===\n");
    uint8_t b0 = bq_read_reg(0x40);
    uint8_t b1 = bq_read_reg(0x41);
    printf("Device Number: 0x%04X\n", (uint16_t)b0 | ((uint16_t)b1 << 8));

    while (true) {
        sleep_ms(1000);
    }

    // // In main():
    // printf("=== Reading Device Number ===\n");

    // // Wake device
    // for (int i = 0; i < 5; i++) {
    //     bq_read_reg(0x00);
    // }

    // // Write subcommand 0x0001 to 0x3E/0x3F
    // printf("Writing subcommand...\n");
    // bq_write_reg(0x3E, 0x01);  // low byte
    // bq_write_reg(0x3F, 0x00);  // high byte

    // sleep_ms(1);  // wait for subcommand to process

    // // Poll until subcommand completes
    // printf("Polling for completion...\n");
    // for (int i = 0; i < 20; i++) {
    //     uint8_t lo = bq_read_reg(0x3E);
    //     uint8_t hi = bq_read_reg(0x3F);
    //     printf("  Poll %d: 0x3E=%02X 0x3F=%02X\n", i, lo, hi);
    //     if (lo == 0x01 && hi == 0x00) {
    //         printf("  Subcommand complete!\n");
    //         break;
    //     }
    //     sleep_ms(1);
    // }

    // // Read result from transfer buffer
    // uint8_t b0 = bq_read_reg(0x40);
    // uint8_t b1 = bq_read_reg(0x41);
    // uint16_t device_number = (uint16_t)b0 | ((uint16_t)b1 << 8);
    // printf("Device Number: 0x%04X\n", device_number);

}