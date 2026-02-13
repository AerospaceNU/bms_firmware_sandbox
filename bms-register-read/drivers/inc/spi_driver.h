#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

/* Global Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "hardware/spi.h"

//* SPI Driver Functions */

/**
 * @brief Initialize the SPI controller with predefined settings.
 */
void spi_controller_init();
/**
 * @brief Deinitialize the SPI controller.
 */
void spi_controller_deinit();
/**
 * @brief Perform a blocking SPI write-read operation.
 * @param tx_buffer Pointer to the transmit buffer.
 * @param rx_buffer Pointer to the receive buffer.
 * @param length Number of bytes to transfer.
 * @return 0 on success, non-zero error code on failure.
 */
int spi_controller_wr_blocking(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t length);
/**
 * @brief Perform a blocking SPI read operation.
 * @param rx_buffer Pointer to the receive buffer.
 * @param length Number of bytes to read.
 * @return 0 on success, non-zero error code on failure.
 */
int spi_controller_read_blocking(uint8_t* rx_buffer, size_t length);
/**
 * @brief Perform a blocking SPI write operation.
 * @param tx_buffer Pointer to the transmit buffer.
 * @param length Number of bytes to write.
 * @return 0 on success, non-zero error code on failure.
 */
int spi_controller_write_blocking(uint8_t* tx_buffer, size_t length);


#endif // SPI_DRIVER_H