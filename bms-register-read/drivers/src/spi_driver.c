#include "spi_driver.h"

/* Local Includes */
#include "pico/stdlib.h"
#include "config.h"

void spi_controller_init() {
  // Initialize SPI with predefined settings from config.h
  spi_init(SPI_CONTROLLER_PORT, SPI_CONTROLLER_BAUDRATE);
  spi_set_format(
    SPI_CONTROLLER_PORT,
    SPI_CONTROLLER_BITS_PER_WORD,
    SPI_CONTROLLER_CPOL,
    SPI_CONTROLLER_CPHA,
    SPI_CONTROLLER_ORDER
  );

  // Configure GPIO pins for SPI functionality
  gpio_set_function(SPI_CONTROLLER_PIN_SDI, GPIO_FUNC_SPI);
  gpio_pull_up(SPI_CONTROLLER_PIN_SDI); // Enable pull-up on MISO/RX/SDI line
  gpio_set_function(SPI_CONTROLLER_PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(SPI_CONTROLLER_PIN_SDO, GPIO_FUNC_SPI);
  
  // Configure Chip Select pin as output and set it high (inactive)
  gpio_set_function(SPI_CONTROLLER_PIN_CSB, GPIO_FUNC_SIO);
  gpio_set_dir(SPI_CONTROLLER_PIN_CSB, GPIO_OUT);
  gpio_put(SPI_CONTROLLER_PIN_CSB, 0); // Deselect device (active low)
}

void spi_controller_deinit() {
  // Deinitialize SPI (if needed, additional cleanup can be added here)
  spi_deinit(SPI_CONTROLLER_PORT);
}

int spi_controller_wr_blocking(uint8_t* tx_buffer, uint8_t* rx_buffer, size_t length) {
  /* Steps:
   * 1. Set CSB low to select the device
   * 2. Use spi_write_read_blocking to perform the transfer
   * 3. Check if the transfer was successful
   * 4. Set CSB high to deselect the device
   * 5. Return appropriate status code
  **/

  // Set CSB low to select the device
  gpio_put(SPI_CONTROLLER_PIN_CSB, 0);
  // Perform the SPI transfer
  if (spi_write_read_blocking(SPI_CONTROLLER_PORT, tx_buffer, rx_buffer, length) != length) {
    gpio_put(SPI_CONTROLLER_PIN_CSB, 1); // Deselect device on error
    return 1; // Error during transfer, transfer length mismatch
  }

  // Set CSB high to deselect the device
  gpio_put(SPI_CONTROLLER_PIN_CSB, 1);
  return 0; // Success
}

int spi_controller_read_blocking(uint8_t* rx_buffer, size_t length) {
  /** Steps:
   *  1. Set CSB low to select the device
   *  2. Use spi_read_blocking to read data
   *  3. Check if the read was successful
   *  4. Set CSB high to deselect the device
   *  5. Return appropriate status code
  **/

  // Set CSB low to select the device
  gpio_put(SPI_CONTROLLER_PIN_CSB, 0);
  // Perform the SPI read
  if (spi_read_blocking(SPI_CONTROLLER_PORT, 0x00, rx_buffer, length) != length) {
    gpio_put(SPI_CONTROLLER_PIN_CSB, 1); // Deselect device on error
    return 1; // Error during read, read length mismatch
  }
  // Set CSB high to deselect the device
  gpio_put(SPI_CONTROLLER_PIN_CSB, 1);
  return 0; // Success
}

int spi_controller_write_blocking(uint8_t* tx_buffer, size_t length) {
  /** Steps:
   *  1. Set CSB low to select the device
   *  2. Use spi_write_blocking to write data
   *  3. Check if the write was successful
   *  4. Set CSB high to deselect the device
   *  5. Return appropriate status code
  **/

  // Set CSB low to select the device
  gpio_put(SPI_CONTROLLER_PIN_CSB, 0);
  // Perform the SPI write
  if (spi_write_blocking(SPI_CONTROLLER_PORT, tx_buffer, length) != length) {
    gpio_put(SPI_CONTROLLER_PIN_CSB, 1); // Deselect device on error
    return 1; // Error during write, write length mismatch
  }
  // Set CSB high to deselect the device
  gpio_put(SPI_CONTROLLER_PIN_CSB, 1);
  return 0; // Success
}