#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_RX   0
#define PIN_CS   1
#define PIN_SCK  2
#define PIN_TX   3
#define SPI_BAUDRATE 1000*1000 // 1 MHz
#define BITS_PER_TRANSFER 8
#define SPI_CPOL 0
#define SPI_CPHA 0
#define SPI_ORDER 1 // MSB first
#define SPI_CS_DELAY 50
#define SPI_TRANSACTION_DELAY 100

#define DEBUG 1

void spi_controller_init() {
  // Initialize SPI with predefined settings from config.h
  spi_init(SPI_PORT, SPI_BAUDRATE);
  spi_set_format(
    SPI_PORT,
    BITS_PER_TRANSFER,
    SPI_CPOL,
    SPI_CPHA,
    SPI_ORDER
  );

  // Configure GPIO pins for SPI functionality
  gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
  gpio_pull_up(PIN_RX); // Enable pull-up on MISO/RX/SDI line
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_TX, GPIO_FUNC_SPI);
  
  // Configure Chip Select pin as output and set it high (inactive)
  gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);
}

void print_hex(uint8_t data[], size_t length) {
    // Prints the contents of a byte array in hexadecimal format without repeated 0x prefix
    // Example output: "0xABCDEF"
    printf("0x");
    for (size_t i = 0; i < length; i++) {
        printf("%02X", data[i]);
    }
    printf("\n");
}

void spi_wr_blocking(uint8_t tx_buf[], uint8_t rx_buf[], size_t length) {
    gpio_put(PIN_CS, 0); // Assert CS to start the transaction
    sleep_ms(SPI_CS_DELAY); // Short delay to ensure CS is recognized by the slave device
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, length); // Send data and read response
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
    if (DEBUG) {
        print_hex(rx_buf, length); // Print the received data in hexadecimal format for debugging
    }
}

void spi_w_blocking(uint8_t tx_buf[], size_t length) {
    gpio_put(PIN_CS, 0); // Assert CS to start the transaction
    sleep_ms(SPI_CS_DELAY); // Short delay to ensure CS is recognized by the slave device
    spi_write_blocking(SPI_PORT, tx_buf, length); // Send data
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
}

void spi_r_blocking(uint8_t rx_buf[], size_t length) {
    gpio_put(PIN_CS, 0); // Assert CS to start the transaction
    sleep_ms(SPI_CS_DELAY); // Short delay to ensure CS is recognized by the slave device
    spi_read_blocking(SPI_PORT, 0x00, rx_buf, length); // Read data
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
    if (DEBUG) {
        print_hex(rx_buf, length); // Print the received data in hexadecimal format for debugging
    }
}

uint8_t calculate_crc(uint8_t data[], size_t length) {
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i]; // XOR byte into CRC
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; // Shift left and XOR with polynomial
            } else {
                crc <<= 1; // Just shift left
            }
        }
    }
    return crc;
}

uint16_t convert_to_voltage(uint8_t high_byte, uint8_t low_byte) {
    // Combine the two bytes into a single 16-bit value little-endian format
    uint16_t raw_value = (low_byte << 8) | high_byte; // Combine high and low bytes
    return raw_value; // Return the raw value as voltage (assuming 1 LSB = 1 mV)
}

uint16_t read_cell_voltage(uint8_t cell_address) {
    // Steps:
    // 1. Read command from the address provided
    // 2. Save the 2nd byte as the LSB of the voltage reading
    // 3. Read the next byte and save it as the MSB of the voltage reading
    // 4. Combine the MSB and LSB to get the full voltage reading
    // 5. Return the voltage reading as a 16-bit unsigned integer
    uint16_t voltage = 0;

    // 1. Read command from the address provided
    // Build the SPI command to read the cell voltage
    uint8_t lsb_tx_buf[3] = {0x00, 0x00, 0x00}; // Dummy data to read LSB
    uint8_t msb_tx_buf[3] = {0x00, 0x00, 0x00}; // Dummy data to read MSB
    uint8_t rx_buf[3]; // Buffer to store received data
    lsb_tx_buf[0] = cell_address; // Set the command byte to the cell address
    msb_tx_buf[0] = cell_address + 1; // Set the command byte to read the MSB (next address)
    lsb_tx_buf[2] = calculate_crc(lsb_tx_buf, 2); // Calculate CRC for the LSB command
    msb_tx_buf[2] = calculate_crc(msb_tx_buf, 2); // Calculate CRC for the MSB command

    // 2. Save the 2nd byte as the LSB of the voltage reading
    // spi_wr_blocking(lsb_tx_buf, rx_buf, 3); // Send command to read LSB and receive response
    spi_w_blocking(lsb_tx_buf, 3); // Send command to read LSB without reading response (since we will read it in the next step)
    print_hex(lsb_tx_buf, 3); // Debug print for LSB command being sent
    sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions
    spi_r_blocking(rx_buf, 3); // Read the response for the LSB
    uint8_t lsb = rx_buf[1]; // Save the 2nd byte as LSB
    // 3. Read the next byte and save it as the MSB of the voltage reading
    // spi_wr_blocking(msb_tx_buf, rx_buf, 3); // Send command to read MSB and receive response
    spi_w_blocking(msb_tx_buf, 3); // Send command to read MSB without reading response (since we will read it in the next step)
    print_hex(msb_tx_buf, 3); // Debug print for MSB command being sent
    sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions
    spi_r_blocking(rx_buf, 3); // Read the response for the MSB
    uint8_t msb = rx_buf[1]; // Save the 2nd byte as MSB
    // 4. Combine the MSB and LSB to get the full voltage reading
    voltage = (msb << 8) | lsb; // Combine MSB and LSB to get the full voltage reading
    // 5. Return the voltage reading as a 16-bit unsigned integer
    // Convert from millivolts to volts
    return voltage; // Return the voltage reading
}

int main()
{
    stdio_init_all();
    spi_controller_init();
    sleep_ms(8000); // Sleep for a bit to allow time to connect a logic analyser or oscilloscope


    size_t num_bytes = 3; // Number of bytes to send
    uint8_t tx_buf[] = {0x00, 0x00, 0x00}; // Data to send
    uint8_t rx_buf[num_bytes]; // Buffer to store received data

    spi_w_blocking(tx_buf, num_bytes); // Send dummy data without reading response
    sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions

    size_t cells = 6; // Number of battery cells to read from the BMS
    uint8_t cell_command = 0x14;

    while (true) {
        // printf("Reading cell voltages...\n");
        // for (size_t i = 0; i < cells; i++) {

        //     printf("DEBUG: Reading voltage for Cell %d at address 0x%02X\n", i+1, cell_command + (i*2)); // Debug print for cell address being read
        //     uint16_t voltage = read_cell_voltage(cell_command + (i*2)); // Read voltage for the current cell
        //     printf("Cell %d Voltage: %d mV\n", i+1, voltage); // Print the voltage reading for the current cell

        //     sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions
        // }
        // printf("\n\n\n"); // Print a newline after reading all cells

        uint8_t rx_buf[3]; // Buffer to store received data
        uint8_t tx_buf[3] = {0x00, 0x00, 0x00}; // Dummy data to read LSB
        tx_buf[0] = 0x00;
        tx_buf[2] = calculate_crc(tx_buf, 2); // Calculate CRC for the command
        spi_w_blocking(tx_buf, 3);
        printf("tx_buf after sending LSB command: ");
        print_hex(tx_buf, 3); // Debug print for command being sent
        sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions
        spi_r_blocking(rx_buf, 3); // Read the response for the LSB
        uint8_t lsb = rx_buf[1]; // Save the 2nd byte as LSB
        print_hex(rx_buf, 3); // Debug print for the response received
        tx_buf[0] = 0x01;
        tx_buf[2] = calculate_crc(tx_buf, 2); // Calculate CRC for the command
        spi_w_blocking(tx_buf, 3);
        printf("tx_buf after sending MSB command: ");
        print_hex(tx_buf, 3); // Debug print for command being sent
        sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions
        spi_r_blocking(rx_buf, 3); // Read the response for the MSB
        uint8_t msb = rx_buf[1]; // Save the 2nd byte as MSB
        print_hex(rx_buf, 3); // Debug print for the response received

        uint16_t control_status = (msb << 8) | lsb; // Combine MSB and LSB to get the full voltage reading
        // Print the second bit of the control status to check if the BMS is in DEEPSLEEP mode (1 = DEEPSLEEP, 0 = ACTIVE)
        printf("BMS Control Status: 0x%04X, DEEPSLEEP Mode: %s\n", control_status, (control_status & 0x0002) ? "DEEPSLEEP" : "ACTIVE");

        sleep_ms(5000);
    }
}
