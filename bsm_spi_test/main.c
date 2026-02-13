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
#define SPI_MAX_RETRIES 3

// Errors
#define SPI_ERR(x)                              (100+x)
#define SPI_UNKNOWN_ERR                         SPI_ERR(0)
#define SPI_READ_TOO_FAST_ERR                   SPI_ERR(1)
#define SPI_INTERNAL_OSC_ASLEEP_ERR             SPI_ERR(2)
#define SPI_CRC_MISMATCH_ERR                    SPI_ERR(3)
#define SPI_UNEXPECTED_RESPONSE_ERR             SPI_ERR(4)

#define DEBUG 1

/// @brief Initializes the SPI controller with predefined settings and configures the GPIO pins for SPI functionality
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

/// @brief Prints the contents of a byte array in hexadecimal format for debugging purposes
/// @param data The byte array to be printed
/// @param length The number of bytes in the array to be printed
void print_hex(uint8_t data[], size_t length) {
    // Prints the contents of a byte array in hexadecimal format without repeated 0x prefix
    // Example output: "0xABCDEF"
    printf("0x");
    for (size_t i = 0; i < length; i++) {
        printf("%02X", data[i]);
    }
    printf("\n");
}

/// @brief Calculate the CRC-8 checksum for a given byte array using the polynomial 0x07 (x^8 + x^2 + x + 1)
/// @param data The byte array for which the CRC is to be calculated
/// @param length The number of bytes in the array for which the CRC is to be calculated
/// @return The calculated CRC-8 value
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

/// @brief Verify the CRC of a received byte array by comparing the received CRC (last byte) with the calculated CRC of the preceding bytes
/// @param rx_buf The byte array containing the received data and CRC
/// @param length The number of bytes in the array
/// @return 1 if CRC matches, 0 otherwise
int verify_crc(uint8_t rx_buf[], size_t length) {
    if (length < 2) {
        return 0; // Not enough data to verify CRC
    }
    uint8_t received_crc = rx_buf[2];
    uint8_t calculated_crc = calculate_crc(rx_buf, 2); // Calculate CRC for the first 2 bytes
    return (received_crc == calculated_crc); // Return 1 if CRC matches, 0 otherwise
}

/// @brief Perform a blocking write-read operation over SPI, where data is sent from the tx_buf and response is read into the rx_buf. The function also handles asserting and deasserting the Chip Select (CS) line, includes a short delay to ensure proper timing, and verifies the CRC of the received data before returning the result.
/// @param tx_buf The byte array containing the data to be sent
/// @param rx_buf The byte array into which the response will be read
/// @param length The number of bytes to be transferred
/// @return 1 if CRC matches, 0 otherwise
int spi_wr_blocking(uint8_t tx_buf[], uint8_t rx_buf[], size_t length) {
    gpio_put(PIN_CS, 0); // Assert CS to start the transaction
    sleep_ms(SPI_CS_DELAY); // Short delay to ensure CS is recognized by the slave device
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, length); // Send data and read response
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
    if (DEBUG) {
        print_hex(rx_buf, length); // Print the received data in hexadecimal format for debugging
    }
    return verify_crc(rx_buf, length); // Verify CRC and return the result
}

int is_internal_osc_asleep(uint8_t read_buf[]) {
    if (read_buf[0] == 0xFF && read_buf[1] == 0xFF && read_buf[2] == 0xFF) {
        printf("Internal oscillator is asleep, received data: ");
        print_hex(read_buf, 3); // Debug print for the received data when internal oscillator is asleep
        return 1; // Internal oscillator is asleep
    } else {
        return 0; // Internal oscillator is active
    }
}

int is_outgoing_buffer_not_updated(uint8_t read_buf[]) {
    if (read_buf[0] == 0xFF && read_buf[1] == 0xFF && read_buf[2] == 0x00) {
        printf("Outgoing buffer has not been updated, received data: ");
        print_hex(read_buf, 3); // Debug print for the received data when outgoing buffer is not updated
        return 1; // Outgoing buffer has not been updated
    } else {
        return 0; // Outgoing buffer has been updated
    }
}

/// @brief Perform a blocking write operation over SPI, where data is sent from the tx_buf. The function handles asserting and deasserting the Chip Select (CS) line and includes a short delay to ensure proper timing. This function does not read any response from the slave device, so it is used for commands where no response is expected or when the response will be read in a separate transaction.
/// @param tx_buf The byte array containing the data to be sent
/// @param length The number of bytes to be transferred
void spi_w_blocking(uint8_t tx_buf[], size_t length) {
    gpio_put(PIN_CS, 0); // Assert CS to start the transaction
    sleep_ms(SPI_CS_DELAY); // Short delay to ensure CS is recognized by the slave device
    spi_write_blocking(SPI_PORT, tx_buf, length); // Send data
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
}

/// @brief Perform a blocking read operation over SPI, where data is read into the rx_buf. The function handles asserting and deasserting the Chip Select (CS) line, includes a short delay to ensure proper timing, and verifies the CRC of the received data before returning the result. This function is used for commands where a response is expected from the slave device after a separate write transaction.
/// @param rx_buf The byte array into which the response will be read
/// @param length The number of bytes to be transferred
/// @return 1 if CRC matches, 0 otherwise
int spi_r_blocking(uint8_t rx_buf[], size_t length) {
    gpio_put(PIN_CS, 0); // Assert CS to start the transaction
    sleep_ms(SPI_CS_DELAY); // Short delay to ensure CS is recognized by the slave device
    spi_read_blocking(SPI_PORT, 0x00, rx_buf, length); // Read data
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
    if (DEBUG) {
        print_hex(rx_buf, length); // Print the received data in hexadecimal format for debugging
    }
    return verify_crc(rx_buf, length); // Verify CRC and return the result
}

/* UNTESTED */
/// @brief Send a direct command to the stack monitor, save data byte to provided pointer, and return success or error code based on the response received. The function implements a retry mechanism for handling specific error conditions such as reading too early, internal oscillator being asleep, unexpected response, and CRC mismatch.
/// @param command The command byte to be sent to the stack monitor
/// @param data A pointer to a byte where the received data will be saved if the command is successful
/// @return 1 if the command is successful and data is saved, or an SPI error code (greater than 100) if the command fails after retries
int read_direct_command(uint8_t command, uint8_t* data) {
    // Steps:
    // 1. Send the command byte to the Stack Monitor
    // 2. Wait and read the response from the Stack Monitor.
    // 3. If response is 0xFFFF00, read too early, read again.
    // 4. If response is 0xFFFFFF, internal oscillator is asleep, wake up osc, read again.
    // 5. If return value byte 0 is not equal to command addr sent, read again..
    // 6. If CRC check fails, err.
    // 7. If all checks pass, save data byte (rx[1]) to data pointer and return success.
    size_t tries = 0;
    int err_code = SPI_UNKNOWN_ERR;
    uint8_t tx_buf[3] = {0x00, 0x00, 0x00}; // Dummy data to read response
    uint8_t rx_buf[3]; // Buffer to store received data
    tx_buf[0] = command; // Set the command byte to the command address
    tx_buf[2] = calculate_crc(tx_buf, 2); // Calculate CRC for the command
    spi_w_blocking(tx_buf, 3); // Send command without reading response (since we will read it in the next step)
    free(tx_buf); // Free the dynamically allocated tx_buf
    sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions
    while (tries < SPI_MAX_RETRIES) {
        int crc = spi_r_blocking(rx_buf, 3); // Read the response for the command
        
        // Checks
        if (crc == 0) { // Check if CRC check failed
            printf("CRC check failed, received data:");
            print_hex(rx_buf, 3); // Debug print for the received data when CRC check fails
            return 0; // CRC check failed, return error  
        } else if (is_outgoing_buffer_not_updated(rx_buf)) { // Check if the outgoing buffer has not been updated by checking if the response is 0xFFFF00
            printf("Read too early");
            err_code = SPI_READ_TOO_FAST_ERR;
            tries += 1;
            break;
        } else if (is_internal_osc_asleep(rx_buf)) { // Check if the internal oscillator is asleep by checking if all bytes are 0xFF
            printf("Internal oscillator is asleep, cannot read data");
            // wake_up_int_osc();
            err_code = SPI_INTERNAL_OSC_ASLEEP_ERR;
            tries += 1;
            break;
        } else if (rx_buf[0] != command) { // Check if response is for the command we sent
            printf("Received response for a different command than sent, expected addr: 0x%02X, received addr: 0x%02X\n", command, rx_buf[0]);
            err_code = SPI_UNEXPECTED_RESPONSE_ERR;
            tries += 1;
            break;
        } else { // All checks passed, save data and return success
            *data = rx_buf[1]; // Save the data byte to the provided pointer
            return 1; // Success
        }
    }
    return err_code;
}

/// @brief 
/// @param cell_address 
/// @return 
[[deprecated]] uint16_t read_cell_voltage(uint8_t cell_address) {
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

    /*********************************************************************************************/

    uint8_t command = 0x14;
    uint16_t voltage = 0;

    while (true) {
        
        uint8_t lsb, msb;

        int result = read_direct_command(command, &lsb);
        if (result == 1) {
            printf("Successfully read LSB: 0x%02X\n", lsb);
        } else {
            printf("Failed to read LSB, error code: %d\n", result);
        }
        int result = read_direct_command(command + 1, &msb);
        if (result == 1) {
            printf("Successfully read MSB: 0x%02X\n", msb);
        } else {
            printf("Failed to read MSB, error code: %d\n", result);
        }

        voltage = (msb << 8) | lsb; // Combine MSB and LSB to get the full voltage reading
        printf("Voltage reading for command 0x%02X: %d mV\n", command, voltage); // Print the voltage reading for the current command

        
        /*****************************************************************************************/
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

        /*****************************************************************************************/
        sleep_ms(5000);
    }
}
