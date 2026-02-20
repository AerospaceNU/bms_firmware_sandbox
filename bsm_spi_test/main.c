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
#define SPI_TRANSACTION_DELAY_US 50
#define SPI_MAX_RETRIES 10
#define WAKEUP_DELAY_MS 5
#define SUBCOMMAND_LOWER 0x3E
#define SUBCOMMAND_UPPER 0x3F

typedef enum {
    CELL_1 = 0x14,
    CELL_2 = 0x16,
    CELL_3 = 0x18,
    CELL_4 = 0x1A,
    CELL_5 = 0x1C,
    CELL_6 = 0x1E,
    CELL_7 = 0x20,
    CELL_8 = 0x22,
    CELL_9 = 0x24,
    CELL_10 = 0x26,
    CELL_11 = 0x28,
    CELL_12 = 0x2A,
    CELL_13 = 0x2C,
    CELL_14 = 0x2E,
    CELL_15 = 0x30,
    CELL_16 = 0x32,
    LAST_CELL = CELL_16,
    STACK_VOLTAGE = 0x34
} cell_voltage_t;

typedef enum {
    INT_THERMISTOR = 0x68,
    CFETOFF_THERMISTOR = 0x6A,
    DFETOFF_THERMISTOR = 0x6C,
    ALERT_THERMISTOR = 0x6E,
    TS1_THERMISTOR = 0x70,
    TS2_THERMISTOR = 0x72,
    TS3_THERMISTOR = 0x74,
    HDQ_THERMISTOR = 0x76,
    DCHG_THERMISTOR = 0x78,
    DDSG_THERMISTOR = 0x7A,
} thermistor_t;

// Errors
#define SUCCESS                                 1
#define SPI_ERR(x)                              (100+x)
#define SPI_UNKNOWN_ERR                         SPI_ERR(0)
#define SPI_READ_TOO_FAST_ERR                   SPI_ERR(1)
#define SPI_INTERNAL_OSC_ASLEEP_ERR             SPI_ERR(2)
#define SPI_CRC_MISMATCH_ERR                    SPI_ERR(3)
#define SPI_UNEXPECTED_RESPONSE_ERR             SPI_ERR(4)

#define DEBUG 0 // Set to 1 to enable debug prints, 0 to disable

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
    if (rx_buf[0] == 0xFF && rx_buf[1] == 0xFF && (rx_buf[2] == 0xFF || rx_buf[2] == 0x00)) {
        return 1; // Special case for 0xFFFF00 or 0xFFFFFF responses, consider CRC valid
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
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, length); // Send data and read response
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
    if (DEBUG) {
        print_hex(rx_buf, length); // Print the received data in hexadecimal format for debugging
    }
    return verify_crc(rx_buf, length); // Verify CRC and return the result
}

int is_internal_osc_asleep(uint8_t read_buf[]) {
    if (read_buf[0] == 0xFF && read_buf[1] == 0xFF && read_buf[2] == 0xFF) {
        if (DEBUG) {
            printf("Internal oscillator is asleep, received data: ");
            print_hex(read_buf, 3); // Debug print for the received data when internal oscillator is asleep
        }
        return 1; // Internal oscillator is asleep
    } else {
        return 0; // Internal oscillator is active
    }
}

int is_outgoing_buffer_not_updated(uint8_t read_buf[]) {
    if (read_buf[0] == 0xFF && read_buf[1] == 0xFF && read_buf[2] == 0x00) {
        if (DEBUG) {
            printf("Outgoing buffer has not been updated, received data: ");
            print_hex(read_buf, 3); // Debug print for the received data when outgoing buffer is not updated
        }
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
    spi_write_blocking(SPI_PORT, tx_buf, length); // Send data
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
}

/// @brief Perform a blocking read operation over SPI, where data is read into the rx_buf. The function handles asserting and deasserting the Chip Select (CS) line, includes a short delay to ensure proper timing, and verifies the CRC of the received data before returning the result. This function is used for commands where a response is expected from the slave device after a separate write transaction.
/// @param rx_buf The byte array into which the response will be read
/// @param length The number of bytes to be transferred
/// @return 1 if CRC matches, 0 otherwise
int spi_r_blocking(uint8_t rx_buf[], size_t length) {
    gpio_put(PIN_CS, 0); // Assert CS to start the transaction
    spi_read_blocking(SPI_PORT, 0x00, rx_buf, length); // Read data
    gpio_put(PIN_CS, 1); // Deassert CS to end the transaction
    if (DEBUG) {
        print_hex(rx_buf, length); // Print the received data in hexadecimal format for debugging
    }
    return verify_crc(rx_buf, length); // Verify CRC and return the result
}

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
    // 6. If CRC check fails, read again.
    // 7. If all checks pass, save data byte (rx[1]) to data pointer and return success.
    size_t tries = 0;
    int err_code = SPI_UNKNOWN_ERR;
    uint8_t tx_buf[3] = {0x00, 0x00, 0x00}; // Dummy data to read response
    uint8_t rx_buf[3]; // Buffer to store received data
    tx_buf[0] = command; // Set the command byte to the command address
    tx_buf[2] = calculate_crc(tx_buf, 2); // Calculate CRC for the command
    while (tries < SPI_MAX_RETRIES) {
        spi_w_blocking(tx_buf, 3); // Send command without reading response (since we will read it in the next step)
        sleep_us(SPI_TRANSACTION_DELAY_US); // Short delay between transmissions
        int crc = spi_r_blocking(rx_buf, 3); // Read the response for the command
        
        // Checks
        if (crc == 0) { // Check if CRC check failed
            if (DEBUG) {
                printf("CRC check failed, received data:");
                print_hex(rx_buf, 3); // Debug print for the received data when CRC check fails
            }
            err_code = SPI_CRC_MISMATCH_ERR;
            tries += 1;
            break;
        } else if (is_outgoing_buffer_not_updated(rx_buf)) { // Check if the outgoing buffer has not been updated by checking if the response is 0xFFFF00
            if (DEBUG) {
                printf("Read too early");
            }
            err_code = SPI_READ_TOO_FAST_ERR;
            tries += 1;
            break;
        } else if (is_internal_osc_asleep(rx_buf)) { // Check if the internal oscillator is asleep by checking if all bytes are 0xFF
            if (DEBUG) {
                printf("Internal oscillator is asleep, cannot read data");
            }
            // wake_up_int_osc();
            err_code = SPI_INTERNAL_OSC_ASLEEP_ERR;
            tries += 1;
            break;
        } else if (rx_buf[0] != command) { // Check if response is for the command we sent
            if (DEBUG) {
                printf("Received response for a different command than sent, expected addr: 0x%02X, received addr: 0x%02X\n", command, rx_buf[0]);
            }
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

int write_direct_command(uint8_t address, uint8_t data) {
    // Steps:
    // 1. Send the command byte, data byte, and CRC byte to the Stack Monitor
    // 2. Wait for at least 50 us, then read back transmission.
    // 3. If response if 0xFFFF00, read too fast, read again.
    // 4. If response is 0xFFFFFF, internal oscillator is off, wake osc up and retry write transaction.
    // 5. If response is 0xFFFFAA, write CRC was wrong, retry write transaction
    // 6. If return address byte is different from written address, read again.
    // 7. If CRC check fails, read again
    // 8. If all checks pass, return success
    
}

int write_dc_helper(uint8_t address, uint8_t data, bool write_again, size_t tries) {

}

/// @brief Read a 16-bit value from the stack monitor by sending two commands to read the LSB and MSB bytes.
/// @details 16-bit values are stored in little-endian format, so the LSB is read first using the provided command address, and then the MSB is read using the command address + 1. The function combines the received LSB and MSB bytes into a single 16-bit value and saves it to the provided pointer. The function implements a retry mechanism to handle potential errors during the SPI transactions, such as reading too early, internal oscillator being asleep, unexpected response, and CRC mismatch. If the read operation is successful, it returns 1; otherwise, it returns an appropriate SPI error code after exhausting retries.
/// @param lsb_command The command byte corresponding to the LSB of the value to be read. The MSB is expected to be at the next command address (lsb_command + 1). For example, if reading a cell voltage, the LSB command might be CELL_1 (0x14) and the MSB command would be CELL_1 + 1 (0x15).
/// @param value A pointer to a uint16_t variable where the combined 16-bit value will be stored if the read operation is successful. The value is expected to be in little-endian format, with the LSB read from the lsb_command address and the MSB read from the lsb_command + 1 address. For example, if reading a cell voltage, the resulting value would represent the voltage in millivolts (mV) after combining the LSB and MSB bytes.
/// @return 1 if the read operation is successful and the value is stored, or an SPI error code (greater than 100) if the read operation fails after retries
int read_i2(uint8_t lsb_command, uint16_t* value) {
    uint8_t lsb, msb;
    int result = SPI_UNKNOWN_ERR;
    for (int i = 0; i < SPI_MAX_RETRIES; i++) {
        result = read_direct_command(lsb_command, &lsb);
        
        if (result) {
            if (DEBUG) {
                printf("Successfully read LSB for command 0x%02X: 0x%02X\n", lsb_command, lsb);
            }
            result = read_direct_command(lsb_command + 1, &msb);
            if (result) {
                if (DEBUG) {
                    printf("Successfully read MSB for command 0x%02X: 0x%02X\n", lsb_command + 1, msb);
                }
                *value = (msb << 8) | lsb; // Combine MSB and LSB to get the full value
                return 1; // Success
            } else {
                if (DEBUG) {
                    printf("Failed to read MSB for command 0x%02X, error code: %d\n", lsb_command + 1, SPI_UNKNOWN_ERR);
                }
            }
        } else {
            if (DEBUG) {
                printf("Failed to read LSB for command 0x%02X, error code: %d\n", lsb_command, SPI_UNKNOWN_ERR);
            }
        }
        if (DEBUG) {
            printf("Retrying read_i2 for command 0x%02X, attempt %d\n", lsb_command, i+1);
        }
    }
    if (DEBUG) {
        printf("Failed to read value for command 0x%02X after %d attempts, last error code: %d\n", lsb_command, SPI_MAX_RETRIES, result);
    }
    return result; // Failed after retries
}

void wakeup() {
    uint8_t tx_buf[3] = {0x00, 0x00, 0x00};
    tx_buf[2] = calculate_crc(tx_buf, 2); // Calculate CRC for the dummy command
    spi_w_blocking(tx_buf, 3); // Send dummy command to wake up the internal oscillator
    sleep_us(WAKEUP_DELAY_MS * 1000); // Wait for the internal oscillator to wake up
    spi_r_blocking(tx_buf, 3); // Read the response to clear the wake-up command from the buffer
    print_hex(tx_buf, 3); // Debug print for the response received after wake-up command
}

/// @brief Read a voltage value from the stack monitor by sending the appropriate command and combining the received LSB and MSB bytes into a single 16-bit voltage value. The function uses the read_i2 helper function to perform the necessary SPI transactions and handle retries and error conditions.
/// @param command The command byte corresponding to the voltage reading to be read (e.g., CELL_1, CELL_2, etc.)
/// @param voltage A pointer to a uint16_t variable where the read voltage value will be stored if the read operation is successful. The voltage value is expected to be in millivolts (mV) and is obtained by combining the LSB and MSB bytes read from the stack monitor.
/// @return 1 if the read operation is successful, 0 otherwise.
int read_voltage(cell_voltage_t command, uint16_t* voltage) {
    return read_i2(command, voltage);
}

int read_temperature(thermistor_t command, uint16_t* temperature) {
    return read_i2(command, temperature);
}

int read_all_voltages(uint16_t* voltages, size_t num_cells) {
    for (size_t i = 0; i < num_cells; i++) {
        cell_voltage_t cell_command = CELL_1 + i; // Assuming cell voltage commands are sequential
        if (read_voltage(cell_command, &voltages[i]) != 1) {
            if (DEBUG) {
                printf("Failed to read voltage for Cell %d at address 0x%02X\n", i+1, cell_command);
            }
            return 0; // Failed to read voltage for this cell
        }
        if (DEBUG) {
            printf("Successfully read voltage for Cell %d: %d mV\n", i+1, voltages[i]);
        }
    }
    return 1; // Success
}

int read_all_temperatures(thermistor_t* commands, int16_t* temperatures, size_t num_thermistors) {
    for (size_t i = 0; i < num_thermistors; i++) {
        if (read_temperature(commands[i], &temperatures[i]) != 1) {
            if (DEBUG) {
                printf("Failed to read temperature for Thermistor %d at address 0x%02X\n", i+1, commands[i]);
            }
            return 0; // Failed to read temperature for this thermistor
        }
        if (DEBUG) {
            printf("Successfully read temperature for Thermistor %d: %d (0.1K)\n", i+1, temperatures[i]);
        }
    }
    return 1; // Success
}

int read_subcommand(uint16_t addr, uint8_t data_buf[], size_t* data_len) {
    // Steps:
    // 1. Split addr into lower and upper bytes
    // 2. Write lower byte to 0x3E
    // 3. Write upper byte to 0x3F
    // 4. Continuously read 0x3E and 0x3F back:
    //    - If reads 0xFF, operation has not completed
    //    - If reads back what was originally written, operation has completed
    // 5. Read length of response from 0x61
    // 6. Read response into buffer from 0x40 for expected length
    // 7. Read checksum at 0x60 and verify it matches data read

    // 1. Split addr into lower and upper bytes
    uint8_t lower_addr = (addr & 0xFF00) >> 8;
    uint8_t upper_addr = (addr & 0x00FF);

    int result = write_sc_addrs(lower_addr, upper_addr);
    if (result != SUCCESS) {

    }
}

/**
 * @brief Writes 16-bit Subcommand Address to 0x3E and 0x3F, and reads back data until success (max of SPI_MAX_RETRIES) or returns one of SPI_ERR.
 * @param lower - lower byte of 16-bit address
 * @param upper - upper byte of 16-bit address
 * @returns SUCCESS or one of SPI_ERR
 */
int write_sc_addrs(uint8_t lower, uint8_t upper) {
    // Write lower and upper bytes to 0x3E and 0x3F
    uint8_t tx_buf[3] = {0x00, 0x00, 0x00};
    tx_buf[0] = SUBCOMMAND_LOWER;
    tx_buf[1] = lower;
    tx_buf[2] = calculate_crc(tx_buf, 2);
    spi_w_blocking(tx_buf, 3);
    sleep_us(SPI_TRANSACTION_DELAY_US);
    tx_buf[0] = SUBCOMMAND_UPPER;
    tx_buf[1] = upper;
    tx_buf[2] = calculate_crc(tx_buf, 2);
    spi_w_blocking(tx_buf, 3);
    sleep_us(SPI_TRANSACTION_DELAY_US);

    uint8_t rx_lower, rx_upper;
    size_t tries = 0;
    int result;

    do {
        result = read_direct_command(SUBCOMMAND_LOWER, rx_lower);
        if (!result) {
            return result;
        }
        result = read_direct_command(SUBCOMMAND_UPPER, rx_upper);
        if (!result) {
            return result;
        }
    } while (rx_lower != lower && rx_upper != upper);
    return SUCCESS;
}

int main()
{
    stdio_init_all();
    spi_controller_init();

    /*********************************************************************************************/

    wakeup(); // Wake up the internal oscillator before starting communication


    size_t num_thermistors = 6;
    thermistor_t used_thermistors[] = {
        TS1_THERMISTOR,
        TS2_THERMISTOR,
        TS3_THERMISTOR,
        DFETOFF_THERMISTOR,
        DCHG_THERMISTOR,
        DDSG_THERMISTOR
    };

    size_t cells = 6;
    cell_voltage_t cell_commands[] = {
        CELL_1,
        CELL_2,
        CELL_3,
        CELL_4,
        CELL_5,
        LAST_CELL
    };

    while (true) {
        
        // printf("Control Status..........\n");
        // read_control_status();
        // printf("End Control Status......\n\n");

        
        printf("Reading All Voltages...\n");
        uint16_t voltages[cells];
        
        for (size_t i = 0; i < cells; i++) {
            if (read_voltage(cell_commands[i], &voltages[i])) {
                float voltage_v = voltages[i] / 1000.0; // Convert mV to V for easier readability
                printf("Cell %d Voltage: %.3f V\n", i+1, voltage_v); // Print voltage in volts with 3 decimal places
            } else {
                printf("Failed to read voltage for Cell %d at address 0x%02X\n", i+1, cell_commands[i]);
            }
        }
        uint16_t stack_voltage = 0;
        if (read_voltage(STACK_VOLTAGE, &stack_voltage)) {
            float stack_voltage_v = stack_voltage / 100.0; // Convert cV to V for easier readability
            printf("Stack Voltage: %.3f V\n", stack_voltage_v); // Print stack voltage in volts with 3 decimal places
        } else {
            printf("Failed to read stack voltage at address 0x%02X\n", STACK_VOLTAGE);
        }
        printf("\n"); // Print a newline after reading all cell voltages

        printf("Reading Temperatures...\n");
        uint16_t int_temperature = 0;
        thermistor_t int_temp_command = INT_THERMISTOR;
        if (read_temperature(int_temp_command, &int_temperature)) {
            // Convert 0.1K to Fahrenheit for debugging purposes
            float int_temp_f = (int_temperature * 0.1) * 9 / 5 - 459.67;
            printf("Internal Temperature: %.1f °F\n", int_temp_f);  
        } else {
            printf("Failed to read internal temperature\n");
        }

        for (size_t i = 0; i < num_thermistors; i++) {
            uint16_t temp = 0;
            if (read_temperature(used_thermistors[i], &temp)) {
                // Convert 0.1K to Fahrenheit for debugging purposes
                float temp_f = (temp * 0.1) * 9 / 5 - 459.67;
                printf("Thermistor at address 0x%02X Temperature: %.1f °F\n", used_thermistors[i], temp_f);  
            } else {
                printf("Failed to read temperature for Thermistor at address 0x%02X\n", used_thermistors[i]);
            }
        }

        printf("\n\n\n"); // Print a newline after reading stack voltage

        
        /*****************************************************************************************/
        // printf("Reading cell voltages...\n");
        // for (size_t i = 0; i < cells; i++) {

        //     printf("DEBUG: Reading voltage for Cell %d at address 0x%02X\n", i+1, cell_command + (i*2)); // Debug print for cell address being read
        //     uint16_t voltage = read_cell_voltage(cell_command + (i*2)); // Read voltage for the current cell
        //     printf("Cell %d Voltage: %d mV\n", i+1, voltage); // Print the voltage reading for the current cell

        //     sleep_us(SPI_TRANSACTION_DELAY); // Short delay between transmissions
        // }
        // printf("\n\n\n"); // Print a newline after reading all cells

        /*
        uint8_t rx_buf[3]; // Buffer to store received data
        uint8_t tx_buf[3] = {0x00, 0x00, 0x00}; // Dummy data to read LSB
        tx_buf[0] = 0x00;
        tx_buf[2] = calculate_crc(tx_buf, 2); // Calculate CRC for the command
        spi_w_blocking(tx_buf, 3);
        printf("tx_buf after sending LSB command: ");
        print_hex(tx_buf, 3); // Debug print for command being sent
        sleep_us(SPI_TRANSACTION_DELAY_US); // Short delay between transmissions
        spi_r_blocking(rx_buf, 3); // Read the response for the LSB
        lsb = rx_buf[1]; // Save the 2nd byte as LSB
        print_hex(rx_buf, 3); // Debug print for the response received
        tx_buf[0] = 0x01;
        tx_buf[2] = calculate_crc(tx_buf, 2); // Calculate CRC for the command
        spi_w_blocking(tx_buf, 3);
        printf("tx_buf after sending MSB command: ");
        print_hex(tx_buf, 3); // Debug print for command being sent
        sleep_us(SPI_TRANSACTION_DELAY_US); // Short delay between transmissions
        spi_r_blocking(rx_buf, 3); // Read the response for the MSB
        msb = rx_buf[1]; // Save the 2nd byte as MSB
        print_hex(rx_buf, 3); // Debug print for the response received

        uint16_t control_status = (msb << 8) | lsb; // Combine MSB and LSB to get the full voltage reading
        // Print the second bit of the control status to check if the BMS is in DEEPSLEEP mode (1 = DEEPSLEEP, 0 = ACTIVE)
        printf("BMS Control Status: 0x%04X, DEEPSLEEP Mode: %s\n", control_status, (control_status & 0x0002) ? "DEEPSLEEP" : "ACTIVE");
        
        */

        /*****************************************************************************************/
        sleep_ms(1000);
    }
}
