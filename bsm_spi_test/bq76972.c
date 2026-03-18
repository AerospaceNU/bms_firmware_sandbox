#include "bq76972.h"

#include <stdio.h>

#include "hardware/spi.h"
#include "pico/stdlib.h"

// ============================================================================
// Configuration
// ============================================================================

#define SPI_PORT spi0
#define PIN_RX 0
#define PIN_CS 1
#define PIN_SCK 2
#define PIN_TX 3

#define SPI_BAUDRATE (250 * 1000)
#define BITS_PER_TRANSFER 8
#define SPI_CPOL 0
#define SPI_CPHA 0
#define SPI_ORDER 1

#define SPI_TRANSACTION_DELAY_US 2000
#define SPI_MAX_RETRIES 10
#define WAKEUP_DELAY_MS 5
#define SPI_RETRY_BACKOFF_US 2000

#define SUBCOMMAND_LOWER 0x3E
#define SUBCOMMAND_UPPER 0x3F
#define RESPONSE_BUFFER_START 0x40
#define RESPONSE_BUFFER_CHKSUM 0x60
#define RESPONSE_BUFFER_LENGTH 0x61

#define DATA_MEMORY_MAX_BYTES 32
#define DATA_MEMORY_RESPONSE_DELAY_US 10000
#define CONFIG_SETTLE_DELAY_MS 100

// Data memory addresses for pin configuration
#define DFETOFF_CONFIG_ADDR 0x92FB
#define TS1_CONFIG_ADDR 0x92FD
#define TS2_CONFIG_ADDR 0x92FE
#define TS3_CONFIG_ADDR 0x92FF
#define DCHG_CONFIG_ADDR 0x9301
#define DDSG_CONFIG_ADDR 0x9302

// For dedicated TS pins (TS1, TS2, TS3):
//   PIN_FXN[2:0] = 0b111 = thermistor mode
//   OPT[1:0] (bits [4:3]) = 00 = used for protections, 18K temperature model
//   OPT[3:2] (bits [6:5]) = 00 = 18K pull-up
// 0x07 matches TI's reference code for cell temperature thermistors.
#define THERMISTOR_CONFIG_TS_PIN 0x07

// For multifunction pins (DFETOFF, DCHG, DDSG):
//   PIN_FXN[2:0] = 0b011 = ADC input / thermistor mode
//   OPT[1:0] (bits [4:3]) = 01 = thermistor, reported but not used for protections
//   OPT[3:2] (bits [6:5]) = 00 = 18K temperature model
//   OPT[5:4] (bits [8:7]) = 00 = 18K pull-up
#define THERMISTOR_CONFIG_MULTIFUNCTION_PIN 0x0B

#define THERMISTOR_TS_DEFAULT_CONFIG THERMISTOR_CONFIG_TS_PIN
#define THERMISTOR_MF_DEFAULT_CONFIG THERMISTOR_CONFIG_MULTIFUNCTION_PIN

#define DEBUG 0

// ============================================================================
// Private Helper Functions
// ============================================================================

static void print_hex(uint8_t data[], size_t length)
{
    if (!DEBUG)
    {
        return;
    }

    printf("0x");
    for (size_t i = 0; i < length; i++)
    {
        printf("%02X", data[i]);
    }
    printf("\n");
}

static uint8_t calculate_crc(uint8_t data[], size_t length)
{
    uint8_t crc = 0;

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static int verify_crc(uint8_t rx_buf[], size_t length)
{
    if (length < 3)
    {
        return 0;
    }

    // Special cases documented by TI for SPI responses
    if (rx_buf[0] == 0xFF && rx_buf[1] == 0xFF &&
        (rx_buf[2] == 0xFF || rx_buf[2] == 0x00))
    {
        return 1;
    }

    return rx_buf[2] == calculate_crc(rx_buf, 2);
}

static int is_internal_osc_asleep(uint8_t rx_buf[])
{
    return rx_buf[0] == 0xFF && rx_buf[1] == 0xFF && rx_buf[2] == 0xFF;
}

static int is_outgoing_buffer_not_updated(uint8_t rx_buf[])
{
    return rx_buf[0] == 0xFF && rx_buf[1] == 0xFF && rx_buf[2] == 0x00;
}

static int spi_write_read(uint8_t tx_buf[], uint8_t rx_buf[], size_t length)
{
    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, length);
    gpio_put(PIN_CS, 1);
    print_hex(rx_buf, length);
    return verify_crc(rx_buf, length);
}

static void spi_write(uint8_t tx_buf[], size_t length)
{
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, tx_buf, length);
    gpio_put(PIN_CS, 1);
}

static int spi_read(uint8_t rx_buf[], size_t length)
{
    gpio_put(PIN_CS, 0);
    spi_read_blocking(SPI_PORT, 0x00, rx_buf, length);
    gpio_put(PIN_CS, 1);
    print_hex(rx_buf, length);
    return verify_crc(rx_buf, length);
}

static uint8_t calculate_dm_checksum(uint16_t addr, const uint8_t *data, size_t len)
{
    uint16_t sum = (uint16_t)(addr & 0xFFU) + (uint16_t)((addr >> 8) & 0xFFU);

    for (size_t i = 0; i < len; i++)
    {
        sum = (uint16_t)(sum + data[i]);
    }

    return (uint8_t)(~sum);
}

// Write two consecutive direct-command registers as two separate SPI transactions.
// The BQ769x2 SPI interface only supports single-byte register writes per
// transaction (unlike I2C which supports block writes).
static int bq76972_write_direct_pair(uint8_t command_lsb,
                                     uint8_t value_lsb,
                                     uint8_t command_msb,
                                     uint8_t value_msb)
{
    if ((uint8_t)(command_lsb + 1U) != command_msb)
    {
        return SPI_UNKNOWN_ERR;
    }

    // First register write: command_lsb = value_lsb
    int result = bq76972_write_direct(command_lsb, value_lsb);
    if (result != SUCCESS)
    {
        return result;
    }

    // Second register write: command_msb = value_msb
    return bq76972_write_direct(command_msb, value_msb);
}

// ============================================================================
// Public API Functions
// ============================================================================

void bq76972_init(void)
{
    spi_init(SPI_PORT, SPI_BAUDRATE);
    spi_set_format(SPI_PORT, BITS_PER_TRANSFER, SPI_CPOL, SPI_CPHA, SPI_ORDER);

    gpio_set_function(PIN_RX, GPIO_FUNC_SPI);
    gpio_pull_up(PIN_RX);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_TX, GPIO_FUNC_SPI);

    gpio_set_function(PIN_CS, GPIO_FUNC_SIO);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
}

int bq76972_read_direct(uint8_t command, uint8_t *data)
{
    if (data == NULL)
    {
        return SPI_UNKNOWN_ERR;
    }

    size_t tries = 0;
    int err_code = SPI_UNKNOWN_ERR;
    uint8_t tx_buf[3] = {command, 0x00, 0x00};
    uint8_t rx_buf[3] = {0};

    tx_buf[2] = calculate_crc(tx_buf, 2);

    while (tries < SPI_MAX_RETRIES)
    {
        spi_write(tx_buf, 3);
        sleep_us(SPI_TRANSACTION_DELAY_US);

        if (!spi_read(rx_buf, 3))
        {
            err_code = SPI_CRC_MISMATCH_ERR;
            tries++;
            sleep_us(SPI_RETRY_BACKOFF_US);
            continue;
        }

        if (is_outgoing_buffer_not_updated(rx_buf))
        {
            err_code = SPI_READ_TOO_FAST_ERR;
            tries++;
            sleep_us(SPI_RETRY_BACKOFF_US);
            continue;
        }

        if (is_internal_osc_asleep(rx_buf))
        {
            err_code = SPI_INTERNAL_OSC_ASLEEP_ERR;
            tries++;
            bq76972_wakeup();
            sleep_ms(WAKEUP_DELAY_MS);
            continue;
        }

        if (rx_buf[0] != command)
        {
            err_code = SPI_UNEXPECTED_RESPONSE_ERR;
            tries++;
            sleep_us(SPI_RETRY_BACKOFF_US);
            continue;
        }

        *data = rx_buf[1];
        return SUCCESS;
    }

    return err_code;
}

int bq76972_write_direct(uint8_t command, uint8_t data)
{
    // BQ769x2 SPI protocol: bit 7 of the first byte indicates R/W.
    // Bit 7 = 0 for read, bit 7 = 1 for write.
    uint8_t tx_buf[3] = {(uint8_t)(command | 0x80U), data, 0x00};
    tx_buf[2] = calculate_crc(tx_buf, 2);
    spi_write(tx_buf, 3);
    sleep_us(SPI_TRANSACTION_DELAY_US);
    return SUCCESS;
}

int bq76972_send_subcommand(uint16_t subcommand)
{
    return bq76972_write_direct_pair(SUBCOMMAND_LOWER,
                                     (uint8_t)(subcommand & 0xFFU),
                                     SUBCOMMAND_UPPER,
                                     (uint8_t)((subcommand >> 8) & 0xFFU));
}

int bq76972_enter_config_update(void)
{
    int result = bq76972_send_subcommand(0x0090);
    if (result == SUCCESS)
    {
        sleep_ms(2);
    }
    return result;
}

int bq76972_exit_config_update(void)
{
    int result = bq76972_send_subcommand(0x0092);
    if (result == SUCCESS)
    {
        sleep_ms(100);
    }
    return result;
}

int bq76972_write_data_memory(uint16_t addr, const uint8_t *data, size_t len,
                              bool use_config_update)
{
    if (data == NULL || len == 0 || len > DATA_MEMORY_MAX_BYTES)
    {
        return SPI_UNKNOWN_ERR;
    }

    int result = SUCCESS;

    if (use_config_update)
    {
        result = bq76972_enter_config_update();
        if (result != SUCCESS)
        {
            return result;
        }
    }

    result = bq76972_write_direct_pair(SUBCOMMAND_LOWER,
                                       (uint8_t)(addr & 0xFFU),
                                       SUBCOMMAND_UPPER,
                                       (uint8_t)((addr >> 8) & 0xFFU));

    for (size_t i = 0; i < len && result == SUCCESS; i++)
    {
        result = bq76972_write_direct((uint8_t)(RESPONSE_BUFFER_START + i), data[i]);
    }

    if (result == SUCCESS)
    {
        uint8_t checksum = calculate_dm_checksum(addr, data, len);
        uint8_t total_length = (uint8_t)(len + 4U);

        result = bq76972_write_direct_pair(RESPONSE_BUFFER_CHKSUM,
                                           checksum,
                                           RESPONSE_BUFFER_LENGTH,
                                           total_length);
    }

    if (use_config_update)
    {
        int exit_result = bq76972_exit_config_update();
        if (result == SUCCESS)
        {
            result = exit_result;
        }
    }

    if (result == SUCCESS)
    {
        sleep_ms(CONFIG_SETTLE_DELAY_MS);
    }

    return result;
}

int bq76972_write_data_memory_u8(uint16_t addr, uint8_t value, bool use_config_update)
{
    return bq76972_write_data_memory(addr, &value, 1, use_config_update);
}

int bq76972_write_data_memory_u16(uint16_t addr, uint16_t value, bool use_config_update)
{
    uint8_t bytes[2] = {
        (uint8_t)(value & 0xFFU),
        (uint8_t)((value >> 8) & 0xFFU),
    };

    return bq76972_write_data_memory(addr, bytes, 2, use_config_update);
}

int bq76972_read_data_memory(uint16_t addr, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || len > DATA_MEMORY_MAX_BYTES)
    {
        return SPI_UNKNOWN_ERR;
    }

    int result = bq76972_write_direct_pair(SUBCOMMAND_LOWER,
                                           (uint8_t)(addr & 0xFFU),
                                           SUBCOMMAND_UPPER,
                                           (uint8_t)((addr >> 8) & 0xFFU));
    if (result != SUCCESS)
    {
        return result;
    }

    sleep_us(DATA_MEMORY_RESPONSE_DELAY_US);

    // Poll 0x3E until the device echoes back the address we wrote,
    // indicating the subcommand has completed. 0xFF means still processing.
    for (int poll = 0; poll < 20; poll++)
    {
        uint8_t subcmd_lo = 0xFF;
        result = bq76972_read_direct(SUBCOMMAND_LOWER, &subcmd_lo);
        if (result == SUCCESS && subcmd_lo == (uint8_t)(addr & 0xFFU))
        {
            break;
        }
        sleep_ms(1);
    }

    // Diagnostic: read the first 4 bytes of the response buffer plus
    // checksum and length, so we can see exactly what the device returned.
    uint8_t raw_buf[4] = {0};
    for (size_t i = 0; i < 4 && i < (len + 2); i++)
    {
        result = bq76972_read_direct((uint8_t)(RESPONSE_BUFFER_START + i), &raw_buf[i]);
        if (result != SUCCESS)
        {
            printf("[DM_READ 0x%04X] Failed reading 0x%02X (err=%d)\n",
                   addr, (unsigned)(RESPONSE_BUFFER_START + i), result);
            return result;
        }
    }

    uint8_t reported_checksum = 0;
    uint8_t reported_length = 0;

    result = bq76972_read_direct(RESPONSE_BUFFER_CHKSUM, &reported_checksum);
    if (result != SUCCESS)
    {
        printf("[DM_READ 0x%04X] Failed reading checksum (err=%d)\n", addr, result);
        return result;
    }

    result = bq76972_read_direct(RESPONSE_BUFFER_LENGTH, &reported_length);
    if (result != SUCCESS)
    {
        printf("[DM_READ 0x%04X] Failed reading length (err=%d)\n", addr, result);
        return result;
    }

    printf("[DM_READ 0x%04X] buf=[0x%02X 0x%02X 0x%02X 0x%02X] chk=0x%02X len=%d\n",
           addr, raw_buf[0], raw_buf[1], raw_buf[2], raw_buf[3],
           reported_checksum, reported_length);

    // Try interpretation A: data starts at 0x40 (no address echo)
    uint8_t chk_a = calculate_dm_checksum(addr, &raw_buf[0], len);
    // Try interpretation B: data starts at 0x42 (address echo at 0x40-0x41)
    uint8_t chk_b = calculate_dm_checksum(addr, &raw_buf[2], len);
    printf("[DM_READ 0x%04X] expected_chk(0x40)=0x%02X expected_chk(0x42)=0x%02X\n",
           addr, chk_a, chk_b);

    // Use whichever interpretation matches
    if (reported_checksum == chk_a && reported_length >= (uint8_t)(len + 4U))
    {
        // Data starts at 0x40
        for (size_t i = 0; i < len; i++)
        {
            data[i] = raw_buf[i];
        }
        printf("[DM_READ 0x%04X] Matched at offset 0 -> data=0x%02X\n",
               addr, data[0]);
        return SUCCESS;
    }
    else if (reported_checksum == chk_b && reported_length >= (uint8_t)(len + 4U))
    {
        // Data starts at 0x42
        for (size_t i = 0; i < len; i++)
        {
            data[i] = raw_buf[2 + i];
        }
        printf("[DM_READ 0x%04X] Matched at offset 2 -> data=0x%02X\n",
               addr, data[0]);
        return SUCCESS;
    }
    else
    {
        printf("[DM_READ 0x%04X] NO MATCH - neither offset works\n", addr);
        // Fall back to offset 0 for the data, but report CRC error
        for (size_t i = 0; i < len; i++)
        {
            data[i] = raw_buf[i];
        }
        return SPI_CRC_MISMATCH_ERR;
    }
}

int bq76972_read_data_memory_u8(uint16_t addr, uint8_t *value)
{
    return bq76972_read_data_memory(addr, value, 1);
}

int bq76972_read_data_memory_u16(uint16_t addr, uint16_t *value)
{
    if (value == NULL)
    {
        return SPI_UNKNOWN_ERR;
    }

    uint8_t bytes[2] = {0};
    int result = bq76972_read_data_memory(addr, bytes, 2);
    if (result == SUCCESS)
    {
        *value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    }
    return result;
}

int bq76972_read_u16(uint8_t lsb_command, uint16_t *value)
{
    if (value == NULL)
    {
        return SPI_UNKNOWN_ERR;
    }

    uint8_t lsb = 0;
    uint8_t msb = 0;
    int last_error = SPI_UNKNOWN_ERR;

    for (int i = 0; i < SPI_MAX_RETRIES; i++)
    {
        int result = bq76972_read_direct(lsb_command, &lsb);
        if (result != SUCCESS)
        {
            last_error = result;
            continue;
        }

        result = bq76972_read_direct((uint8_t)(lsb_command + 1U), &msb);
        if (result != SUCCESS)
        {
            last_error = result;
            continue;
        }

        *value = (uint16_t)lsb | ((uint16_t)msb << 8);
        return SUCCESS;
    }

    return last_error;
}

int bq76972_read_cell_voltage(cell_voltage_t cell, uint16_t *voltage_mv)
{
    return bq76972_read_u16((uint8_t)cell, voltage_mv);
}

int bq76972_read_all_cell_voltages(uint16_t *voltages, size_t num_cells)
{
    if (voltages == NULL || num_cells == 0)
    {
        return 0;
    }

    for (size_t i = 0; i < num_cells; i++)
    {
        cell_voltage_t cell = (cell_voltage_t)(CELL_1 + (i * 2));
        if (bq76972_read_cell_voltage(cell, &voltages[i]) != SUCCESS)
        {
            return 0;
        }
    }

    return 1;
}

int bq76972_read_temperature(thermistor_t sensor, int16_t *temperature_dK)
{
    if (temperature_dK == NULL)
    {
        return SPI_UNKNOWN_ERR;
    }

    uint16_t raw_value = 0;
    int result = bq76972_read_u16((uint8_t)sensor, &raw_value);
    if (result == SUCCESS)
    {
        *temperature_dK = (int16_t)raw_value;
    }

    return result;
}

int bq76972_read_all_temperatures(const thermistor_t *sensors,
                                  int16_t *temperatures_dK,
                                  size_t num_sensors)
{
    if (sensors == NULL || temperatures_dK == NULL || num_sensors == 0)
    {
        return SPI_UNKNOWN_ERR;
    }

    for (size_t i = 0; i < num_sensors; i++)
    {
        int result = bq76972_read_temperature(sensors[i], &temperatures_dK[i]);
        if (result != SUCCESS)
        {
            return result;
        }
    }

    return SUCCESS;
}

float bq76972_temperature_dK_to_c(int16_t temperature_dK)
{
    return ((float)temperature_dK / 10.0f) - 273.15f;
}

int bq76972_configure_thermistors(uint8_t ts1_config,
                                  uint8_t ts2_config,
                                  uint8_t ts3_config,
                                  uint8_t dfetoff_config,
                                  uint8_t dchg_config,
                                  uint8_t ddsg_config)
{
    int result = bq76972_enter_config_update();
    if (result != SUCCESS)
    {
        return result;
    }

    result = bq76972_write_data_memory_u8(DFETOFF_CONFIG_ADDR, dfetoff_config, false);
    if (result == SUCCESS)
    {
        result = bq76972_write_data_memory_u8(TS1_CONFIG_ADDR, ts1_config, false);
    }
    if (result == SUCCESS)
    {
        result = bq76972_write_data_memory_u8(TS2_CONFIG_ADDR, ts2_config, false);
    }
    if (result == SUCCESS)
    {
        result = bq76972_write_data_memory_u8(TS3_CONFIG_ADDR, ts3_config, false);
    }
    if (result == SUCCESS)
    {
        result = bq76972_write_data_memory_u8(DCHG_CONFIG_ADDR, dchg_config, false);
    }
    if (result == SUCCESS)
    {
        result = bq76972_write_data_memory_u8(DDSG_CONFIG_ADDR, ddsg_config, false);
    }

    {
        int exit_result = bq76972_exit_config_update();
        if (result == SUCCESS)
        {
            result = exit_result;
        }
    }

    if (result == SUCCESS)
    {
        sleep_ms(CONFIG_SETTLE_DELAY_MS);
    }

    return result;
}

int bq76972_configure_thermistors_default(void)
{
    return bq76972_configure_thermistors(THERMISTOR_TS_DEFAULT_CONFIG,
                                         THERMISTOR_TS_DEFAULT_CONFIG,
                                         THERMISTOR_TS_DEFAULT_CONFIG,
                                         THERMISTOR_MF_DEFAULT_CONFIG,
                                         THERMISTOR_MF_DEFAULT_CONFIG,
                                         THERMISTOR_MF_DEFAULT_CONFIG);
}

int bq76972_read_thermistor_pin_configs(uint8_t *ts1_config,
                                        uint8_t *ts2_config,
                                        uint8_t *ts3_config,
                                        uint8_t *dfetoff_config,
                                        uint8_t *dchg_config,
                                        uint8_t *ddsg_config)
{
    if (ts1_config == NULL || ts2_config == NULL || ts3_config == NULL ||
        dfetoff_config == NULL || dchg_config == NULL || ddsg_config == NULL)
    {
        return SPI_UNKNOWN_ERR;
    }

    int result = bq76972_read_data_memory_u8(DFETOFF_CONFIG_ADDR, dfetoff_config);
    if (result == SUCCESS)
    {
        result = bq76972_read_data_memory_u8(TS1_CONFIG_ADDR, ts1_config);
    }
    if (result == SUCCESS)
    {
        result = bq76972_read_data_memory_u8(TS2_CONFIG_ADDR, ts2_config);
    }
    if (result == SUCCESS)
    {
        result = bq76972_read_data_memory_u8(TS3_CONFIG_ADDR, ts3_config);
    }
    if (result == SUCCESS)
    {
        result = bq76972_read_data_memory_u8(DCHG_CONFIG_ADDR, dchg_config);
    }
    if (result == SUCCESS)
    {
        result = bq76972_read_data_memory_u8(DDSG_CONFIG_ADDR, ddsg_config);
    }

    return result;
}

int bq76972_verify_thermistor_config(void)
{
    uint8_t ts1, ts2, ts3, dfetoff, dchg, ddsg;

    int result = bq76972_read_thermistor_pin_configs(&ts1, &ts2, &ts3,
                                                     &dfetoff, &dchg, &ddsg);
    if (result != SUCCESS)
    {
        printf("[BQ76972] Failed to read thermistor configs (err=%d)\n", result);
        return result;
    }

    printf("[BQ76972] Thermistor config readback:\n");
    printf("  TS1=0x%02X  TS2=0x%02X  TS3=0x%02X\n", ts1, ts2, ts3);
    printf("  DFETOFF=0x%02X  DCHG=0x%02X  DDSG=0x%02X\n", dfetoff, dchg, ddsg);

    int ok = 1;
    if (ts1 == 0x00)
    {
        printf("[BQ76972] WARNING: TS1 config is 0x00 (not enabled)\n");
        ok = 0;
    }
    if (ts2 == 0x00)
    {
        printf("[BQ76972] WARNING: TS2 config is 0x00 (not enabled)\n");
        ok = 0;
    }
    if (ts3 == 0x00)
    {
        printf("[BQ76972] WARNING: TS3 config is 0x00 (not enabled)\n");
        ok = 0;
    }
    if (dfetoff == 0x00)
    {
        printf("[BQ76972] WARNING: DFETOFF config is 0x00 (not enabled)\n");
        ok = 0;
    }
    if (dchg == 0x00)
    {
        printf("[BQ76972] WARNING: DCHG config is 0x00 (not enabled)\n");
        ok = 0;
    }
    if (ddsg == 0x00)
    {
        printf("[BQ76972] WARNING: DDSG config is 0x00 (not enabled)\n");
        ok = 0;
    }

    if (!ok)
    {
        printf("[BQ76972] One or more thermistor pins failed to configure!\n");
    }

    return ok ? SUCCESS : SPI_UNEXPECTED_RESPONSE_ERR;
}

int bq76972_read_all_thermistor_values(int16_t *temperatures_dK,
                                       size_t num_temperatures)
{
    static const thermistor_t thermistors[BQ76972_THERMISTOR_COUNT] = {
        TS1_THERMISTOR,
        TS2_THERMISTOR,
        TS3_THERMISTOR,
        DFETOFF_THERMISTOR,
        DCHG_THERMISTOR,
        DDSG_THERMISTOR,
    };

    if (temperatures_dK == NULL ||
        num_temperatures < (size_t)BQ76972_THERMISTOR_COUNT)
    {
        return SPI_UNKNOWN_ERR;
    }

    bq76972_wakeup();
    sleep_ms(2);

    return bq76972_read_all_temperatures(thermistors,
                                         temperatures_dK,
                                         BQ76972_THERMISTOR_COUNT);
}

void bq76972_wakeup(void)
{
    uint8_t tx_buf[3] = {0x00, 0x00, 0x00};
    tx_buf[2] = calculate_crc(tx_buf, 2);

    spi_write(tx_buf, 3);
    sleep_ms(WAKEUP_DELAY_MS);
    (void)spi_read(tx_buf, 3);
}