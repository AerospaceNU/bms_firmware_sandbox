#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define I2C_SDA  8
#define I2C_SCL  9

// Dummy battery data (replace with real sensor readings later)
#define BATT_VOLTAGE_MV        15268
#define BATT_CURRENT_MA        -6700
#define BATT_TEMP_DK           2981
#define BATT_REMAINING_MAH     4000
#define BATT_FULL_MAH          5000
#define BATT_STATE_OF_CHARGE   80
#define BATT_CYCLE_COUNT       12

// Dummy cell voltages in mV (4S LiPo, replace with real readings later)
#define CELL1_MV               3676
#define CELL2_MV               3696
#define CELL3_MV               4200
#define CELL4_MV               3696

static uint8_t current_register = 0x00;
static uint8_t byte_index = 0;

uint8_t get_register_byte(uint8_t reg, uint8_t byte_idx) {
    uint16_t value = 0;

    switch (reg) {
        case 0x08: value = BATT_TEMP_DK;           break;
        case 0x09: value = BATT_VOLTAGE_MV;        break;
        case 0x0A: value = BATT_CURRENT_MA;        break;
        case 0x0D: value = BATT_STATE_OF_CHARGE;   break;
        case 0x0F: value = BATT_REMAINING_MAH;     break;
        case 0x10: value = BATT_FULL_MAH;          break;
        case 0x13: value = BATT_CYCLE_COUNT;        break;

        // Cell voltages (ArduPilot SMBus Generic reads 0x3F down to 0x34)
        case 0x3F: value = CELL1_MV;               break;
        case 0x3E: value = CELL2_MV;               break;
        case 0x3D: value = CELL3_MV;               break;
        case 0x3C: value = CELL4_MV;               break;

        default:   value = 0x0000;                 break;
    }

    if (byte_idx == 0) return (uint8_t)(value & 0xFF);
    else               return (uint8_t)((value >> 8) & 0xFF);
}

void i2c_slave_irq_handler() {
    uint32_t status = i2c_get_hw(I2C_PORT)->intr_stat;

    if (status & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        current_register = (uint8_t)i2c_get_hw(I2C_PORT)->data_cmd;
        byte_index = 0;
    }

    if (status & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        i2c_get_hw(I2C_PORT)->data_cmd = get_register_byte(current_register, byte_index);
        (void)i2c_get_hw(I2C_PORT)->clr_rd_req;
        byte_index++;
    }
}

int main() {
    stdio_init_all();

    i2c_init(I2C_PORT, 100 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    i2c_set_slave_mode(I2C_PORT, true, 0x0B);

    irq_set_exclusive_handler(I2C0_IRQ, i2c_slave_irq_handler);
    irq_set_enabled(I2C0_IRQ, true);

    i2c_get_hw(I2C_PORT)->intr_mask = I2C_IC_INTR_MASK_M_RD_REQ_BITS |
                                       I2C_IC_INTR_MASK_M_RX_FULL_BITS;

    while (true) {
        tight_loop_contents();
    }
}
