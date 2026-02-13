// bq76972_spi.h
#ifndef BQ76972_SPI_H
#define BQ76972_SPI_H

#include <stdint.h>
#include <stdbool.h>

void bq76972_init(void);
bool bq76972_wake(void);
void bq76972_write_reg(uint8_t addr, uint8_t data);
uint8_t bq76972_read_reg(uint8_t addr);
bool bq76972_read_device_number(uint16_t *dev_num);

#endif