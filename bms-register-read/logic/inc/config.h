#ifndef CONFIG_H
#define CONFIG_H

/* Global Includes */
#include <stdint.h>

/* Local Includes */
#include "hardware/spi.h"

//*************************************************************************************************
// Project-wide configuration settings
//*************************************************************************************************


//*************************************************************************************************
// SPI Configuration
//*************************************************************************************************

#define SPI_CONTROLLER_PORT           spi0
#define SPI_CONTROLLER_BAUDRATE       1000*1000  // 1 MHz
#define SPI_CONTROLLER_BITS_PER_WORD  8
#define SPI_CONTROLLER_CPOL           SPI_CPOL_0
#define SPI_CONTROLLER_CPHA           SPI_CPHA_0
#define SPI_CONTROLLER_ORDER          SPI_MSB_FIRST

// Pin Definitions
#define SPI_CONTROLLER_PIN_SDI        0 // RX
#define SPI_CONTROLLER_PIN_CSB        1 // Chip Select
#define SPI_CONTROLLER_PIN_SCK        2 // Clock
#define SPI_CONTROLLER_PIN_SDO        3 // TX






#endif // CONFIG_H