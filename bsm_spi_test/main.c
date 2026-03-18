#include <stdio.h>

#include "bq76972.h"
#include "pico/stdlib.h"

int main(void)
{
    stdio_init_all();
    bq76972_init();
    bq76972_wakeup();

    sleep_ms(5000);

    int therm_cfg_result = bq76972_configure_thermistors_default();
    if (therm_cfg_result != SUCCESS)
    {
        printf("Thermistor pin configuration failed (err=%d)\n",
               therm_cfg_result);
    }
    else
    {
        printf("Thermistor pin configuration succeeded\n");
    }

    // Verify the config actually wrote — this is critical for debugging
    bq76972_verify_thermistor_config();

    // The BQ76972 measurement loop needs time to complete a full cycle
    // after config changes. With 21 slots at 3ms each = ~63ms per loop,
    // and thermistors are spread across 3 loops, need ~200ms minimum.
    printf("Waiting for measurement loop to populate...\n");
    sleep_ms(500);

    // Configuration
    const size_t num_cells = 6;
    const cell_voltage_t cell_addresses[] = {CELL_1, CELL_2, CELL_3,
                                             CELL_4, CELL_5, CELL_6};

    // NOTE: This must match the order in bq76972_read_all_thermistor_values()
    // which reads: TS1, TS2, TS3, DFETOFF, DCHG, DDSG
    // (NOT HDQ — the old array had HDQ_THERMISTOR here which was wrong)
    const size_t num_sensors = BQ76972_THERMISTOR_COUNT;
    const thermistor_t sensor_addresses[] = {
        TS1_THERMISTOR,
        TS2_THERMISTOR,
        TS3_THERMISTOR,
        DFETOFF_THERMISTOR,
        DCHG_THERMISTOR,
        DDSG_THERMISTOR,
    };

    while (true)
    {
        printf("=== Battery Monitor Reading ===\n\n");

        // // Read cell voltages
        // printf("Cell Voltages:\n");
        // uint16_t voltages[num_cells];
        // for (size_t i = 0; i < num_cells; i++)
        // {
        //     if (bq76972_read_cell_voltage(cell_addresses[i], &voltages[i]) ==
        //         SUCCESS)
        //     {
        //         float voltage_v = voltages[i] / 1000.0f;
        //         printf("  Cell %zu: %.3f V\n", i + 1, voltage_v);
        //     }
        //     else
        //     {
        //         printf("  Cell %zu: READ ERROR\n", i + 1);
        //     }
        // }

        // // Read stack voltage
        // uint16_t stack_voltage = 0;
        // if (bq76972_read_cell_voltage(STACK_VOLTAGE, &stack_voltage) ==
        //     SUCCESS)
        // {
        //     float stack_v = stack_voltage / 100.0f;
        //     printf("  Stack: %.2f V\n", stack_v);
        // }
        // else
        // {
        //     printf("  Stack: READ ERROR\n");
        // }
        // printf("\n");

        // Read temperatures — read each sensor individually so we can see
        // exactly which address succeeds/fails and what the raw values are
        printf("Temperatures:\n");
        for (size_t i = 0; i < num_sensors; i++)
        {
            int16_t raw_dK = 0;
            int result =
                bq76972_read_temperature(sensor_addresses[i], &raw_dK);
            if (result == SUCCESS)
            {
                float temp_c = bq76972_temperature_dK_to_c(raw_dK);
                printf("  Sensor %zu (0x%02X): raw=%d (0x%04X) -> %.1f °C\n",
                       i + 1,
                       (unsigned int)sensor_addresses[i],
                       raw_dK,
                       (unsigned int)(uint16_t)raw_dK,
                       temp_c);
            }
            else
            {
                printf("  Sensor %zu (0x%02X): READ FAILED (err=%d)\n",
                       i + 1,
                       (unsigned int)sensor_addresses[i],
                       result);
            }
        }

        printf("\n");
        sleep_ms(500);
    }

    return 0;
}