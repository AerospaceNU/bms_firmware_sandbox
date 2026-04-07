#include <stdio.h>

#include "bq76972.h"
#include "pico/stdlib.h"

int main(void)
{
    stdio_init_all();
    bq76972_init();
    bq76972_wakeup();

    sleep_ms(5000);

    // Set current resolution to 0.1 mA per count
    int cfg_result = bq76972_configure_user_amps(USER_AMPS_0_1_MA);
    if (cfg_result != SUCCESS)
    {
        printf("USER_AMPS config failed (err=%d), using default\n", cfg_result);
    }

    // int therm_cfg_result = bq76972_configure_thermistors_default();
    // if (therm_cfg_result != SUCCESS)
    // {
    //     printf("Thermistor pin configuration failed (err=%d)\n",
    //            therm_cfg_result);
    // }
    // else
    // {
    //     printf("Thermistor pin configuration succeeded\n");
    // }

    // // Verify the config actually wrote — this is critical for debugging
    // bq76972_verify_thermistor_config();

    // // The BQ76972 measurement loop needs time to complete a full cycle
    // // after config changes. With 21 slots at 3ms each = ~63ms per loop,
    // // and thermistors are spread across 3 loops, need ~200ms minimum.
    // printf("Waiting for measurement loop to populate...\n");
    // sleep_ms(500);

    while (true)
    {
        printf("=== Battery Monitor Reading ===\n\n");

        // Read current via CC2 (0x3A), scaled by USER_AMPS setting
        float current_mA = 0.0f;
        int result = bq76972_read_current_mA(&current_mA);
        if (result == SUCCESS)
        {
            printf("Current: %.1f mA\n", current_mA);
        }
        else
        {
            printf("Current: READ ERROR (err=%d)\n", result);
        }

        // // Read temperatures
        // printf("Temperatures:\n");
        // const size_t num_sensors = BQ76972_THERMISTOR_COUNT;
        // const thermistor_t sensor_addresses[] = {
        //     TS1_THERMISTOR, TS2_THERMISTOR, TS3_THERMISTOR,
        //     DFETOFF_THERMISTOR, DCHG_THERMISTOR, DDSG_THERMISTOR,
        // };
        // for (size_t i = 0; i < num_sensors; i++)
        // {
        //     int16_t raw_dK = 0;
        //     int tres = bq76972_read_temperature(sensor_addresses[i], &raw_dK);
        //     if (tres == SUCCESS)
        //     {
        //         float temp_c = bq76972_temperature_dK_to_c(raw_dK);
        //         printf("  Sensor %zu (0x%02X): raw=%d (0x%04X) -> %.1f °C\n",
        //                i + 1, (unsigned int)sensor_addresses[i],
        //                raw_dK, (unsigned int)(uint16_t)raw_dK, temp_c);
        //     }
        //     else
        //     {
        //         printf("  Sensor %zu (0x%02X): READ FAILED (err=%d)\n",
        //                i + 1, (unsigned int)sensor_addresses[i], tres);
        //     }
        // }

        printf("\n");
        sleep_ms(500);
    }

    return 0;
}