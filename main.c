/* FRDM-MCXN947 + VL53L0X: one distance measurement at a time, printed to UART.
 * Read main() first, then follow calls into hardware_init.c and platform.c.
 * This is a bare-metal application: no FreeRTOS, camera or inference engine.
 */
#include "app_config.h"
#include "fsl_debug_console.h" /* SDK PRINTF routes text to MCU-Link USB serial. */
#include "vl53l0x_api.h"      /* ST's initialization and measurement functions. */

static void Check(VL53L0X_DEV sensor, VL53L0X_Error error, const char *operation)
{
    /* const char * points to a string that this function must not change.
     * A successful call returns immediately so normal execution continues.
     */
    if (error == VL53L0X_ERROR_NONE)
    {
        return;
    }
    char description[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(error, description);
    /* %s prints a string; %d prints signed decimal. Casts match those types.
     * \r\n is a carriage return + newline for typical serial terminals.
     */
    PRINTF("ERROR during %s: %s (ST=%d, last NXP I2C=%d)\r\n",
           operation, description, (int)error, (int)sensor->last_i2c_status);
    PRINTF("Check sensor power, SDA/SCL, pull-ups and XSHUT. Reset to retry.\r\n");
    /* Don't keep issuing sensor commands after a failed initialization or read.
     * This is an intentional fatal-error loop, useful as a debugger breakpoint.
     */
    for (;;)
    {
        APP_DelayMs(1000U);
    }
}

int main(void)
{
    APP_InitHardware();
    PRINTF("\r\nFRDM-MCXN947 / VL53L0X distance example\r\n");
    PRINTF("I2C2: SDA J2-18, SCL J2-20, address 0x29\r\n");

    /* static gives this object a lifetime spanning the entire program.
     * {0} zero-initializes ST's internal state and our extra fields.
     * &sensor is its address; ST receives that pointer and updates the object.
     */
    static VL53L0X_Dev_t sensor = {0};
    sensor.I2cDevAddr = APP_SENSOR_ADDRESS_7BIT;
    sensor.i2c       = APP_I2C_BASE;

    /* Allow time after power-up. XSHUT must already be held/released HIGH by
     * the breakout. This version does not drive the optional XSHUT pin.
     */
    APP_DelayMs(10U);

    /* A real register read tests address acknowledgement and read transactions.
     * Register 0xC0 identifies the VL53L0X; expected model ID is 0xEE.
     */
    uint8_t model_id = 0U;
    Check(&sensor, VL53L0X_RdByte(&sensor, 0xC0U, &model_id), "model ID read");
    PRINTF("Model ID: 0x%02x (expected 0xee)\r\n", (unsigned int)model_id);
    if (model_id != 0xEEU)
    {
        Check(&sensor, VL53L0X_ERROR_NOT_SUPPORTED, "model ID mismatch");
    }

    /* DataInit loads the driver's initial state and sensor configuration.
     * StaticInit loads tuning/default operating settings and reference data.
     * These are both required; finding a device on I2C does not initialize it.
     */
    Check(&sensor, VL53L0X_DataInit(&sensor), "DataInit");
    Check(&sensor, VL53L0X_StaticInit(&sensor), "StaticInit");

    /* SPADs are tiny light-sensitive detector elements. Reference management
     * chooses a suitable reference detector configuration for this device.
     * Output parameters receive the chosen count and detector type.
     */
    uint32_t reference_spad_count = 0U;
    uint8_t aperture_spads = 0U;
    Check(&sensor, VL53L0X_PerformRefSpadManagement(&sensor,
          &reference_spad_count, &aperture_spads), "reference SPAD management");

    /* Reference calibration adjusts internal VHV/phase settings needed for
     * ranging. ST performs the device-specific sequence; we check its result.
     * This does not replace offset/crosstalk calibration for a final enclosure.
     */
    uint8_t vhv_settings = 0U;
    uint8_t phase_calibration = 0U;
    Check(&sensor, VL53L0X_PerformRefCalibration(&sensor,
          &vhv_settings, &phase_calibration), "reference calibration");

    /* Single ranging produces one measurement per explicit request.
     * The timing budget sets measurement work time, in microseconds.
     * 33 ms is a starting setting, not a promise of total loop duration.
     */
    Check(&sensor, VL53L0X_SetDeviceMode(&sensor,
          VL53L0X_DEVICEMODE_SINGLE_RANGING), "single-ranging mode");
    Check(&sensor, VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&sensor,
          APP_TIMING_BUDGET_US), "measurement timing budget");
    PRINTF("Initialized: reference SPAD count=%u, aperture=%u\r\n",
           (unsigned int)reference_spad_count, (unsigned int)aperture_spads);

    for (;;) /* Embedded programs normally keep running until reset/power-off. */
    {
        VL53L0X_RangingMeasurementData_t measurement = {0};
        /* ST starts ranging, polls for completion, reads results and clears
         * the measurement interrupt flag. The GPIO interrupt wire is unused.
         * Our PollingDelay is called inside ST's completion-waiting loop.
         */
        Check(&sensor, VL53L0X_PerformSingleRangingMeasurement(&sensor,
              &measurement), "single measurement");

        /* API success means the operation completed. RangeStatus separately
         * tells us whether the optical result is valid. Zero means valid.
         * An invalid measurement is NOT a distance of zero millimeters.
         */
        if (measurement.RangeStatus == 0U)
        {
            PRINTF("Distance: %u mm\r\n", (unsigned int)measurement.RangeMilliMeter);
        }
        else
        {
            char reason[VL53L0X_MAX_STRING_LENGTH];
            VL53L0X_GetRangeStatusString(measurement.RangeStatus, reason);
            PRINTF("No valid distance: %s (status=%u)\r\n",
                   reason, (unsigned int)measurement.RangeStatus);
        }
        APP_DelayMs(APP_PAUSE_MS); /* Slow the output enough to read comfortably. */
    }
}
