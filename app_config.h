/* Settings you can change without editing ST's driver. */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "fsl_lpi2c.h" /* LPI2C2 and NXP's status_t / transfer types. */

/* These pins match NXP's FRDM-MCXN947 LPI2C2 board-to-board example:
 * P4_0 = SDA = J2 pin 18; P4_1 = SCL = J2 pin 20.
 * This program uses the sensor alone; disconnect camera/display shields that
 * could occupy these pins. Check your exact breakout before connecting power.
 */
#define APP_I2C_BASE             LPI2C2
#define APP_I2C_CLOCK_HZ         CLOCK_GetLPFlexCommClkFreq(2U)
#define APP_I2C_BAUDRATE_HZ      100000U
#define APP_SENSOR_ADDRESS_7BIT  0x29U
#define APP_TIMING_BUDGET_US     33000U
#define APP_PAUSE_MS             200U

/* U means unsigned integer. 100000 clocks/sec is 100 kHz. 33000 us is 33 ms.
 * The pause is AFTER a measurement, so it is not the total sampling period.
 */
void APP_InitHardware(void);
void APP_DelayMs(uint32_t milliseconds);

#endif
