/* MCU-specific wiring and timing. ST's sensor code does not configure pins. */
#include "app_config.h"
#include "board.h"
#include "clock_config.h"
#include "fsl_port.h"

void APP_DelayMs(uint32_t milliseconds)
{
    /* Busy waiting keeps the CPU occupied. No RTOS/scheduler is required.
     * Split into milliseconds so milliseconds * 1000 cannot overflow here.
     * SystemCoreClock is the CPU frequency maintained by the SDK clock code.
     */
    while (milliseconds != 0U)
    {
        SDK_DelayAtLeastUs(1000U, SystemCoreClock);
        --milliseconds;
    }
}

void APP_InitHardware(void)
{
    /* Start the board's standard CPU clocks before configuring peripherals. */
    BOARD_InitBootClocks();

    /* Attach a 12 MHz free-running oscillator to the USB-serial UART and I2C.
     * The divider of 1 passes this clock unchanged; I2C divides it further.
     * FLEXCOMM4 serves UART; FLEXCOMM2 serves our sensor's LPI2C controller.
     */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1U);
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom2Clk, 1U);

    /* PORT blocks control which internal peripheral reaches each physical pin.
     * Clock gating saves power; registers must have a clock before use.
     */
    CLOCK_EnableClock(kCLOCK_Port1);
    CLOCK_EnableClock(kCLOCK_Port4);

    /* A structure groups related settings; .field = value names each field.
     * = {0} starts all remaining fields at zero before we set them explicitly.
     * &pin passes the address of the structure to the SDK function.
     */
    port_pin_config_t pin = {0};
    pin.pullSelect          = kPORT_PullDisable;
    pin.pullValueSelect     = kPORT_LowPullResistor;
    pin.slewRate            = kPORT_FastSlewRate;
    pin.passiveFilterEnable = kPORT_PassiveFilterDisable;
    pin.openDrainEnable     = kPORT_OpenDrainDisable;
    pin.driveStrength       = kPORT_LowDriveStrength;
    pin.mux                 = kPORT_MuxAlt2;
    pin.inputBuffer         = kPORT_InputBufferEnable;
    pin.invertInput         = kPORT_InputNormal;
    pin.lockRegister        = kPORT_UnlockRegister;

    /* UART4 RX and TX use P1_8 and P1_9, connected to the MCU-Link bridge.
     * Alt2 selects FC4 functions on these two pins; it isn't GPIO mode.
     */
    PORT_SetPinConfig(PORT1, 8U, &pin);
    PORT_SetPinConfig(PORT1, 9U, &pin);

    /* Alt2 on P4_0/P4_1 selects FC2 SDA/SCL. The peripheral is configured for
     * open-drain I2C below. These pad settings follow NXP's LPI2C2 example.
     * Internal pull-ups help idle lines rise; appropriate EXTERNAL pull-ups
     * to 3.3 V (often already on the breakout) are needed for a sound I2C bus.
     */
    pin.pullSelect = kPORT_PullUp;
    PORT_SetPinConfig(PORT4, 0U, &pin);
    PORT_SetPinConfig(PORT4, 1U, &pin);

    /* Initializes the SDK's PRINTF backend at board default 115200 baud. */
    BOARD_InitDebugConsole();

    lpi2c_master_config_t config;
    LPI2C_MasterGetDefaultConfig(&config);
    config.baudRate_Hz = APP_I2C_BAUDRATE_HZ;
    config.pinConfig = kLPI2C_2PinOpenDrain;
    /* Open drain drives LOW or releases the wire; pull-ups create HIGH.
     * MasterInit also selects LPI2C mode in this SDK's LP_FLEXCOMM driver.
     */
    LPI2C_MasterInit(APP_I2C_BASE, &config, APP_I2C_CLOCK_HZ);
}
