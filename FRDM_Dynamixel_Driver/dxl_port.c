/*
 * dxl_port.c  -  hardware side of the servo bus. See dxl_port.h for pins.
 */
#include "dxl_port.h"

#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"
#include "fsl_lpflexcomm.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

/* ---- which hardware we use ---- */
#define DXL_UART          LPUART2
#define DXL_FLEXCOMM_INST 2u
#define DXL_UART_IRQ      LP_FLEXCOMM2_IRQn
#define DXL_UART_CLK_HZ   CLOCK_GetLPFlexCommClkFreq(2u) /* 12 MHz FRO, set in hardware_init.c */

#define TXEN_PORT PORT0
#define TXEN_GPIO GPIO0
#define TXEN_PIN  29u /* P0_29 = Arduino D2 = J1 pin 6 */

/* ---- receive ring buffer ----
 * The interrupt (producer) writes at 'head'; the main code (consumer) reads
 * at 'tail'. Only ONE side ever changes each index, so no locking is needed.
 * Size is a power of 2 so "wrap around" is just "& (SIZE-1)".
 * 'volatile' = tell the compiler these can change behind its back (in the
 * interrupt), so it must re-read them from memory every time. */
#define RX_BUF_SIZE 256u
static volatile uint8_t s_rx_buf[RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;
static volatile uint16_t s_rx_tail = 0;
static volatile uint32_t s_overruns = 0;
static volatile uint32_t s_dropped  = 0;

static uint32_t s_cycles_per_us = 150u;

/* ---- interrupt handler ----
 * Called by the SDK's LP_FLEXCOMM2 dispatcher (fsl_lpflexcomm.c) whenever
 * the UART raises its RX interrupt. We registered it in dxl_port_init().
 * Keep it SHORT: just move bytes from the 8-byte hardware FIFO into RAM. */
static void dxl_uart_isr(uint32_t instance, void *handle)
{
    (void)instance;
    (void)handle;

    uint32_t flags = LPUART_GetStatusFlags(DXL_UART);
    if ((flags & (uint32_t)kLPUART_RxOverrunFlag) != 0u)
    {
        /* A byte arrived while the FIFO was full: it is lost. The UART stops
         * receiving until this flag is cleared, so always clear it. */
        (void)LPUART_ClearStatusFlags(DXL_UART, (uint32_t)kLPUART_RxOverrunFlag);
        s_overruns++;
    }

    while (LPUART_GetRxFifoCount(DXL_UART) > 0u)
    {
        uint8_t b      = LPUART_ReadByte(DXL_UART);
        uint16_t next  = (uint16_t)((s_rx_head + 1u) & (RX_BUF_SIZE - 1u));
        if (next == s_rx_tail)
        {
            s_dropped++; /* software buffer full */
        }
        else
        {
            s_rx_buf[s_rx_head] = b;
            s_rx_head           = next;
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

static inline void tx_en(bool high)
{
    GPIO_PinWrite(TXEN_GPIO, TXEN_PIN, high ? 1u : 0u);
}

bool dxl_port_init(uint32_t baud)
{
    /* 1) Cycle counter for microsecond timing (Cortex-M33 DWT unit).
     *    It counts CPU clock cycles: 150 per microsecond at 150 MHz. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cycles_per_us = SystemCoreClock / 1000000u;

    /* 2) TX_EN pin: P0_29 as a plain GPIO output, starting LOW (= listen).
     *    LOW = listen is the safe default; the PCB's 10k pull-down does the
     *    same thing in hardware (Data path doc, R5). */
    CLOCK_EnableClock(kCLOCK_Port0);
    CLOCK_EnableClock(kCLOCK_Gpio0);
    PORT_SetPinMux(TXEN_PORT, TXEN_PIN, kPORT_MuxAlt0); /* Alt0 = GPIO */
    TXEN_PORT->PCR[TXEN_PIN] |= PORT_PCR_IBE_MASK;      /* input buffer on, so we can read it back */
    gpio_pin_config_t out_cfg = {kGPIO_DigitalOutput, 0u};
    GPIO_PinInit(TXEN_GPIO, TXEN_PIN, &out_cfg);

    /* 3) The UART itself. 8 data bits, no parity, 1 stop bit = "8N1",
     *    which is what DYNAMIXEL uses (xl430/xl330 e-Manual, Communication). */
    lpuart_config_t cfg;
    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps    = baud;
    cfg.enableTx        = true;
    cfg.enableRx        = true;
    cfg.rxFifoWatermark = 0u; /* interrupt as soon as 1 byte is waiting */
    status_t st         = LPUART_Init(DXL_UART, &cfg, DXL_UART_CLK_HZ);
    if (st != kStatus_Success)
    {
        return false; /* e.g. baud error > 3% (fsl_lpuart.c ~line 332) */
    }

    /* 4) Hook our ISR into the SDK dispatcher, then turn the interrupt on. */
    LP_FLEXCOMM_SetIRQHandler(DXL_FLEXCOMM_INST, dxl_uart_isr, NULL, LP_FLEXCOMM_PERIPH_LPUART);
    LPUART_EnableInterrupts(DXL_UART,
                            (uint32_t)kLPUART_RxDataRegFullInterruptEnable | (uint32_t)kLPUART_RxOverrunInterruptEnable);
    (void)EnableIRQ(DXL_UART_IRQ);
    return true;
}

bool dxl_port_send(const uint8_t *data, uint16_t len)
{
    tx_en(true);
    /* LPUART_WriteBlocking waits for TDRE before each byte, then at the end
     * waits for TC = "transmission complete" = the last stop bit has left the
     * pin (see the end of LPUART_WriteBlocking in fsl_lpuart.c).
     * That is exactly when TX_EN may drop. Dropping it on TDRE instead would
     * cut off the last byte (FC2 personas doc, Direction). */
    status_t st = LPUART_WriteBlocking(DXL_UART, data, len);
    tx_en(false);
    return st == kStatus_Success;
}

bool dxl_port_read_byte(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return false; /* empty */
    }
    *byte     = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1u) & (RX_BUF_SIZE - 1u));
    return true;
}

void dxl_port_flush_rx(void)
{
    s_rx_tail = s_rx_head;
}

uint32_t dxl_port_now(void)
{
    return DWT->CYCCNT;
}

uint32_t dxl_port_elapsed_us(uint32_t start)
{
    /* Unsigned subtraction still gives the right answer when the 32-bit
     * counter wraps (every ~28 s at 150 MHz), as long as the interval is
     * shorter than one wrap. */
    return (DWT->CYCCNT - start) / s_cycles_per_us;
}

uint32_t dxl_port_overruns(void)
{
    return s_overruns;
}

uint32_t dxl_port_dropped(void)
{
    return s_dropped;
}

void dxl_port_force_tx_en(bool high)
{
    tx_en(high);
}
