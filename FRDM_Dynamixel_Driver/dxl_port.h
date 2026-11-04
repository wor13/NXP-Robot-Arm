/*
 * dxl_port.h  -  the HARDWARE side of the servo bus (FRDM-MCXN947).
 *
 * Everything that touches a pin, a UART register or a timer lives here,
 * so the rest of the driver doesn't care which board it runs on.
 * (Same split as ROBOTIS DynamixelSDK: PortHandler = this file,
 *  PacketHandler = dxl_protocol + dxl. See 05_Firmware/reference/DynamixelSDK.)
 *
 * Pins (Option B, decided 10/17; FC2 in "LPUART + LPI2C" persona):
 *   TXD    = P4_2 = FC2_P2 -> FRDM J1 pin 4  (UM12018 Table 17, p.21)
 *   RXD    = P4_3 = FC2_P3 -> FRDM J1 pin 2  (UM12018 Table 17, p.21)
 *   TX_EN  = P0_29 (GPIO)  -> FRDM J1 pin 6  (UM12018 Table 17, p.21)
 * The pin muxing for P4_2/P4_3 is done by the SDK example's pin_mux.c.
 * TX_EN is set up in dxl_port_init() below.
 */
#ifndef DXL_PORT_H_
#define DXL_PORT_H_

#include <stdint.h>
#include <stdbool.h>

/* Start LPUART2 at 'baud' with a receive interrupt, set up TX_EN and the
 * microsecond timer. Call AFTER LP_FLEXCOMM_Init(2, ...LPI2CAndLPUART). */
bool dxl_port_init(uint32_t baud);

/* Send bytes: raise TX_EN, push the bytes out, wait until the LAST STOP BIT
 * has left the pin, drop TX_EN. Returns false on a UART error. */
bool dxl_port_send(const uint8_t *data, uint16_t len);

/* Take one received byte out of the receive buffer. false = nothing there. */
bool dxl_port_read_byte(uint8_t *byte);

/* Throw away anything still sitting in the receive buffer. */
void dxl_port_flush_rx(void);

/* Microsecond timing based on the CPU cycle counter. Usage:
 *   uint32_t t0 = dxl_port_now();  ...  uint32_t us = dxl_port_elapsed_us(t0); */
uint32_t dxl_port_now(void);
uint32_t dxl_port_elapsed_us(uint32_t start);

/* Diagnostics: bytes lost because the hardware FIFO overflowed (overrun),
 * or because our software ring buffer was full. Both should stay 0. */
uint32_t dxl_port_overruns(void);
uint32_t dxl_port_dropped(void);

/* Test helper: hold TX_EN at a level so you can check J1-6 with a multimeter. */
void dxl_port_force_tx_en(bool high);

#endif /* DXL_PORT_H_ */
