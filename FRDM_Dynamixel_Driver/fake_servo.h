/*
 * fake_servo.h  -  pretends to be ONE DYNAMIXEL servo, on LPUART1 of the same FRDM.
 *
 * TEST ONLY. Lets the real driver (LPUART2) talk to "a servo" with no arm.
 *
 * Pins (FC1 in "LPUART + LPI2C" persona, set up by the SDK example's pin_mux.c):
 *   fake servo TXD = P0_26 = FC1_P2 -> FRDM J2 pin 10 (UM12018 Table 18, p.22)
 *   fake servo RXD = P0_27 = FC1_P3 -> FRDM J2 pin 6  (UM12018 Table 18, p.22)
 *   (P0_27 is also the green LED, UM12018 Table 5 p.9 - harmless, see guide)
 *
 * Wiring:  J1-4 (driver TX) -> J2-6 (fake RX)
 *          J2-10 (fake TX)  -> J1-2 (driver RX)
 */
#ifndef FAKE_SERVO_H_
#define FAKE_SERVO_H_

#include <stdint.h>
#include <stdbool.h>

bool fake_servo_init(uint8_t id, uint32_t baud); /* call after LP_FLEXCOMM_Init(1, ...) */
void fake_servo_set_silent(bool silent);         /* true = stop answering (simulates a dead bus) */
void fake_servo_set_present_position(uint32_t pos);
uint32_t fake_servo_get_goal_position(void);
uint32_t fake_servo_packets_seen(void);
uint32_t fake_servo_crc_errors(void);

#endif /* FAKE_SERVO_H_ */
