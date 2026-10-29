#ifndef APP_VL53L0X_TYPES_H
#define APP_VL53L0X_TYPES_H
#include <stdint.h> /* Fixed-width uint8_t, uint16_t, uint32_t, int8_t, etc. */
#include <stddef.h> /* size_t and NULL. */
/* ST represents some fractional values as integers: value / 65536.0.
 * The upper 16 bits hold the integer part; the lower 16 hold the fraction.
 * For example 1.5 is stored as 98304. This avoids floating-point arithmetic.
 */
typedef uint32_t FixPoint1616_t;
#endif
