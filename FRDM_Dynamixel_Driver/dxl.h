/*
 * dxl.h  -  the functions Thomas (and Safe Stop) will call.
 *
 * Each call = one "transaction": send one instruction packet, then (if the
 * servo is supposed to answer) wait for its status packet.
 *
 * Bus health (6.8 decision 10/18, N = 3): every failed transaction (no
 * answer / bad CRC / wrong reply) adds 1 to a counter; a good one resets it.
 * 3 failures in a row = BUS FAULT, which Safe Stop treats like a stop event.
 * There is NO hidden retry inside these functions: the 5 ms control loop
 * calling again next cycle IS the retry. That keeps timing predictable.
 */
#ifndef DXL_H_
#define DXL_H_

#include <stdint.h>
#include <stdbool.h>
#include "dxl_protocol.h"

/* Control-table addresses we use (02_Datasheets/Motors/xl430-w250 _ ROBOTIS Docs.pdf,
 * Control Table; the same addresses on the XL330). */
#define DXL_ADDR_MODEL_NUMBER      0u   /* 2 bytes, XL430-W250 = 1060 */
#define DXL_ADDR_FIRMWARE_VERSION  6u   /* 1 byte */
#define DXL_ADDR_RETURN_DELAY_TIME 9u   /* 1 byte, units of 2 us, default 250 = 500 us */
#define DXL_ADDR_TORQUE_ENABLE     64u  /* 1 byte */
#define DXL_ADDR_GOAL_POSITION     116u /* 4 bytes */
#define DXL_ADDR_PRESENT_POSITION  132u /* 4 bytes */

#define DXL_BUS_FAULT_LIMIT 3u /* N = 3 (Safe Stop decision 10/18) */

/* How long to wait for a reply. A real servo answers after its Return Delay
 * (500 us at the default) + the reply bytes (10 us each at 1 Mbps).
 * 3 ms leaves a big margin; shrink it once you measure real servos. */
#define DXL_REPLY_TIMEOUT_US 3000u

typedef enum
{
    DXL_OK = 0,
    DXL_ERR_TX,        /* UART refused to send */
    DXL_ERR_BUILD,     /* packet didn't fit, or would need byte stuffing */
    DXL_ERR_TIMEOUT,   /* no complete reply in time */
    DXL_ERR_CRC,       /* reply arrived damaged */
    DXL_ERR_BAD_REPLY, /* reply from the wrong ID, wrong type, or wrong length */
    DXL_ERR_SERVO      /* bus is fine, but the servo set its ERR byte (see last_servo_error) */
} dxl_result_t;

typedef struct
{
    uint32_t transactions;
    uint32_t ok;
    uint32_t timeouts;
    uint32_t crc_errors;
    uint32_t bad_replies;
    uint8_t consecutive_fails;
    bool bus_fault;            /* latched true after 3 fails in a row */
    uint32_t last_rtt_us;      /* send start -> reply complete, last good transaction */
    uint8_t last_servo_error;  /* ERR byte of the last status packet */
} dxl_stats_t;

void dxl_init(void); /* clears stats; call after dxl_port_init() */

dxl_result_t dxl_ping(uint8_t id, uint16_t *model, uint8_t *firmware);
dxl_result_t dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *data);
dxl_result_t dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len);

/* Convenience wrappers for 4-byte values (positions) */
dxl_result_t dxl_read_u32(uint8_t id, uint16_t addr, uint32_t *value);
dxl_result_t dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value);

/* Sync Write: same address, same length, several servos, ONE packet (P2 p.26-27).
 * Broadcast -> no reply, so it can't detect a dead servo by itself. */
dxl_result_t dxl_sync_write_u32(uint16_t addr, const uint8_t *ids, const uint32_t *values, uint8_t count);

const dxl_stats_t *dxl_get_stats(void);
void dxl_clear_bus_fault(void);
const char *dxl_result_str(dxl_result_t r);

#endif /* DXL_H_ */
