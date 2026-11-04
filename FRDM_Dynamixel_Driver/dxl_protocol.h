/*
 * dxl_protocol.h  -  DYNAMIXEL Protocol 2.0 packets: CRC, build, parse.
 *
 * This layer is PURE LOGIC: no UART, no pins, no timers.
 * It only turns numbers into bytes and bytes back into numbers.
 * Because of that, it can be tested against the ROBOTIS manual's
 * example packets before any hardware is involved.
 *
 * Source for every format detail:
 *   02_Datasheets/ROBOTIS/DYNAMIXEL Protocol 2.0 _ ROBOTIS Docs.pdf  (called "P2" below)
 *   02_Datasheets/ROBOTIS/CRC-16 (IBM_ANSI) _ ROBOTIS Docs.pdf       (called "CRC doc")
 */
#ifndef DXL_PROTOCOL_H_
#define DXL_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

/* Instruction codes (P2 p.6-7, Instruction table) */
#define DXL_INST_PING       0x01u
#define DXL_INST_READ       0x02u
#define DXL_INST_WRITE      0x03u
#define DXL_INST_STATUS     0x55u /* every reply from a servo uses this */
#define DXL_INST_SYNC_READ  0x82u
#define DXL_INST_SYNC_WRITE 0x83u

/* IDs (P2 p.5): 0..252 are servos, 254 = broadcast (everyone listens) */
#define DXL_BROADCAST_ID 0xFEu

/* Servo error numbers in the ERR byte of a status packet (P2 p.9).
 * Bit 7 (0x80) is the "Alert" bit = hardware problem inside the servo (P2 p.8). */
#define DXL_ERRNUM_RESULT_FAIL  0x01u
#define DXL_ERRNUM_INSTRUCTION  0x02u
#define DXL_ERRNUM_CRC          0x03u
#define DXL_ERRNUM_DATA_RANGE   0x04u
#define DXL_ERRNUM_DATA_LENGTH  0x05u
#define DXL_ERRNUM_DATA_LIMIT   0x06u
#define DXL_ERRNUM_ACCESS       0x07u
#define DXL_ERR_ALERT_BIT       0x80u

/* Size limits. We never need big packets for 6 servos, so keep it small.
 * Full packet = 4 header + 1 ID + 2 LEN + 1 INST + 1 ERR + params + 2 CRC
 *             = params + 11 bytes (worst case, a status packet). */
#define DXL_MAX_PARAMS 64u
#define DXL_MAX_PACKET (DXL_MAX_PARAMS + 11u)

/* One decoded packet (instruction OR status). */
typedef struct
{
    uint8_t id;
    uint8_t instr;     /* 0x01 Ping, 0x02 Read, ... or 0x55 for a status packet */
    uint8_t error;     /* only meaningful when instr == 0x55 */
    uint16_t n_params; /* number of bytes in params[] */
    uint8_t params[DXL_MAX_PARAMS];
} dxl_packet_t;

/* CRC-16 (IBM/ANSI), polynomial 0x8005, start value 0. Copied from the CRC doc p.1-2. */
uint16_t dxl_crc(uint16_t crc_accum, const uint8_t *data, uint16_t size);

/* Build an instruction packet into out[].
 * Returns the total number of bytes to send, or 0 if it does not fit
 * or if the data would need byte stuffing (not supported yet, see .c). */
uint16_t dxl_build_instruction(uint8_t id, uint8_t instr, const uint8_t *params, uint16_t n_params,
                               uint8_t *out, uint16_t out_size);

/* Build a status (reply) packet. Used only by the fake servo. */
uint16_t dxl_build_status(uint8_t id, uint8_t error, const uint8_t *params, uint16_t n_params,
                          uint8_t *out, uint16_t out_size);

/* ---- Receiving: a byte-at-a-time parser ----
 * Bytes arrive one at a time from the UART. Feed each one in.
 * The parser answers BUSY until a whole packet is in, then DONE (or an error). */
typedef enum
{
    DXL_PARSE_BUSY = 0,  /* need more bytes */
    DXL_PARSE_DONE,      /* *out holds a good packet */
    DXL_PARSE_CRC_ERROR, /* a whole packet arrived but the CRC was wrong */
    DXL_PARSE_TOO_LONG   /* LEN field is impossible or bigger than our buffer */
} dxl_parse_result_t;

typedef struct
{
    uint8_t state;     /* 0 = hunting for header, 1 = reading ID+LEN, 2 = reading the rest */
    uint32_t last4;    /* last 4 bytes seen, used to spot FF FF FD 00 */
    uint16_t idx;      /* how many bytes are in buf[] */
    uint16_t length;   /* the LEN field from the packet */
    uint8_t buf[DXL_MAX_PACKET];
} dxl_parser_t;

void dxl_parser_reset(dxl_parser_t *p);
dxl_parse_result_t dxl_parser_feed(dxl_parser_t *p, uint8_t byte, dxl_packet_t *out);

/* Little-endian helpers (DYNAMIXEL sends the LOW byte first, P2 p.5-6) */
static inline void dxl_put_u16(uint8_t *b, uint16_t v)
{
    b[0] = (uint8_t)(v & 0xFFu);
    b[1] = (uint8_t)(v >> 8);
}
static inline void dxl_put_u32(uint8_t *b, uint32_t v)
{
    b[0] = (uint8_t)(v);
    b[1] = (uint8_t)(v >> 8);
    b[2] = (uint8_t)(v >> 16);
    b[3] = (uint8_t)(v >> 24);
}
static inline uint16_t dxl_get_u16(const uint8_t *b)
{
    return (uint16_t)(b[0] | ((uint16_t)b[1] << 8));
}
static inline uint32_t dxl_get_u32(const uint8_t *b)
{
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

#endif /* DXL_PROTOCOL_H_ */
