/*
 * dxl_protocol.c  -  DYNAMIXEL Protocol 2.0 packets: CRC, build, parse.
 * Pure logic. No hardware. See dxl_protocol.h for sources.
 */
#include "dxl_protocol.h"
#include <string.h>

/* ---------------------------------------------------------------------------
 * CRC
 * Copied from the CRC doc p.1-2 (update_crc). The table is the doc's table,
 * checked entry by entry against polynomial 0x8005.
 * Idea: a CRC is the remainder of a polynomial division of the packet bytes.
 * The table holds the pre-computed remainder for every possible byte, so the
 * loop does one lookup per byte instead of 8 shift/XOR steps.
 * ------------------------------------------------------------------------- */
static const uint16_t s_crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202

};

uint16_t dxl_crc(uint16_t crc_accum, const uint8_t *data, uint16_t size)
{
    for (uint16_t j = 0; j < size; j++)
    {
        uint16_t i = (uint16_t)(((crc_accum >> 8) ^ data[j]) & 0xFFu);
        crc_accum  = (uint16_t)((crc_accum << 8) ^ s_crc_table[i]);
    }
    return crc_accum;
}

/* ---------------------------------------------------------------------------
 * Building packets
 *
 *  byte:  0    1    2    3    4    5      6      7      8 ...       last two
 *        FF   FF   FD   00   ID   LEN_L  LEN_H  INST   PARAMS...   CRC_L CRC_H
 *
 *  LEN = (number of params) + 3   -> counts INST + params + 2 CRC bytes (P2 p.6)
 *  Total bytes on the wire = 7 + LEN = params + 10
 * ------------------------------------------------------------------------- */

/* Byte stuffing (P2 p.10): if FF FF FD ever appears inside the data, the
 * sender must insert an extra FD so the receiver doesn't mistake it for a new
 * header. Our D1 data (positions 0-4095, small addresses) never makes that
 * pattern, so instead of stuffing we REFUSE to send such a packet.
 * That way the bug can never be silent. TODO D2: implement stuffing. */
static bool needs_stuffing(const uint8_t *pkt, uint16_t start, uint16_t end)
{
    for (uint16_t i = start; i + 2u < end; i++)
    {
        if (pkt[i] == 0xFFu && pkt[i + 1u] == 0xFFu && pkt[i + 2u] == 0xFDu)
        {
            return true;
        }
    }
    return false;
}

/* Shared builder: 'first' is an optional extra byte (the ERR byte of a status
 * packet) that goes right after INST, before the params. */
static uint16_t build(uint8_t id, uint8_t instr, bool has_first, uint8_t first, const uint8_t *params,
                      uint16_t n_params, uint8_t *out, uint16_t out_size)
{
    uint16_t body   = (uint16_t)(n_params + (has_first ? 1u : 0u)); /* bytes after INST, before CRC */
    uint16_t length = (uint16_t)(body + 3u);                        /* LEN field */
    uint16_t total  = (uint16_t)(7u + length);

    if (total > out_size || body > DXL_MAX_PARAMS + 1u)
    {
        return 0; /* doesn't fit */
    }

    out[0] = 0xFF; /* header */
    out[1] = 0xFF;
    out[2] = 0xFD;
    out[3] = 0x00; /* reserved */
    out[4] = id;
    dxl_put_u16(&out[5], length); /* little-endian: low byte first */
    out[7] = instr;

    uint16_t k = 8;
    if (has_first)
    {
        out[k++] = first;
    }
    if (n_params > 0u)
    {
        memcpy(&out[k], params, n_params);
        k = (uint16_t)(k + n_params);
    }

    if (needs_stuffing(out, 4u, k))
    {
        return 0;
    }

    /* CRC covers everything from the first FF up to the last param.
     * That's k bytes = 5 + LEN (CRC doc p.2: "data_blk_size = 5 + Packet Length"). */
    uint16_t crc = dxl_crc(0, out, k);
    out[k++]     = (uint8_t)(crc & 0xFFu); /* CRC_L */
    out[k++]     = (uint8_t)(crc >> 8);    /* CRC_H */

    return k; /* equals total */
}

uint16_t dxl_build_instruction(uint8_t id, uint8_t instr, const uint8_t *params, uint16_t n_params,
                               uint8_t *out, uint16_t out_size)
{
    return build(id, instr, false, 0, params, n_params, out, out_size);
}

uint16_t dxl_build_status(uint8_t id, uint8_t error, const uint8_t *params, uint16_t n_params, uint8_t *out,
                          uint16_t out_size)
{
    return build(id, DXL_INST_STATUS, true, error, params, n_params, out, out_size);
}

/* ---------------------------------------------------------------------------
 * Parsing, one byte at a time  (receive steps from P2 p.10-11)
 *
 *  state 0: slide the last 4 bytes along until they equal FF FF FD 00
 *  state 1: collect ID, LEN_L, LEN_H  (buf[4..6])
 *  state 2: collect LEN more bytes (INST, [ERR], params, CRC_L, CRC_H)
 *           then check the CRC and unpack.
 * ------------------------------------------------------------------------- */
void dxl_parser_reset(dxl_parser_t *p)
{
    p->state  = 0;
    p->last4  = 0;
    p->idx    = 0;
    p->length = 0;
}

dxl_parse_result_t dxl_parser_feed(dxl_parser_t *p, uint8_t byte, dxl_packet_t *out)
{
    switch (p->state)
    {
        case 0: /* hunting for header */
            p->last4 = (p->last4 << 8) | byte;
            if (p->last4 == 0xFFFFFD00u)
            {
                p->buf[0] = 0xFF;
                p->buf[1] = 0xFF;
                p->buf[2] = 0xFD;
                p->buf[3] = 0x00;
                p->idx    = 4;
                p->state  = 1;
            }
            return DXL_PARSE_BUSY;

        case 1: /* ID, LEN_L, LEN_H */
            p->buf[p->idx++] = byte;
            if (p->idx == 7u)
            {
                p->length = dxl_get_u16(&p->buf[5]);
                /* Smallest legal LEN is 3 (INST + 2 CRC). Largest must fit our buffer. */
                if (p->length < 3u || (uint32_t)7u + p->length > sizeof(p->buf))
                {
                    dxl_parser_reset(p);
                    return DXL_PARSE_TOO_LONG;
                }
                p->state = 2;
            }
            return DXL_PARSE_BUSY;

        default: /* state 2: the rest of the packet */
            p->buf[p->idx++] = byte;
            if (p->idx < (uint16_t)(7u + p->length))
            {
                return DXL_PARSE_BUSY;
            }
            break; /* whole packet is in */
    }

    /* ---- whole packet received: check CRC ---- */
    uint16_t n_before_crc = (uint16_t)(p->idx - 2u); /* = 5 + LEN */
    uint16_t crc_calc     = dxl_crc(0, p->buf, n_before_crc);
    uint16_t crc_rx       = dxl_get_u16(&p->buf[n_before_crc]);
    if (crc_calc != crc_rx)
    {
        dxl_parser_reset(p);
        return DXL_PARSE_CRC_ERROR;
    }

    /* ---- unpack ---- */
    out->id    = p->buf[4];
    out->instr = p->buf[7];
    uint16_t first_param;
    if (out->instr == DXL_INST_STATUS)
    {
        /* status packet: INST, ERR, params...  -> params = LEN - 4 */
        if (p->length < 4u)
        {
            dxl_parser_reset(p);
            return DXL_PARSE_TOO_LONG;
        }
        out->error    = p->buf[8];
        first_param   = 9;
        out->n_params = (uint16_t)(p->length - 4u);
    }
    else
    {
        /* instruction packet: INST, params...  -> params = LEN - 3 */
        out->error    = 0;
        first_param   = 8;
        out->n_params = (uint16_t)(p->length - 3u);
    }
    if (out->n_params > DXL_MAX_PARAMS)
    {
        dxl_parser_reset(p);
        return DXL_PARSE_TOO_LONG;
    }
    memcpy(out->params, &p->buf[first_param], out->n_params);

    dxl_parser_reset(p);
    return DXL_PARSE_DONE;
}
