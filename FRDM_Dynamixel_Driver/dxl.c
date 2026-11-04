/*
 * dxl.c  -  servo transactions on top of dxl_protocol (bytes) and dxl_port (hardware).
 */
#include "dxl.h"
#include "dxl_port.h"
#include <string.h>

static dxl_stats_t s_stats;
static dxl_parser_t s_parser;
static uint8_t s_tx[DXL_MAX_PACKET];

void dxl_init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    dxl_parser_reset(&s_parser);
}

/* Book-keeping for the bus-fault counter (N = 3). */
static void note_result(dxl_result_t r)
{
    s_stats.transactions++;
    if (r == DXL_OK || r == DXL_ERR_SERVO)
    {
        /* DXL_ERR_SERVO still means the bus WORKS (a valid reply came back). */
        s_stats.ok++;
        s_stats.consecutive_fails = 0;
        return;
    }
    if (r == DXL_ERR_TIMEOUT)
    {
        s_stats.timeouts++;
    }
    else if (r == DXL_ERR_CRC)
    {
        s_stats.crc_errors++;
    }
    else if (r == DXL_ERR_BAD_REPLY)
    {
        s_stats.bad_replies++;
    }
    if (s_stats.consecutive_fails < 255u)
    {
        s_stats.consecutive_fails++;
    }
    if (s_stats.consecutive_fails >= DXL_BUS_FAULT_LIMIT)
    {
        s_stats.bus_fault = true; /* stays true until dxl_clear_bus_fault() */
    }
}

/* The core of every call:
 *   1. build the packet   2. empty the receive buffer   3. send
 *   4. if a reply is expected: feed received bytes to the parser until a
 *      packet is complete or the timeout runs out
 *   5. check the reply is a status packet from the ID we asked */
static dxl_result_t transact(uint8_t id, uint8_t instr, const uint8_t *params, uint16_t n_params,
                             bool expect_reply, dxl_packet_t *reply)
{
    uint16_t n = dxl_build_instruction(id, instr, params, n_params, s_tx, sizeof(s_tx));
    if (n == 0u)
    {
        return DXL_ERR_BUILD; /* not counted as a bus failure: it's our bug */
    }

    dxl_port_flush_rx();          /* old junk must not be mistaken for the reply */
    dxl_parser_reset(&s_parser);

    uint32_t t0 = dxl_port_now();
    if (!dxl_port_send(s_tx, n))
    {
        note_result(DXL_ERR_TX);
        return DXL_ERR_TX;
    }
    if (!expect_reply)
    {
        note_result(DXL_OK);
        return DXL_OK;
    }

    /* Wait for the reply. Timeout counts from the END of our transmission. */
    uint32_t t_sent = dxl_port_now();
    dxl_result_t r  = DXL_ERR_TIMEOUT;
    while (dxl_port_elapsed_us(t_sent) < DXL_REPLY_TIMEOUT_US)
    {
        uint8_t b;
        if (!dxl_port_read_byte(&b))
        {
            continue; /* nothing yet, keep waiting */
        }
        dxl_parse_result_t pr = dxl_parser_feed(&s_parser, b, reply);
        if (pr == DXL_PARSE_BUSY)
        {
            continue;
        }
        if (pr == DXL_PARSE_CRC_ERROR)
        {
            r = DXL_ERR_CRC;
        }
        else if (pr == DXL_PARSE_TOO_LONG)
        {
            r = DXL_ERR_BAD_REPLY;
        }
        else /* DXL_PARSE_DONE */
        {
            if (reply->instr != DXL_INST_STATUS || reply->id != id)
            {
                r = DXL_ERR_BAD_REPLY;
            }
            else
            {
                s_stats.last_rtt_us      = dxl_port_elapsed_us(t0);
                s_stats.last_servo_error = reply->error;
                r                        = (reply->error == 0u) ? DXL_OK : DXL_ERR_SERVO;
            }
        }
        break;
    }

    note_result(r);
    return r;
}

dxl_result_t dxl_ping(uint8_t id, uint16_t *model, uint8_t *firmware)
{
    dxl_packet_t rep;
    /* Ping has no params. Reply params: model L, model H, firmware (P2 p.12) */
    dxl_result_t r = transact(id, DXL_INST_PING, NULL, 0, true, &rep);
    if (r == DXL_OK)
    {
        if (rep.n_params < 3u)
        {
            return DXL_ERR_BAD_REPLY;
        }
        if (model != NULL)
        {
            *model = dxl_get_u16(&rep.params[0]);
        }
        if (firmware != NULL)
        {
            *firmware = rep.params[2];
        }
    }
    return r;
}

dxl_result_t dxl_read(uint8_t id, uint16_t addr, uint16_t len, uint8_t *data)
{
    /* Read params: addr L, addr H, len L, len H (P2 p.14) */
    uint8_t p[4];
    dxl_put_u16(&p[0], addr);
    dxl_put_u16(&p[2], len);

    dxl_packet_t rep;
    dxl_result_t r = transact(id, DXL_INST_READ, p, 4, true, &rep);
    if (r == DXL_OK)
    {
        if (rep.n_params != len)
        {
            return DXL_ERR_BAD_REPLY;
        }
        memcpy(data, rep.params, len);
    }
    return r;
}

dxl_result_t dxl_write(uint8_t id, uint16_t addr, const uint8_t *data, uint16_t len)
{
    /* Write params: addr L, addr H, data... (P2 p.15-16) */
    uint8_t p[DXL_MAX_PARAMS];
    if ((uint32_t)len + 2u > sizeof(p))
    {
        return DXL_ERR_BUILD;
    }
    dxl_put_u16(&p[0], addr);
    memcpy(&p[2], data, len);

    dxl_packet_t rep;
    /* A broadcast write gets no reply (P2 p.5 warning). */
    bool expect_reply = (id != DXL_BROADCAST_ID);
    return transact(id, DXL_INST_WRITE, p, (uint16_t)(len + 2u), expect_reply, &rep);
}

dxl_result_t dxl_read_u32(uint8_t id, uint16_t addr, uint32_t *value)
{
    uint8_t b[4];
    dxl_result_t r = dxl_read(id, addr, 4, b);
    if (r == DXL_OK)
    {
        *value = dxl_get_u32(b);
    }
    return r;
}

dxl_result_t dxl_write_u32(uint8_t id, uint16_t addr, uint32_t value)
{
    uint8_t b[4];
    dxl_put_u32(b, value);
    return dxl_write(id, addr, b, 4);
}

dxl_result_t dxl_sync_write_u32(uint16_t addr, const uint8_t *ids, const uint32_t *values, uint8_t count)
{
    /* Sync Write params (P2 p.26): addr L, addr H, len L, len H,
     * then for each servo: ID, data(4 bytes). Sent to broadcast ID 0xFE. */
    uint8_t p[DXL_MAX_PARAMS];
    uint16_t k = 0;
    if (4u + (uint32_t)count * 5u > sizeof(p))
    {
        return DXL_ERR_BUILD;
    }
    dxl_put_u16(&p[k], addr);
    k = (uint16_t)(k + 2u);
    dxl_put_u16(&p[k], 4u);
    k = (uint16_t)(k + 2u);
    for (uint8_t i = 0; i < count; i++)
    {
        p[k++] = ids[i];
        dxl_put_u32(&p[k], values[i]);
        k = (uint16_t)(k + 4u);
    }
    dxl_packet_t unused;
    return transact(DXL_BROADCAST_ID, DXL_INST_SYNC_WRITE, p, k, false, &unused);
}

const dxl_stats_t *dxl_get_stats(void)
{
    return &s_stats;
}

void dxl_clear_bus_fault(void)
{
    s_stats.bus_fault         = false;
    s_stats.consecutive_fails = 0;
}

const char *dxl_result_str(dxl_result_t r)
{
    switch (r)
    {
        case DXL_OK:            return "OK";
        case DXL_ERR_TX:        return "TX_ERROR";
        case DXL_ERR_BUILD:     return "BUILD_ERROR";
        case DXL_ERR_TIMEOUT:   return "TIMEOUT";
        case DXL_ERR_CRC:       return "CRC_ERROR";
        case DXL_ERR_BAD_REPLY: return "BAD_REPLY";
        case DXL_ERR_SERVO:     return "SERVO_ERROR";
        default:                return "?";
    }
}
