/*
 * fake_servo.c  -  a pretend DYNAMIXEL on LPUART1. TEST ONLY.
 *
 * Everything happens inside the UART interrupt:
 *   RX: each received byte goes into the SAME parser the driver uses.
 *       When a full packet for our ID is in, we build the reply.
 *   TX: the reply is sent by the "transmit FIFO empty" interrupt, a few
 *       bytes at a time, so the interrupt never sits and waits.
 *       (If we waited inside the interrupt, the driver's own receive
 *        interrupt could be blocked and lose bytes.)
 *
 * Is it OK that the fake uses the same parser/builder as the driver?
 * Only because main.c first checks those against the ROBOTIS manual's own
 * example bytes. Without that check, a CRC bug would be on both sides and
 * cancel out.
 */
#include "fake_servo.h"
#include "dxl_protocol.h"

#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include "fsl_lpuart.h"
#include "fsl_lpflexcomm.h"
#include <string.h>

#define FAKE_UART          LPUART1
#define FAKE_FLEXCOMM_INST 1u
#define FAKE_UART_IRQ      LP_FLEXCOMM1_IRQn
#define FAKE_UART_CLK_HZ   CLOCK_GetLPFlexCommClkFreq(1u)
#define FAKE_TX_FIFO_SIZE  8u /* FSL_FEATURE_LPUART_FIFO_SIZEn = 8 for this chip */

/* STEP 2 (bus test) change 1: wait a little before answering, like a real
 * servo's Return Delay Time (a real one waits 500 us by default).
 * Why: on the breadboard bus the driver's TX_EN must drop BEFORE the reply
 * starts, or the first bits of the reply are lost in the switched-off buffer.
 * 20 us is plenty (the driver drops TX_EN about 1 us after its last stop bit). */
#define FAKE_RETURN_DELAY_US 20u

/* A pretend control table, 256 bytes is enough for the addresses we use. */
static uint8_t s_table[256];
static uint8_t s_id;
static volatile bool s_silent;
static volatile uint32_t s_packets_seen;
static volatile uint32_t s_crc_errors;

static dxl_parser_t s_parser;
static dxl_packet_t s_rx_pkt;

static uint8_t s_tx_buf[DXL_MAX_PACKET];
static volatile uint16_t s_tx_len;
static volatile uint16_t s_tx_idx;

/* Start sending s_tx_buf: just switch on the TX-empty interrupt.
 * The ISR below then refills the FIFO until everything is out. */
static void start_reply(uint8_t error, const uint8_t *params, uint16_t n)
{
    uint16_t len = dxl_build_status(s_id, error, params, n, s_tx_buf, sizeof(s_tx_buf));
    if (len == 0u)
    {
        return;
    }
    SDK_DelayAtLeastUs(FAKE_RETURN_DELAY_US, SystemCoreClock); /* step 2 change 1 */
    s_tx_idx = 0;
    s_tx_len = len;
    LPUART_EnableInterrupts(FAKE_UART, (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable);
}

/* What a servo does with each instruction (P2 p.11-28). */
static void handle_packet(const dxl_packet_t *pkt)
{
    s_packets_seen++;
    if (s_silent)
    {
        return;
    }

    /* STEP 2 (bus test) change 2: ignore status packets (replies).
     * On the one-wire bus the fake servo hears its OWN reply (its receive
     * buffer is always on). Without this line it would answer its own reply
     * with an "instruction error", hear that, answer again... forever.
     * Real servos also ignore status packets. */
    if (pkt->instr == DXL_INST_STATUS)
    {
        return;
    }

    /* Sync Write goes to broadcast; we look for our ID inside it. */
    if (pkt->instr == DXL_INST_SYNC_WRITE && pkt->id == DXL_BROADCAST_ID)
    {
        if (pkt->n_params < 4u)
        {
            return;
        }
        uint16_t addr = dxl_get_u16(&pkt->params[0]);
        uint16_t len  = dxl_get_u16(&pkt->params[2]);
        for (uint16_t k = 4; (uint32_t)k + 1u + len <= pkt->n_params; k = (uint16_t)(k + 1u + len))
        {
            if (pkt->params[k] == s_id && (uint32_t)addr + len <= sizeof(s_table))
            {
                memcpy(&s_table[addr], &pkt->params[k + 1u], len);
            }
        }
        return; /* broadcast: never reply */
    }

    if (pkt->id != s_id && !(pkt->id == DXL_BROADCAST_ID && pkt->instr == DXL_INST_PING))
    {
        return; /* not for us */
    }

    switch (pkt->instr)
    {
        case DXL_INST_PING:
        {
            /* Reply: model L, model H, firmware (P2 p.12) */
            uint8_t p[3] = {s_table[0], s_table[1], s_table[6]};
            start_reply(0, p, 3);
            break;
        }
        case DXL_INST_READ:
        {
            if (pkt->n_params != 4u)
            {
                start_reply(DXL_ERRNUM_DATA_LENGTH, NULL, 0);
                break;
            }
            uint16_t addr = dxl_get_u16(&pkt->params[0]);
            uint16_t len  = dxl_get_u16(&pkt->params[2]);
            if ((uint32_t)addr + len > sizeof(s_table) || len > DXL_MAX_PARAMS)
            {
                start_reply(DXL_ERRNUM_ACCESS, NULL, 0); /* P2 p.14 note: access error, no params */
                break;
            }
            start_reply(0, &s_table[addr], len);
            break;
        }
        case DXL_INST_WRITE:
        {
            if (pkt->n_params < 3u)
            {
                start_reply(DXL_ERRNUM_DATA_LENGTH, NULL, 0);
                break;
            }
            uint16_t addr = dxl_get_u16(&pkt->params[0]);
            uint16_t len  = (uint16_t)(pkt->n_params - 2u);
            if ((uint32_t)addr + len > sizeof(s_table))
            {
                start_reply(DXL_ERRNUM_ACCESS, NULL, 0);
                break;
            }
            memcpy(&s_table[addr], &pkt->params[2], len);
            if (pkt->id != DXL_BROADCAST_ID)
            {
                start_reply(0, NULL, 0); /* P2 p.16: write reply has no params */
            }
            break;
        }
        default:
            start_reply(DXL_ERRNUM_INSTRUCTION, NULL, 0); /* unknown instruction (P2 p.9) */
            break;
    }
}

static void fake_isr(uint32_t instance, void *handle)
{
    (void)instance;
    (void)handle;

    uint32_t flags = LPUART_GetStatusFlags(FAKE_UART);
    if ((flags & (uint32_t)kLPUART_RxOverrunFlag) != 0u)
    {
        (void)LPUART_ClearStatusFlags(FAKE_UART, (uint32_t)kLPUART_RxOverrunFlag);
    }

    /* ---- receive side ---- */
    while (LPUART_GetRxFifoCount(FAKE_UART) > 0u)
    {
        uint8_t b             = LPUART_ReadByte(FAKE_UART);
        dxl_parse_result_t pr = dxl_parser_feed(&s_parser, b, &s_rx_pkt);
        if (pr == DXL_PARSE_DONE)
        {
            handle_packet(&s_rx_pkt);
        }
        else if (pr == DXL_PARSE_CRC_ERROR)
        {
            s_crc_errors++; /* a real servo would reply with ERR 0x03, but stay simple */
        }
    }

    /* ---- transmit side ---- */
    if ((LPUART_GetEnabledInterrupts(FAKE_UART) & (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable) != 0u)
    {
        while (s_tx_idx < s_tx_len && LPUART_GetTxFifoCount(FAKE_UART) < FAKE_TX_FIFO_SIZE)
        {
            LPUART_WriteByte(FAKE_UART, s_tx_buf[s_tx_idx]);
            s_tx_idx++;
        }
        if (s_tx_idx >= s_tx_len)
        {
            /* All bytes are in the FIFO; stop asking for TX-empty interrupts
             * or this ISR would fire forever. */
            LPUART_DisableInterrupts(FAKE_UART, (uint32_t)kLPUART_TxDataRegEmptyInterruptEnable);
        }
    }
    SDK_ISR_EXIT_BARRIER;
}

bool fake_servo_init(uint8_t id, uint32_t baud)
{
    memset(s_table, 0, sizeof(s_table));
    s_id = id;
    /* Same numbers as the manual's Ping example (P2 p.12): model 1030 = 0x0406,
     * firmware 38 = 0x26. Then our reply bytes must equal the manual's exactly.
     * (A real XL430-W250 reports model 1060 - xl430 doc, Control Table.) */
    dxl_put_u16(&s_table[0], 1030u);
    s_table[6] = 38u;
    s_table[7] = id;
    s_table[9] = 250u;                  /* Return Delay Time default */
    dxl_put_u32(&s_table[132], 166u);   /* Present Position = 166, P2 p.14 Read example */
    dxl_parser_reset(&s_parser);
    s_silent = false;

    lpuart_config_t cfg;
    LPUART_GetDefaultConfig(&cfg);
    cfg.baudRate_Bps    = baud;
    cfg.enableTx        = true;
    cfg.enableRx        = true;
    cfg.rxFifoWatermark = 0u;
    if (LPUART_Init(FAKE_UART, &cfg, FAKE_UART_CLK_HZ) != kStatus_Success)
    {
        return false;
    }
    LP_FLEXCOMM_SetIRQHandler(FAKE_FLEXCOMM_INST, fake_isr, NULL, LP_FLEXCOMM_PERIPH_LPUART);
    LPUART_EnableInterrupts(FAKE_UART,
                            (uint32_t)kLPUART_RxDataRegFullInterruptEnable | (uint32_t)kLPUART_RxOverrunInterruptEnable);
    (void)EnableIRQ(FAKE_UART_IRQ);
    return true;
}

void fake_servo_set_silent(bool silent)
{
    s_silent = silent;
}

void fake_servo_set_present_position(uint32_t pos)
{
    dxl_put_u32(&s_table[132], pos);
}

uint32_t fake_servo_get_goal_position(void)
{
    return dxl_get_u32(&s_table[116]);
}

uint32_t fake_servo_packets_seen(void)
{
    return s_packets_seen;
}

uint32_t fake_servo_crc_errors(void)
{
    return s_crc_errors;
}
