/*
 * dxl_step1_main.c  -  Step 1 test program: servo driver vs. a fake servo.
 *
 * Built on top of the SDK example "lpflexcomm_interrupt_transfer"
 * (its pin_mux.c, hardware_init.c, app.h and reconfig.cmake are reused as-is).
 *
 * Output: COM4 at 115200 (MCU-Link VCOM = debug console on FC4).
 *
 * Wiring (USB unplugged while wiring):
 *   J1-4  (driver TX, P4_2)   ->  J2-6  (fake servo RX, P0_27)
 *   J2-10 (fake servo TX, P0_26) -> J1-2 (driver RX, P4_3)
 *
 * Pick what the program does with TEST_MODE:
 *   0 = self-test (part A, no wires needed) + live tests (part B) + 10 Hz status lines
 *   1 = TX_EN check: J1-6 high 5 s, low 5 s, repeat   (multimeter on J1-6)
 *   2 = "multimeter as oscilloscope": driver TX sends one byte value nonstop
 *       (multimeter on J1-4, see the guide for the expected voltages)
 */
#define TEST_MODE  0
#define FLOOD_BYTE 0x55u /* used by TEST_MODE 2: try 0x00, 0x55, 0xFF */

#include <string.h>
#include "board.h"
#include "app.h"
#include "fsl_debug_console.h"
#include "fsl_lpflexcomm.h"
#include "fsl_lpuart.h"

#include "dxl_protocol.h"
#include "dxl_port.h"
#include "dxl.h"
#include "fake_servo.h"

#define BUS_BAUD 1000000u /* OMX bus speed, measured 10/8 (Power_Comms_Lab_Tests.xlsx) */
#define SERVO_ID 1u       /* wrist XL330 since the 10/15 swap */

#if TEST_MODE == 0
/* Everything from here down to the matching #endif is only needed for the
 * self-tests. In TEST_MODE 1 or 2 these functions would never be called, and
 * the SDK builds with -Wall -Werror (arch/arm/configuration/common.cmake), so
 * "defined but not used" would stop the build. #if leaves them out instead. */
static uint32_t s_pass = 0, s_fail = 0;

static void check(bool ok, const char *name)
{
    PRINTF("  [%s] %s\r\n", ok ? "PASS" : "FAIL", name);
    if (ok)
    {
        s_pass++;
    }
    else
    {
        s_fail++;
    }
}

static void print_bytes(const char *label, const uint8_t *b, uint16_t n)
{
    PRINTF("    %s:", label);
    for (uint16_t i = 0; i < n; i++)
    {
        PRINTF(" %02X", b[i]);
    }
    PRINTF("\r\n");
}

static bool same(const uint8_t *a, const uint8_t *b, uint16_t n)
{
    return memcmp(a, b, n) == 0;
}

/* Feed a whole byte array to a fresh parser, return the result. */
static dxl_parse_result_t parse_all(const uint8_t *b, uint16_t n, dxl_packet_t *out)
{
    dxl_parser_t p;
    dxl_parser_reset(&p);
    dxl_parse_result_t r = DXL_PARSE_BUSY;
    for (uint16_t i = 0; i < n && r == DXL_PARSE_BUSY; i++)
    {
        r = dxl_parser_feed(&p, b[i], out);
    }
    return r;
}

/* =========================================================================
 * PART A - pure logic vs. the ROBOTIS manual. No wires, no UART.
 * Every "manual_..." array is copied from the Protocol 2.0 PDF example.
 * (The PDF's text sometimes cuts the last bytes off the table rows; look at
 *  the page itself to see them.)
 * ========================================================================= */
static void part_a(void)
{
    uint8_t out[DXL_MAX_PACKET];
    uint16_t n;
    dxl_packet_t pkt;

    PRINTF("\r\n--- Part A: packets vs. ROBOTIS manual ---\r\n");

    /* A1: Ping ID 1 (P2 p.12, Example 1). CRC must be 19 4E. */
    static const uint8_t manual_ping[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E};
    check(dxl_crc(0, manual_ping, 8) == 0x4E19u, "A1 CRC of Ping = 0x4E19 (sent as 19 4E)");

    n = dxl_build_instruction(1, DXL_INST_PING, NULL, 0, out, sizeof(out));
    print_bytes("built ", out, n);
    check(n == sizeof(manual_ping) && same(out, manual_ping, n), "A2 build Ping ID1 == manual p.12");

    /* A3: Read 4 bytes at 132 (Present Position) from ID 1 (P2 p.15) */
    static const uint8_t manual_read[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00,
                                          0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15};
    uint8_t rp[4];
    dxl_put_u16(&rp[0], 132);
    dxl_put_u16(&rp[2], 4);
    n = dxl_build_instruction(1, DXL_INST_READ, rp, 4, out, sizeof(out));
    check(n == sizeof(manual_read) && same(out, manual_read, n), "A3 build Read 132/4 == manual p.15");

    /* A4: Write 512 to Goal Position 116 on ID 1 (P2 p.16) */
    static const uint8_t manual_write[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03,
                                           0x74, 0x00, 0x00, 0x02, 0x00, 0x00, 0xCA, 0x89};
    uint8_t wp[6];
    dxl_put_u16(&wp[0], 116);
    dxl_put_u32(&wp[2], 512);
    n = dxl_build_instruction(1, DXL_INST_WRITE, wp, 6, out, sizeof(out));
    check(n == sizeof(manual_write) && same(out, manual_write, n), "A4 build Write 116=512 == manual p.16");

    /* A5: Sync Write 150 to ID1, 170 to ID2 at 116 (P2 p.27-28) */
    static const uint8_t manual_sync[] = {0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, 0x83, 0x74, 0x00, 0x04, 0x00,
                                          0x01, 0x96, 0x00, 0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00, 0x82, 0x87};
    uint8_t sp[14] = {0x74, 0x00, 0x04, 0x00, 0x01, 0, 0, 0, 0, 0x02, 0, 0, 0, 0};
    dxl_put_u32(&sp[5], 150);
    dxl_put_u32(&sp[10], 170);
    n = dxl_build_instruction(DXL_BROADCAST_ID, DXL_INST_SYNC_WRITE, sp, 14, out, sizeof(out));
    check(n == sizeof(manual_sync) && same(out, manual_sync, n), "A5 build Sync Write == manual p.28");

    /* A6: parse the manual's Ping reply: model 1030, firmware 38 (P2 p.12) */
    static const uint8_t manual_ping_status[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00,
                                                 0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D};
    dxl_parse_result_t r = parse_all(manual_ping_status, sizeof(manual_ping_status), &pkt);
    check(r == DXL_PARSE_DONE && pkt.id == 1u && pkt.instr == DXL_INST_STATUS && pkt.error == 0u &&
              pkt.n_params == 3u && dxl_get_u16(&pkt.params[0]) == 1030u && pkt.params[2] == 38u,
          "A6 parse Ping status -> model 1030, fw 38");

    /* A7: parse the manual's Read reply: Present Position = 166 (P2 p.15) */
    static const uint8_t manual_read_status[] = {0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55,
                                                 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0};
    r = parse_all(manual_read_status, sizeof(manual_read_status), &pkt);
    check(r == DXL_PARSE_DONE && pkt.n_params == 4u && dxl_get_u32(pkt.params) == 166u,
          "A7 parse Read status -> 166");

    /* A8: same packet with ONE byte damaged must be rejected */
    uint8_t bad[sizeof(manual_read_status)];
    memcpy(bad, manual_read_status, sizeof(bad));
    bad[9] ^= 0x01; /* flip one bit of the data, like noise would */
    r = parse_all(bad, sizeof(bad), &pkt);
    check(r == DXL_PARSE_CRC_ERROR, "A8 one flipped bit -> CRC error");

    /* A9: junk before the header must be skipped (receive step 1, P2 p.11) */
    uint8_t junk[3 + sizeof(manual_ping_status)] = {0x12, 0xFF, 0x00};
    memcpy(&junk[3], manual_ping_status, sizeof(manual_ping_status));
    r = parse_all(junk, sizeof(junk), &pkt);
    check(r == DXL_PARSE_DONE && pkt.params[2] == 38u, "A9 junk before header is ignored");

    /* A10: the fake servo's reply builder must make the manual's exact bytes */
    uint8_t pp[3] = {0x06, 0x04, 0x26};
    n = dxl_build_status(1, 0, pp, 3, out, sizeof(out));
    check(n == sizeof(manual_ping_status) && same(out, manual_ping_status, n),
          "A10 build Ping status == manual p.12");
}

/* =========================================================================
 * PART B - live, over the two jumper wires, at 1 Mbps.
 * ========================================================================= */
static void part_b(void)
{
    dxl_result_t r;
    uint16_t model = 0;
    uint8_t fw     = 0;
    uint32_t v     = 0;
    const dxl_stats_t *st = dxl_get_stats();

    PRINTF("\r\n--- Part B: driver (LPUART2) <-> fake servo (LPUART1) at %u bps ---\r\n", BUS_BAUD);

    r = dxl_ping(SERVO_ID, &model, &fw);
    PRINTF("    ping -> %s, model %u, fw %u, round trip %u us\r\n", dxl_result_str(r), model, fw,
           st->last_rtt_us);
    check(r == DXL_OK && model == 1030u && fw == 38u, "B1 Ping ID1");

    r = dxl_read_u32(SERVO_ID, DXL_ADDR_PRESENT_POSITION, &v);
    PRINTF("    read present position -> %s, %u (rtt %u us)\r\n", dxl_result_str(r), v, st->last_rtt_us);
    check(r == DXL_OK && v == 166u, "B2 Read Present Position = 166");

    r = dxl_write_u32(SERVO_ID, DXL_ADDR_GOAL_POSITION, 2048u);
    check(r == DXL_OK && fake_servo_get_goal_position() == 2048u, "B3 Write Goal Position 2048 (fake got it)");
    r = dxl_read_u32(SERVO_ID, DXL_ADDR_GOAL_POSITION, &v);
    check(r == DXL_OK && v == 2048u, "B4 Read back Goal Position = 2048");

    uint8_t ids[1]  = {SERVO_ID};
    uint32_t val[1] = {1000u};
    r = dxl_sync_write_u32(DXL_ADDR_GOAL_POSITION, ids, val, 1);
    SDK_DelayAtLeastUs(500u, SystemCoreClock); /* no reply to wait for, so give the fake time to finish */
    check(r == DXL_OK && fake_servo_get_goal_position() == 1000u, "B5 Sync Write 1000 (fake got it)");

    r = dxl_ping(2u, NULL, NULL);
    PRINTF("    ping ID2 (nobody there) -> %s\r\n", dxl_result_str(r));
    check(r == DXL_ERR_TIMEOUT, "B6 Ping missing ID -> TIMEOUT");
    dxl_clear_bus_fault();

    /* B7: bus fault after N = 3 failures in a row */
    fake_servo_set_silent(true);
    for (int i = 1; i <= 3; i++)
    {
        r = dxl_ping(SERVO_ID, NULL, NULL);
        PRINTF("    silent ping %d -> %s, fails in a row = %u, bus_fault = %u\r\n", i, dxl_result_str(r),
               st->consecutive_fails, st->bus_fault);
    }
    check(st->bus_fault, "B7 3 timeouts -> BUS FAULT");
    fake_servo_set_silent(false);
    r = dxl_ping(SERVO_ID, NULL, NULL);
    check(r == DXL_OK && st->bus_fault, "B8 servo back, fault stays latched until cleared");
    dxl_clear_bus_fault();
    check(!st->bus_fault, "B9 fault cleared");

    /* B10: 1000 transactions in a row, timing statistics */
    uint32_t fails = 0, rtt_min = 0xFFFFFFFFu, rtt_max = 0, rtt_sum = 0;
    for (uint32_t i = 0; i < 1000u; i++)
    {
        fake_servo_set_present_position(i);
        r = dxl_read_u32(SERVO_ID, DXL_ADDR_PRESENT_POSITION, &v);
        if (r != DXL_OK || v != i)
        {
            fails++;
            continue;
        }
        uint32_t t = st->last_rtt_us;
        rtt_sum += t;
        rtt_min = (t < rtt_min) ? t : rtt_min;
        rtt_max = (t > rtt_max) ? t : rtt_max;
    }
    PRINTF("    1000 reads: %u failed, round trip min/avg/max = %u/%u/%u us\r\n", fails, rtt_min,
           (1000u - fails) ? rtt_sum / (1000u - fails) : 0u, rtt_max);
    PRINTF("    overruns %u, dropped %u, fake CRC errors %u\r\n", dxl_port_overruns(), dxl_port_dropped(),
           fake_servo_crc_errors());
    check(fails == 0u && dxl_port_overruns() == 0u, "B10 1000 reads, no failures, no overruns");
}
#endif /* TEST_MODE == 0 */

int main(void)
{
    BOARD_InitHardware(); /* clocks, pins (incl. FC1 + FC2 pins), debug console */

    /* Put FlexComm 1 and 2 in the combined "LPUART + LPI2C" persona.
     * This only works because reconfig.cmake defines
     * LPFLEXCOMM_INIT_NOT_USED_IN_DRIVER=1, so LPUART_Init won't switch it back. */
    LP_FLEXCOMM_Init(1u, LP_FLEXCOMM_PERIPH_LPI2CAndLPUART);
    LP_FLEXCOMM_Init(2u, LP_FLEXCOMM_PERIPH_LPI2CAndLPUART);

    PRINTF("\r\n==== Step 1: DYNAMIXEL driver vs fake servo (TEST_MODE %d) ====\r\n", TEST_MODE);
    PRINTF("core clock %u Hz, FC1 clk %u Hz, FC2 clk %u Hz\r\n", SystemCoreClock, CLOCK_GetLPFlexCommClkFreq(1u),
           CLOCK_GetLPFlexCommClkFreq(2u));

    bool ok_fake = fake_servo_init(SERVO_ID, BUS_BAUD);
    bool ok_port = dxl_port_init(BUS_BAUD);
    dxl_init();
    PRINTF("fake servo UART init: %s, driver UART init: %s\r\n", ok_fake ? "ok" : "FAILED", ok_port ? "ok" : "FAILED");
    if (!ok_fake || !ok_port)
    {
        PRINTF("Stopping: a UART would not start (baud rate not possible from this clock?)\r\n");
        while (1)
        {
        }
    }

#if TEST_MODE == 1
    PRINTF("TX_EN test: measure J1-6 to GND. HIGH ~3.3 V for 5 s, then LOW ~0 V for 5 s.\r\n");
    while (1)
    {
        dxl_port_force_tx_en(true);
        PRINTF("  TX_EN HIGH\r\n");
        SDK_DelayAtLeastUs(5000000u, SystemCoreClock);
        dxl_port_force_tx_en(false);
        PRINTF("  TX_EN LOW\r\n");
        SDK_DelayAtLeastUs(5000000u, SystemCoreClock);
    }
#elif TEST_MODE == 2
    PRINTF("Flood test: sending 0x%02X nonstop on J1-4. Read the DC voltage on J1-4.\r\n", FLOOD_BYTE);
    uint8_t blk[64];
    memset(blk, FLOOD_BYTE, sizeof(blk));
    while (1)
    {
        (void)LPUART_WriteBlocking(LPUART2, blk, sizeof(blk));
    }
#else
    part_a();
    part_b();
    PRINTF("\r\n==== RESULT: %u passed, %u failed ====\r\n", s_pass, s_fail);

    /* Then a status line 10 times a second, like the control loop will. */
    uint32_t pos = 0, n = 0;
    while (1)
    {
        fake_servo_set_present_position(pos);
        pos = (pos + 7u) % 4096u; /* pretend the joint is moving */
        uint32_t v     = 0;
        dxl_result_t r = dxl_read_u32(SERVO_ID, DXL_ADDR_PRESENT_POSITION, &v);
        const dxl_stats_t *st = dxl_get_stats();
        if ((n++ % 10u) == 0u)
        {
            PRINTF("S,%u,%s,pos=%u,rtt=%u,fails=%u,fault=%u\r\n", n, dxl_result_str(r), v, st->last_rtt_us,
                   st->consecutive_fails, st->bus_fault);
        }
        SDK_DelayAtLeastUs(10000u, SystemCoreClock); /* 100 Hz loop, print every 10th = 10 Hz */
    }
#endif
}
