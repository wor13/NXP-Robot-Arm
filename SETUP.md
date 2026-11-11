# FRDM-MCXN947 → DYNAMIXEL arm: quick setup

Drive the OMX arm (XL430 IDs 11–13, XL330 IDs 1–3) from the FRDM-MCXN947 through the ROBOTIS DYNAMIXEL Shield for Arduino MKR. No OpenRB-150 needed. Tested 11/10/2026: ping, 1000 reads with 0 failures, and a slow move on ID 11.

## 1. What you need

- FRDM-MCXN947 + USB-C cable to the laptop
- ROBOTIS DYNAMIXEL MKR shield, **JP1 jumper removed**
- 6 jumper wires
- SMPS2DXL board + 12 V supply (from the OMX kit)
- 2 DYNAMIXEL 3-pin cables
- Serial terminal on the MCU-Link COM port at 115200 baud

## 2. Safety rules (read first)

1. **JP1 off.** With JP1 on, the arm's 12 V reaches the FRDM's 5 V rail. Check: shield 5V pin to shield servo-port pin 2 → **no beep** (power off).
2. **Never plug an XL330 (IDs 1–3) into the shield or the SMPS2DXL.** Those ports carry 12 V on pin 2. XL330s are rated 3.7–6.0 V and get 5 V from the arm's own buck converter.
3. **Power off before plugging or unplugging any cable.**
4. **Order:** power on = FRDM USB first, then 12 V. Power off = 12 V first, then USB.
5. Use different wire colors for 3.3 V and 5 V. Swapping them puts 5 V on an FRDM pin (FRDM pins are 3.3 V only).
6. Only one 12 V supply at a time (don't also power the shield's own jack).

## 3. Wiring: FRDM → shield (USB unplugged, GND first)

| # | FRDM pin | Signal | Shield pin | Shield label |
|---|---|---|---|---|
| 1 | J3-12 | GND | J2-11 | GND |
| 2 | J3-8 | 3.3 V | J2-12 | VCC (3.3V) |
| 3 | J3-10 | 5 V | J2-14 | 5V |
| 4 | J1-4 | P4_2 = UART2 TX | J2-9 | TX (14) |
| 5 | J1-2 | P4_3 = UART2 RX | J2-8 | RX (13) |
| 6 | J1-6 | P0_29 = direction (GPIO) | J1-8 | A6 |

FRDM pins: NXP UM12018 Table 17 / Table 19. Shield pins: ROBOTIS shield schematic `PUBLIC_DXL_SHIELD(MKR)_REVC1`.

## 4. Wiring: shield → arm

```
12 V supply ──► SMPS2DXL ──(port B)──► arm, cable into ID 11 (base XL430)
                   ▲
               (port A)
                   │
             shield servo port   (only DATA + GND used here)
```

The SMPS2DXL sits in the middle, so the motor current goes straight into the arm and never through the shield.

## 5. Check voltages before connecting the arm

FRDM USB on, 12 V off, arm not connected. Measure to GND:

| Point | Expect |
|---|---|
| Shield 3.3V pin | 3.3 V |
| Shield 5V pin | ~4.6–4.75 V (ours: 4.64 V) |
| Shield servo port pin 3 (DATA) | ≈ shield 5V pin |
| Shield servo port pin 2 | ~0 V |
| FRDM J1-2 (RX) | ~3.3 V, **never above 3.4 V** |

Then connect the arm (USB off), power on in order, and re-check that the shield 5V pin still reads ~4.6 V (not 12 V).

## 6. Firmware

Built on the MCUXpresso SDK example `driver_examples/lpflexcomm/interrupt_transfer` (frdmmcxn947, cm33_core0). Its `pin_mux.c` already puts FlexComm 2 on J1-2/J1-4. Add these files to the project and list the `.c` files in `CMakeLists.txt` with `mcux_add_source`:

| File | What it does |
|---|---|
| `dxl_protocol.c/.h` | Builds and parses Protocol 2.0 packets, CRC |
| `dxl_port.c/.h` | UART2 at 1 Mbps + direction pin (J1-6) |
| `dxl.c/.h` | `dxl_ping`, `dxl_read_u32`, `dxl_write`, `dxl_write_u32`, `dxl_sync_write_u32` |
| `fake_servo.c/.h` | Software servo for bench tests (not used on the real arm) |
| `dxl_step1_main.c` | Test program, pick a `TEST_MODE` |

`CMakeLists.txt` also needs `mcux_add_macro(CC "-DLPFLEXCOMM_INIT_NOT_USED_IN_DRIVER=1")`.

Pick the test at the top of `dxl_step1_main.c`:

| `TEST_MODE` | What it does | Moves the arm? |
|---|---|---|
| 0 | Self-test vs fake servo (bench only) | No |
| 1 | Toggles direction pin J1-6 every 5 s (multimeter) | No |
| 2 | Floods 0x55 on the bus (scope) | No |
| 3 | Pings every ID in `REAL_IDS`, 1000 reads, then positions once a second | **No** (read only) |
| 4 | Slow move of `MOVE_ID` by `MOVE_DELTA`, 3 times, then torque off | **Yes** |

Bus: 1,000,000 bps, Protocol 2.0. `REAL_IDS` for the whole arm: `{11u, 12u, 13u, 1u, 2u, 3u}`.

## 7. Expected output (TEST_MODE 3, ID 11)

```
Real-servo test, READ ONLY, 1 ID(s) at 1000000 bps
  ping ID 11 -> OK, model 1060, fw 50, round trip 1183 us
  ID 11, 1000 reads: 0 failed, round trip min/avg/max = 1160/1233/1240 us, overruns 0
P,0,ID11=2048
```

Whole arm (`REAL_IDS` = all six), tested 11/10/2026:

```
  ping ID 11 -> OK, model 1060, fw 50, round trip 1184 us
  ping ID 12 -> OK, model 1060, fw 50, round trip 1182 us
  ping ID 13 -> OK, model 1060, fw 50, round trip 1184 us
  ping ID 1 -> OK, model 1200, fw 53, round trip 768 us
  ping ID 2 -> OK, model 1200, fw 53, round trip 769 us
  ping ID 3 -> OK, model 1200, fw 53, round trip 769 us
P,0,ID11=2049,ID12=755,ID13=3146,ID1=3114,ID2=2085,ID3=943
```

Model 1060 = XL430-W250, 1200 = XL330-M288. Position 0–4095 = one turn. The XL430s answer ~0.4 ms slower than the XL330s; this matters for control-loop timing (reading all six one by one takes ~6 ms).

## 8. Moving a joint safely (pattern used in TEST_MODE 4)

1. Write Profile Velocity (address 112) to a slow value.
2. Write Goal Position (116) = current Present Position (132), so turning torque on causes no jump.
3. Write Torque Enable (64) = 1.
4. Write new Goal Positions. For several joints at once, use `dxl_sync_write_u32`.
5. Write Torque Enable = 0 when done. **The joint goes limp**, so support joints that gravity can pull down.

## 9. Troubleshooting

| Output | Likely cause | Check |
|---|---|---|
| `TIMEOUT` on every ID | No reply reaching RX | 12 V on? Cable in ID 11? TX/RX (rows 4–5) swapped? A6 on J1-6? |
| `CRC_ERROR` / `BAD_REPLY` | Bits dropped | Scope DATA in TEST_MODE 2; the shield's TX buffer is marginal at 3.3 V input |
| Worked, then every ID stops | DATA shorted to GND | Power off; measure DATA–GND on each servo unpowered (tens of Ω = damaged servo) |
| Shield 5V pin reads ~12 V | JP1 is on | Power off immediately, remove JP1 |
