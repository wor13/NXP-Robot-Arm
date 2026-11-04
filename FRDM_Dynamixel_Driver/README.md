# FRDM-MCXN947 Dynamixel servo driver (Step 1)

Author: Isaac (Communication / Data Processing)

Driver that lets the FRDM-MCXN947 talk to the arm's Dynamixel servos
(Protocol 2.0, half-duplex UART at 1 Mbps), plus a test program that checks
the driver against a "fake servo" running on a second UART of the same board.

Status (10/24): all 20 tests in Parts A and B passed on the FRDM board.
Not yet tested on the real arm.

## Files

| File | What it does |
|---|---|
| `dxl.h` / `dxl.c` | Driver you call from your code: ping, read, write, sync write. Latches a bus fault after 3 failed reads. |
| `dxl_protocol.h` / `.c` | Builds and checks Protocol 2.0 packets (header, CRC, byte stuffing). No hardware here. |
| `dxl_port.h` / `.c` | Hardware layer: LPUART2 on FlexComm 2 (TX J1-4, RX J1-2), TX_EN on J1-6 (P0_29), 1 Mbps. |
| `fake_servo.h` / `.c` | Pretend servo on LPUART1 (RX J2-6, TX J2-10), used only for testing. |
| `dxl_step1_main.c` | Test program. Results print on the MCU-Link COM port at 115200. |
| `frdmmcxn947_cm33_core0/` | Board files (pins, clocks) from the SDK example. |

## How to build

1. Install the MCUXpresso SDK for FRDM-MCXN947 and MCUXpresso for VS Code.
2. Import the SDK example `driver_examples/lpflexcomm/interrupt_transfer`
   as a freestanding project. This project started from that example.
3. Copy these files into that project folder, replacing the ones with the same name.
4. Build and flash as usual. Open the COM port at 115200 to see the results.

Pick what the test program does with `TEST_MODE` at the top of `dxl_step1_main.c`.

## Test wiring (USB unplugged while wiring)

- J1-4 (driver TX) -> J2-6 (fake servo RX)
- J2-10 (fake servo TX) -> J1-2 (driver RX)
