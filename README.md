# maglev
Example of PID controller usage applied to magnetic levitation.

## Overview
Magnetic levitator on the base of Texas Instruments TIVA-C TM4C123GH6PM ARM Cortex-M4F MCU (EK-TM4C123GXL board). Regulation is performed by PID algorythm. Remote monitoring and control is provided by external Ethernet controller chip Microchip ENC28J60 (UDP commands).

## Architecture
This version of maglev uses TivaWare library. See also [TI-RTOS version](https://github.com/ussserrr/maglev-ti-rtos) (more up-to-date). General logic is:
  - `main()` function initializes all periphery and SW modules: clocking system, GPIO, PWM, PID, ADC, SPI, UART, etc;
  - in the end of `main()`, `while(1)` loop is starting. It works as a UDP server and constantly looking for commands;
  - `TimerADC_Handler()` interrupt firing every 1 ms and triggering ADC to start measurements;
  - `ADC_Handler()` interrupt manages of getting ADC results and also calling PID mechanism;

For remote control have been used ported from AVR lightweight driver and network stack for ENC28J60.

## Installation
Due to not always proper work of TI CCS (Code Composer Studio) (exporting/importing projects) this repository is just set of necessary files. It's recommended to create brand new clean CCS project and simply put all these files into it. In case of non-proper work try to increase stack size to 2KB (in Project settings).

## Known issues
Besides probably not ideal SW architecture :), there is a strange behavior of PWM module. All current values of registers indicate that levitation shouldn't actually work, but it does. See forward to understand this fact.

## Copyright
All third-party software components (ip_arp_udp_tcp stack, enc28j60 driver and so on) belongs to their authors.
