## Example Summary

This code example serves as a reference for implementing a smart analog sensor
interface (SASI) for smoke detection. This software is only part of a complete
system and is intended to only be used as a reference.

The reference design [TIDA-010941](https://www.ti.com/tool/TIDA-010941)
includes a guide explaining how to design and implement a smoke detection
system using the SASI sensing board. It will further explain the system design,
optical and mechanical design of the sensing board, software and GUI
functionality, and testing (ambient light and air quality) setup and results.

A GUI can be used with the software, and the user can interact with the demo
using the GUI. For access to the GUI and for more details on the system, please
refer to the TI reference design.

The complete reference design will be released in 2023 and will be available at
https://www.ti.com/tool/TIDA-010941.

## Peripherals & Pin Assignments

| Peripheral | Pin | Function |
| --- | --- | --- |
| GPIOA | PA25 | Standard Output |
| GPIOA | PA15 | Standard Output |
| GPIOA | PA6 | Standard Output |
| SYSCTL |  |  |
| TIMG2 | PA3 | Counter Compare Pin 0 |
| TIMG2 | PA4 | Counter Compare Pin 1 |
| TIMG0 | PA12 | Counter Compare Pin 0 |
| TIMG0 | PA13 | Counter Compare Pin 1 |
| TIMG1 | PA7 | Counter Compare Pin 0 |
| TIMG1 | PA27 | Counter Compare Pin 1 |
| OPA0 | PA22 | OPA Output Pin |
| OPA0 | PA24 | OPA Inverting input 0 Pin |
| OPA1 | PA16 | OPA Output Pin |
| OPA1 | PA17 | OPA Inverting input 0 Pin |
| OPA1 | PA18 | OPA Non-inverting input 0 Pin |
| VREF | PA23 | VREF Positive (VREF+) |
| VREF | PA21 | VREF Negative (VREF-) |
| UART0 | PA9 | RX Pin |
| UART0 | PA8 | TX Pin |
| I2C0 | PA10 | I2C Serial Data line (SDA) |
| I2C0 | PA11 | I2C Serial Clock line (SCL) |
| EVENT |  |  |
| DEBUGSS | PA20 | Debug Clock |
| DEBUGSS | PA19 | Debug Data In Out |

## BoosterPacks, Board Resources & Jumper Settings

See the [TIDA-010941](https://www.ti.com/tool/TIDA-010941) reference design
report for more details on the SASI sensing board and hardware files.

### Low-Power Recommendations

See the [TIDA-010941](https://www.ti.com/tool/TIDA-010941) reference design
report for power consumption measurements.


### Device Migration Recommendations
This project was developed for a superset device included in the LP_MSPM0L1306 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#non-sysconfig-compatible-project-migration)
for information about migrating to other MSPM0 devices.

## Example Usage

This example is provided in association with the [TIDA-010941](https://www.ti.com/tool/TIDA-010941).
The board requires an external SWD programmer, such as an XDS-110 or LaunchPad.
Refer to the TI Design User's Guide for information on how to program, run and
customize your solution.
