
## Example Summary

The following example configures TimerG9 in combined capture mode (configure to
measure a PWM signal's period and duty cycle). TimerG8 is used to generate a the
PWM signal. This examples configures timers in down counting mode.

## Peripherals, Pin Functions, MCU Pins, Launchpad Pins
| Peripheral | Function | Pin | Launchpad Pin |
| --- | --- | --- | --- |
| TIMG8 | Capture/Compare Pin 0 | PA7 | J2_19 |
| TIMG9 | Capture/Compare Pin 0 | PB7 | J2_14 |
| BOARD | Debug Clock | PA20 | J19_16 |
| BOARD | Debug Data In Out | PA19 | J19_14 |

### Device Migration Recommendations
This project was developed for a superset device included in the MSPM0 LaunchPad. Please
visit the [CCS User's Guide](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/ccs_ide_guide/doc_guide/doc_guide-srcs/ccs_ide_guide.html#sysconfig-project-migration)
for information about migrating to other MSPM0 devices.

### Low-Power Recommendations
TI recommends to terminate unused pins by setting the corresponding functions to
GPIO and configure the pins to output low or input with internal
pullup/pulldown resistor.

SysConfig allows developers to easily configure unused pins by selecting **Board**→**Configure Unused Pins**.

For more information about jumper configuration to achieve low-power using the
MSPM0 LaunchPad, please visit the LP-MSPM0G3519 User's Guide.

## Example Usage
Connect a jumper cable between the two capture compare pins.
Compile, load and run the example.
The application will capture two full periods before hitting a SW breakpoint.
This ensures that the timers are fully synchronized (please refer the TRM for
additional details). At the breakpoint, the user can inspect the values of
pwmPeriod and pwmDuty.
The value of pwmDuty should be set to ~50.
The value of pwmPeriod should be set to ~1000.
