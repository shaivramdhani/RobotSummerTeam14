# Pin Map

GPIO numbers are intentionally unassigned. Do not fill these in from memory.
Update this file and the matching `PinConfig.h` only after the schematic and
wiring have been verified.

For motor outputs, also verify LEDC channel, PWM frequency, PWM resolution,
H-bridge mode, per-wheel `forward_sign`, and `maximum_safe_test_duty`.

## ESP1 Pin TODOs

| Signal | GPIO | Active level / electrical notes |
| --- | --- | --- |
| `LeftIRFiltered` | TODO | TODO |
| `RightIRFiltered` | TODO | TODO |
| `FREQ` | TODO | TODO |
| `USTRIG1` | TODO | TODO |
| `USECHO1` | TODO | TODO |
| `USTRIG2` | TODO | TODO |
| `USECHO2` | TODO | TODO |
| `LSS` | TODO | TODO |
| `LSBL` | TODO | TODO |
| `LSBR` | TODO | TODO |
| `PWMBL0` | TODO | TODO |
| `PWMBL1` | TODO | TODO |
| `PWMBR0` | TODO | TODO |
| `PWMBR1` | TODO | TODO |
| `PWMFunnel0` | TODO | TODO |
| `PWMFunnel1` | TODO | TODO |
| `LimitSwitchBackRightSide` | TODO | TODO |
| `LimitSwitchFrontRightSide` | TODO | TODO |
| UART TX to ESP2 | TODO | TODO |
| UART RX from ESP2 | TODO | TODO |

## ESP1 Motor Configuration TODOs

| Motor | PWM0 channel | PWM1 channel | Frequency | Resolution | H-bridge mode | Forward sign |
| --- | --- | --- | --- | --- | --- | --- |
| Back-left | TODO | TODO | TODO | TODO | TODO | TODO |
| Back-right | TODO | TODO | TODO | TODO | TODO | TODO |

ESP1 `maximum_safe_test_duty`: `1.0` for drive testing.
ESP1 UART baud rate: TODO.

## ESP2 Pin TODOs

| Signal | GPIO | Active level / electrical notes |
| --- | --- | --- |
| `LSFL` | 19 | Digital comparator, HIGH = black tape |
| `LSFR` | 20 | Digital comparator, HIGH = black tape |
| `PWMFL0` | TODO | TODO |
| `PWMFL1` | TODO | TODO |
| `PWMFR0` | TODO | TODO |
| `PWMFR1` | TODO | TODO |
| `STEP` | TODO | TODO |
| `DIR` | TODO | TODO |
| `SLEEP` | TODO | TODO |
| `MSCLAW1` | 14 | PWM channel/frequency/pulse range TODO |
| `MSCLAW2` | 13 | PWM channel/frequency/pulse range TODO |
| `MSCLAW3` | 12 | PWM channel/frequency/pulse range TODO |
| `PusherServo` | TODO | TODO |
| `WinchServo` | TODO | TODO |
| `LimitSwitchStepperBottom` | TODO | TODO |
| `LimitSwitchStepperMiddle` | TODO | TODO |
| `LimitSwitchStepperTop` | TODO | TODO |
| `LimitSwitchFunnelLeft` | TODO | TODO |
| `LimitSwitchFunnelRight` | TODO | TODO |
| UART TX to ESP1 | TODO | TODO |
| UART RX from ESP1 | TODO | TODO |

## ESP2 Motor Configuration TODOs

| Motor | PWM0 channel | PWM1 channel | Frequency | Resolution | H-bridge mode | Forward sign |
| --- | --- | --- | --- | --- | --- | --- |
| Front-left | TODO | TODO | TODO | TODO | TODO | TODO |
| Front-right | TODO | TODO | TODO | TODO | TODO | TODO |

ESP2 `maximum_safe_test_duty`: `1.0` for drive testing.
ESP2 UART baud rate: TODO.
