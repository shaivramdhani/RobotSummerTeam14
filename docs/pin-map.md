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
| `FREQ` | 2 | IR frequency-select switch, INPUT_PULLUP; HIGH = 1 kHz / LOW = 10 kHz |
| `SolarHookServo` | 3 | LEDC channel 6, 50 Hz, 12-bit, 1000–2000 µs |
| `USTRIG1` | TODO | Unassigned after GPIO12 was reassigned to LSS3 |
| `USECHO1` | TODO | Unassigned after GPIO11 was reassigned to LSS2 |
| VL53L0X SDA | 10 | Dedicated ESP1 I2C data; verify carrier pull-ups and supply requirements before power-up |
| VL53L0X SCL | 9 | Dedicated ESP1 I2C clock; configured for 100 kHz |
| `LSS` | 4 | Digital comparator, HIGH = black tape |
| `LSS2` | 11 | Habitat Pieces left-side line sensor; digital comparator, HIGH = black tape |
| `LSS3` | 12 | Habitat Pieces right-side line sensor; digital comparator, HIGH = black tape |
| `LSBL` | 17 | Digital comparator, HIGH = black tape |
| `LSBR` | 18 | Digital comparator, HIGH = black tape |
| `PWMBL0` | TODO | TODO |
| `PWMBL1` | TODO | TODO |
| `PWMBR0` | TODO | TODO |
| `PWMBR1` | TODO | TODO |
| `PWMFunnel0` | 5 | PWM resource / active direction TODO |
| `PWMFunnel1` | 6 | PWM resource / active direction TODO |
| `LimitSwitchBackRightSide` | TODO | TODO |
| `LimitSwitchFrontRightSide` | TODO | TODO |
| UART TX to ESP2 | TODO | TODO |
| UART RX from ESP2 | TODO | TODO |

## ESP1 Motor Configuration TODOs

| Motor | PWM0 channel | PWM1 channel | Frequency | Resolution | H-bridge mode | Forward sign |
| --- | --- | --- | --- | --- | --- | --- |
| Back-left | TODO | TODO | TODO | TODO | TODO | TODO |
| Back-right | TODO | TODO | TODO | TODO | TODO | TODO |
| Funnel | TODO | TODO | TODO | TODO | TODO | TODO |

ESP1 `maximum_safe_test_duty`: `1.0` for drive testing.
ESP1 UART baud rate: TODO.
ESP1 Solar Hook servo: GPIO3, LEDC channel 6, 50 Hz, 12-bit,
1000–2000 µs. The output initializes detached/disabled.

ESP1 VL53L0X V2: SDA GPIO10, SCL GPIO9, 7-bit address `0x29`. It has
a processor-local I2C bus separate from ESP2's IMU bus. Connect the sensor
ground to ESP1 ground. Confirm the exact carrier board's VIN and I2C pull-up
voltage before applying power.

## ESP2 Pin TODOs

| Signal | GPIO | Active level / electrical notes |
| --- | --- | --- |
| `IMU SDA` | 18 | Current software configuration; verify the physical PCB connection |
| `IMU SCL` | 17 | Current software configuration; verify the physical PCB connection |
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
| `HabitatPusherServo` | 5 | LEDC channel 7, 50 Hz, 12-bit, 1000–2000 µs; open/closed angles remain adjustable |
| `WinchServo` | 6 | MCPWM unit 0, timer 0, generator A; 50 Hz, 1 MHz timer resolution, 1000–2000 µs |
| `LimitSwitchHabitatPiece` | 48 | Habitat Pieces forward stop, LOW released / HIGH pressed |
| `LimitSwitchStepperBottom` | TODO | TODO |
| `LimitSwitchStepperMiddle` | TODO | TODO |
| `LimitSwitchStepperTop` | TODO | TODO |
| `LimitSwitchFunnelLeft` | 47 | PegFinder funnel stop, LOW released / HIGH pressed |
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

ESP2 Habitat Pusher servo uses GPIO5, LEDC channel 7, 50 Hz, 12-bit, and
1000–2000 µs. ESP2 Winch uses GPIO6 and the independent MCPWM unit 0,
timer 0, generator A at 50 Hz with 1 µs timer resolution and the same pulse
range. Both outputs initialize detached/disabled. Habitat Placement remains
locked until both adjustable pusher targets are configured.
