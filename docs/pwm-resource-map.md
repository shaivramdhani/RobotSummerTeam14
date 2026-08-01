# PWM Resource Map

Keep the reviewed LEDC and MCPWM allocations below exclusive.

ESP2 has four motor PWM outputs and five servo outputs. ESP1 has four rear motor
PWM outputs, two funnel motor PWM outputs, and the Solar Hook servo output. The
final allocation must account for ESP32-S3 LEDC timer/channel limits, servo
pulse requirements, motor driver frequency requirements, and any library
constraints.

## Current Status

| Processor | Output group | Signals | Allocation |
| --- | --- | --- | --- |
| ESP1 | Back-left motor | `PWMBL0`, `PWMBL1` | TODO |
| ESP1 | Back-right motor | `PWMBR0`, `PWMBR1` | TODO |
| ESP1 | Funnel motor | `PWMFunnel0`, `PWMFunnel1` | TODO |
| ESP1 | Solar Hook servo | `SolarHookServo` GPIO3 | LEDC 6, 50 Hz, 12-bit |
| ESP2 | Front-left motor | `PWMFL0`, `PWMFL1` | TODO |
| ESP2 | Front-right motor | `PWMFR0`, `PWMFR1` | TODO |
| ESP2 | Claw servos | `MSCLAW1`, `MSCLAW2`, `MSCLAW3` | LEDC 4–6, 50 Hz, 12-bit |
| ESP2 | Winch servo | `WinchServo` GPIO6 | MCPWM unit 0, timer 0, generator A; 50 Hz, 1 MHz timer resolution |
| ESP2 | Habitat Pusher servo | `HabitatPusherServo` GPIO5 | LEDC 7, 50 Hz, 12-bit |

Firmware now provides dual-PWM motor adapters, but they refuse to initialize
unless each motor has GPIOs, LEDC channels, PWM frequency, PWM resolution,
H-bridge mode, and `forward_sign` configured in the owning ESP's `PinConfig.h`.
The ESP2 servo adapter refuses movement until each output has a GPIO, PWM
backend, frequency, safe pulse range, and backend-specific resource allocation.
The three claws use LEDC channels 4–6, and the Habitat Pusher exclusively uses
GPIO5/LEDC channel 7. The GPIO6 Winch uses MCPWM unit 0, timer 0, generator A,
which is independent of all eight LEDC channels. All five run at 50 Hz with
1000–2000 µs pulses; MCPWM uses a 1 MHz timer resolution. The ESP1 Solar Hook
uses GPIO3/LEDC channel 6 on the other processor.

## Review Questions

- Which outputs require hardware PWM instead of simple digital output?
- What PWM frequency and resolution does each motor driver require?
- What servo pulse widths are safe for each mechanism?
- Can all servo outputs share one timer configuration?
- Do any pins conflict with boot strapping, USB, flash, PSRAM, or UART?
- What is the disabled electrical state for every actuator output?
