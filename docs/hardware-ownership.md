# Hardware Ownership

Every physical peripheral must have exactly one software owner.

## ESP1

| Peripheral | Signals | Software owner |
| --- | --- | --- |
| Mission state machine | n/a | ESP1 mission task |
| IR beacon inputs | `LeftIRFiltered`, `RightIRFiltered`, `FREQ` | ESP1 sensor acquisition |
| Ultrasonic sensors | `USTRIG1`, `USECHO1`, `USTRIG2`, `USECHO2` | ESP1 sensor acquisition |
| Rear/side line sensors | `LSS`, `LSBL`, `LSBR` | ESP1 sensor acquisition |
| Back-left motor | `PWMBL0`, `PWMBL1` | ESP1 motor output |
| Back-right motor | `PWMBR0`, `PWMBR1` | ESP1 motor output |
| Funnel motor | `PWMFunnel0`, `PWMFunnel1` | ESP1 funnel output |
| Right-side limit switches | `LimitSwitchBackRightSide`, `LimitSwitchFrontRightSide` | ESP1 sensor acquisition |
| UART link to ESP2 | TX/RX | ESP1 communication |

## ESP2

| Peripheral | Signals | Software owner |
| --- | --- | --- |
| Front line sensors | `LSFL`, `LSFR` | ESP2 sensor acquisition |
| Front-left motor | `PWMFL0`, `PWMFL1` | ESP2 motor output |
| Front-right motor | `PWMFR0`, `PWMFR1` | ESP2 motor output |
| Four-wheel motion calculation | logical command only | ESP2 motion controller |
| Stepper | `STEP`, `DIR`, `SLEEP` | ESP2 stepper controller |
| Servos | `MSCLAW1`, `MSCLAW2`, `MSCLAW3`, `PusherServo`, `WinchServo` | ESP2 servo bank |
| Stepper limit switches | `LimitSwitchStepperBottom`, `LimitSwitchStepperMiddle`, `LimitSwitchStepperTop` | ESP2 sensor acquisition |
| Funnel limit switches | `LimitSwitchFunnelLeft`, `LimitSwitchFunnelRight` | ESP2 sensor acquisition |
| UART link to ESP1 | TX/RX | ESP2 communication |

High-level mission code must not directly access GPIO or PWM resources.
