# Hardware Ownership

Every physical peripheral must have exactly one software owner.

## ESP1

| Peripheral | Signals | Software owner |
| --- | --- | --- |
| Mission state machine | n/a | ESP1 mission task |
| IR beacon inputs | `LeftIRFiltered`, `RightIRFiltered`, `FREQ` GPIO2 | ESP1 sensor acquisition |
| VL53L0X V2 distance sensor | SDA GPIO10, SCL GPIO9 | ESP1 sensor-acquisition task; communication and mission code consume immutable snapshots only |
| Rear/side line sensors | `LSS` GPIO4, `LSS2` GPIO11, `LSS3` GPIO12, `LSBL`, `LSBR` | ESP1 sensor acquisition |
| Back-left motor | `PWMBL0`, `PWMBL1` | ESP1 motor output |
| Back-right motor | `PWMBR0`, `PWMBR1` | ESP1 motor output |
| Funnel motor | `PWMFunnel0`, `PWMFunnel1` | ESP1 funnel output |
| Solar Hook servo | `SolarHookServo` GPIO3 | ESP1 solar-hook servo output |
| Right-side limit switches | `LimitSwitchBackRightSide`, `LimitSwitchFrontRightSide` | ESP1 sensor acquisition |
| UART link to ESP2 | TX/RX | ESP1 communication |

## ESP2

| Peripheral | Signals | Software owner |
| --- | --- | --- |
| MPU-6050-compatible IMU | I2C SDA/SCL | ESP2 sensor-acquisition task; motion and web code consume snapshots only |
| Front line sensors | `LSFL`, `LSFR` | ESP2 sensor acquisition |
| Final Competition start switch | GPIO10, active HIGH | ESP2 motion-control task |
| Front-left motor | `PWMFL0`, `PWMFL1` | ESP2 motor output |
| Front-right motor | `PWMFR0`, `PWMFR1` | ESP2 motor output |
| Four-wheel motion calculation | logical command only | ESP2 motion controller |
| Stepper | `STEP`, `DIR`, `SLEEP` | ESP2 stepper controller |
| Servos | `MSCLAW1`, `MSCLAW2`, `MSCLAW3`; `HabitatPusherServo` GPIO5/LEDC 7; `WinchServo` GPIO6/MCPWM 0/0/A | ESP2 servo bank |
| Habitat-piece approach limit switch | GPIO48, active HIGH | ESP2 sensor acquisition |
| Stepper limit switches | `LimitSwitchStepperBottom`, `LimitSwitchStepperMiddle`, `LimitSwitchStepperTop` | ESP2 sensor acquisition |
| Funnel limit switches | `LimitSwitchFunnelLeft`, `LimitSwitchFunnelRight` | ESP2 sensor acquisition |
| UART link to ESP1 | TX/RX | ESP2 communication |

High-level mission code must not directly access GPIO or PWM resources.
