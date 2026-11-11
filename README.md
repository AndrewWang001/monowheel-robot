# Monowheel Robot

Firmware for a monowheel robot with two 3-DOF legs. The legs stand it up, hold poses and
help it balance side to side, and two brushless motors drive the wheel. It's controlled with
an ELRS radio transmitter.

ESP32 + ESP-IDF 5.5 + FreeRTOS.

## Hardware

- ESP32-WROOM-32 DevKit V1
- BNO085 IMU (I2C)
- 6x Hiwonder HTD-45H bus servos through a BusLinker board
- 2x brushless motors on Repeat AM32 ESCs, both on the same wheel
- RadioMaster RP2 ELRS receiver + T8L transmitter

## Pinout

| Function | GPIO | Goes to |
|---|---|---|
| IMU SDA | 21 | BNO085 SDA |
| IMU SCL | 22 | BNO085 SCL |
| IMU INT | 26 | BNO085 INT |
| IMU RST | 32 | BNO085 RST |
| Servo TX (UART2) | 17 | BusLinker RX |
| Servo RX (UART2) | 16 | BusLinker TX |
| Radio RX (UART1) | 27 | RP2 TX |
| Radio TX (UART1) | 25 | RP2 RX |
| ESC 1 | 18 | motor 1 ESC signal |
| ESC 2 | 19 | motor 2 ESC signal |

Servo bus runs at 115200, the radio at 420000 (CRSF), ESCs at 50 Hz PWM.

BNO085 in I2C mode: PS0 and PS1 to GND, ADO to GND (address 0x4A), CS to 3V3.

All pins live in `components/pins/include/pins.h`. Stay off GPIO 0, 2, 5, 12, 15 (boot
pins), 1 and 3 (USB serial) and 6-11 (flash).

## Servo IDs

| Leg | Base | Shoulder | Elbow |
|---|---|---|---|
| Back | 1 | 3 | 2 |
| Front | 4 | 5 | 6 |

Each servo has an angle offset saved in it so that straight up reads 500.

## Controls (Drive mode)

- CH2 (right stick up/down): drive. Up is forward, down is reverse, gentle near the centre, capped at 95%
- CH5 (left bottom switch): kill switch. KILL stops the motors and freezes the legs
- CH6 (left top button): stand up. Press again to pause, again to keep going
- CH7 (left 3-way switch): legs vertical / half / ground then final pose
- CH9 (right top button): servo offset calibration (hold the legs straight, press again to save)

## Building

1. Install ESP-IDF 5.5
2. Run `python tools/pick_mode.py`, pick a mode
3. Hit Build + Flash + Monitor (or `idf.py flash monitor`)

Unplug the motor battery before flashing. The ESCs can twitch while the ESP resets.

## Modes

- **Drive**: the normal firmware
- **Test: IMU**: prints lean/tilt and whether it's upright, leaning, falling or fallen
- **Test: controller**: prints all the radio channels
- **Test: balance**: side balance with the legs. Sweep the poses by hand or run the PD controller and tune it live
- **Test: IK**: inverse kinematics running live. Sticks move the foot, there's a circle demo, and it self-tests the math on boot
- **Test: leg sweep / motor sweep / motor + controller**: one part at a time
- **Servo calibration**: center, read, check and slider tools for setting up the servos

## Layout

```
main/               entry point, one file per mode in main/modes/
components/
  imu/              BNO085 at 300 Hz, interrupt driven
  rc/               CRSF receiver and kill switch
  esc/  drive/      ESC PWM and stick to throttle
  servo_bus/        bus servo driver
  legs/             joint calibration, angles to servo positions
  leg_control/      pose sequencer (stand-up, poses, calibration)
  leg_kinematics/   leg IK / FK
  gait/             gait planning
  pins/             every GPIO
sim/                Python sims for the legs and gait, servo slider tool
tools/pick_mode.py  mode picker / build and flash
```

## Status

Working: driving, radio and kill switch, servo control and calibration, IMU, stand-up sequencer.

Still going: tuning the balance controller, testing the stand-up on battery, checking joint directions for the IK.
