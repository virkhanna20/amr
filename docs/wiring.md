# AMR Wiring & Pin Allocation

## Overview

```
┌──────────┐    USB     ┌─────────────┐    Serial    ┌─────────────┐
│ RPLidar  │◄──────────►│             │◄────────────►│   Arduino   │
│ (LIDAR)  │ /dev/ttyUSB0│  RPi 4B     │ /dev/ttyACM0 │     Uno     │
└──────────┘            │             │              └──────┬──────┘
                        │  ROS2       │                     │
┌──────────┐    CSI     │  Humble     │              ┌──────┴──────┐
│ Pi Cam   │◄──────────►│             │              │  PWM + DIR  │
│ v3       │            └─────────────┘              ├─────────────┤
└──────────┘                                         │ MD10C #1    │──► Left Motor
                                                     │ (Left)      │◄── Left Encoder
                                                     ├─────────────┤
                                                     │ MD10C #2    │──► Right Motor
                                                     │ (Right)     │◄── Right Encoder
                                                     └─────────────┘
                                                           │
                                                     ┌─────┴─────┐
                                                     │  Battery   │
                                                     │  7-24V     │
                                                     └───────────┘
```

## Arduino Uno Pin Assignments

| Arduino Pin | Function         | Connects To              |
|-------------|------------------|--------------------------|
| D2 (INT0)   | Left Encoder A   | Left motor encoder CH A  |
| D3 (INT1)   | Right Encoder A  | Right motor encoder CH A |
| D4          | Left Motor DIR   | MD10C #1 DIR pin         |
| D5 (PWM)    | Left Motor PWM   | MD10C #1 PWM pin         |
| D6 (PWM)    | Right Motor PWM  | MD10C #2 PWM pin         |
| D7          | Right Motor DIR  | MD10C #2 DIR pin         |
| D8          | Left Encoder B   | Left motor encoder CH B  |
| D9          | Right Encoder B  | Right motor encoder CH B |
| 5V          | Encoder power    | Encoder VCC (if needed)  |
| GND         | Common ground    | MD10C GND + Encoder GND  |

## Cytron MD10C Wiring (per driver)

| MD10C Pin | Connects To          |
|-----------|----------------------|
| PWM       | Arduino PWM pin      |
| DIR       | Arduino digital pin  |
| GND       | Arduino GND          |
| VIN+      | Battery positive     |
| VIN-      | Battery negative     |
| OUT+      | Motor terminal +     |
| OUT-      | Motor terminal -     |

**Note:** Both MD10C drivers share the same battery power supply.
Arduino GND must be connected to MD10C GND for signal reference.

## Motor Specifications

| Parameter              | Value          | Notes                    |
|------------------------|----------------|--------------------------|
| Motor type             | Pro Range DC   | Planetary gear, brushed  |
| Encoder type           | Hall effect    | Built-in, 2 channels     |
| Ticks per revolution   | **MEASURE**    | Default: 330 in config   |
| Wheel radius           | 0.05 m         | Update after measuring   |
| Wheel separation       | 0.35 m         | Update after measuring   |

## USB Device Mapping

| Device         | Default Port    | USB Chip  |
|----------------|-----------------|-----------|
| RPLidar        | /dev/ttyUSB0    | CP2102    |
| Arduino Uno    | /dev/ttyACM0    | ATmega16U2|

## Serial Protocol (Arduino ↔ RPi)

**Baud rate:** 115200

| Direction       | Format                           | Example              |
|-----------------|----------------------------------|----------------------|
| RPi → Arduino   | `M:<left_rpm>,<right_rpm>\n`     | `M:30.5,-30.5\n`    |
| Arduino → RPi   | `E:<left_ticks>,<right_ticks>,<dt_ms>\n` | `E:1520,-1480,20\n` |

**Safety:** Motors stop if no command received within 500ms.
