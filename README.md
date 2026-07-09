# CubeSat-Based Dark Vessel Detection Prototype

A CubeSat-inspired embedded communication system that detects **dark vessels** using wireless heartbeat monitoring. The system consists of an **Arduino-based Vessel Node**, an **STM32F103C8T6 Satellite Node**, and an **Arduino-based Ground Station** communicating through **NRF24L01** wireless transceivers. The satellite monitors vessel heartbeat packets, determines vessel status, calculates its orientation using an **MPU6050**, and transmits telemetry to the ground station.

---

# Project Overview

Illegal or unauthorized vessels often disable their communication systems to avoid detection, making maritime monitoring difficult. This project demonstrates a low-cost CubeSat-inspired communication architecture capable of detecting the presence or absence of a vessel using heartbeat packets.

The system is composed of three nodes:

- **Vessel Node (Arduino UNO):** Periodically transmits heartbeat packets.
- **Satellite Node (STM32F103C8T6):** Receives heartbeat packets, determines vessel status, reads orientation data from the MPU6050 sensor, and generates telemetry packets.
- **Ground Station (Arduino UNO):** Receives telemetry packets and displays vessel status along with satellite orientation.

---

# Objectives

- Design a CubeSat-inspired communication architecture.
- Detect vessel presence using heartbeat packets.
- Identify dark vessels when heartbeat packets are not received.
- Measure satellite orientation using MPU6050.
- Transmit telemetry wirelessly using NRF24L01.
- Display real-time telemetry at the ground station.

---

#  Hardware Used

| Component | Quantity |
|-----------|----------|
| STM32F103C8T6 | 1 |
| Arduino UNO | 2 |
| NRF24L01 + Antenna | 3 |
| MPU6050 | 1 |
| ST-Link Programmer | 1 |
| AMS1117 3.3V Regulator | 1 |
| Breadboards | 3 |
| Jumper Wires | As Required |
| Power Supply | As Required |

---

# Software Used

- STM32CubeIDE
- STM32 HAL Library
- Arduino IDE
- Embedded C
- SPI Communication
- I2C Communication
- UART Communication

---

# System Architecture

```
              HEARTBEAT (HB)
 ┌─────────────────────────────────────────┐
 │                                         │
 ▼                                         │
Vessel Node (Arduino) --------------------► Satellite (STM32)
                                              │
                                              │
                            Read MPU6050 (Pitch, Roll, Yaw)
                                              │
                                              ▼
                               Generate Telemetry Packet
                                              │
                                              ▼
                                 Ground Station (Arduino)
                                              │
                                              ▼
                                Display Serial Telemetry
```

---

# Working Principle

## 1. Vessel Node

- Arduino UNO acts as the vessel.
- Continuously transmits heartbeat packets using an NRF24L01 module.

Example:

```
HB:0
HB:1
HB:2
...
```

---

## 2. Satellite Node (STM32)

The STM32 performs the following operations:

1. Receives heartbeat packets from the vessel.
2. Determines whether the vessel is detected or classified as a dark vessel.
3. Reads accelerometer and gyroscope data from the MPU6050.
4. Calculates Pitch, Roll, and Yaw.
5. Creates a telemetry packet.
6. Transmits the telemetry packet to the ground station.

If heartbeat packets are not received for **5 seconds**, the vessel is marked as a **Dark Vessel**.

---

## 3. Ground Station

The Arduino UNO receives telemetry packets from the satellite and displays:

- Vessel Status
- Pitch
- Roll
- Yaw

on the Serial Monitor.

---

#  Telemetry Packet Format

The STM32 transmits telemetry packets in the following format:

```
V:1 P:-12 R:3 Y:15
```

Where:

| Field | Description |
|--------|-------------|
| V | Vessel Status (1 = Vessel Detected, 0 = Dark Vessel) |
| P | Pitch Angle (°) |
| R | Roll Angle (°) |
| Y | Yaw Angle (°) |

Example:

```
V:1 P:-12 R:3 Y:15
```

If heartbeat is lost:

```
V:0 P:-10 R:4 Y:16
```
---

#  Development Workflow

The project was developed incrementally through the following stages:

- NRF24L01 single-character transmission
- MPU6050 raw accelerometer testing
- MPU6050 raw gyroscope testing
- Accelerometer and gyroscope calibration
- Wireless transmission of MPU6050 data
- Satellite orientation (Pitch, Roll, Yaw) transmission
- Heartbeat-based vessel detection
- Complete Arduino-based prototype integration
- STM32 communication testing
- STM32 MPU6050 integration
- Final CubeSat-based Dark Vessel Detection Prototype

---

#  Communication Flow

## Vessel Node

- Transmits heartbeat packets every second.
- Address: **00001**

↓

## Satellite Node

- Receives heartbeat packets.
- Determines vessel status.
- Reads MPU6050.
- Calculates Pitch, Roll, Yaw.
- Generates telemetry packet.
- Transmits telemetry.
- Address: **00002**

↓

## Ground Station

Receives telemetry packet and displays:

- Vessel Status
- Pitch
- Roll
- Yaw

---

#  Example Output

```
Raw Packet:
V:1 P:-12 R:3 Y:15

Vessel Status : VESSEL DETECTED
Pitch : -12
Roll : 3
Yaw : 15
```

If no heartbeat is received:

```
Raw Packet:
V:0 P:-10 R:4 Y:16

Vessel Status : DARK VESSEL
Pitch : -10
Roll : 4
Yaw : 16
```

---

#  Future Improvements

- Replace NRF24L01 with LoRa or satellite communication modules.
- Integrate GPS for vessel tracking.
- Add cloud-based telemetry logging.
- Develop a graphical monitoring dashboard.
- Improve orientation estimation using sensor fusion (Complementary/Kalman Filter).

---

**Technologies:** STM32 • Arduino • Embedded C • NRF24L01 • MPU6050 • SPI • I2C • UART

---

## ⭐ If you found this project useful, consider giving it a Star!
