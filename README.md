# Modular UAV Transmitter-Receiver System

A modular UAV control and telemetry platform designed to be adapted across different UAV platforms, with multi-channel wireless control and onboard flight-control capabilities. The system was also integrated with and used for PID tuning and flight testing on a quadcopter, validating its practical application in real-world UAV control.

## Transmitter

The transmitter provides multiple control channels for UAV operation, combining analog and digital inputs into a single control interface.

- **Slide potentiometer** — Thrust control
- **3 × Potentiometers** — PID parameter and yaw control
- **Joystick** — Roll and pitch control
- **Switches & push buttons** — Flight modes and custom mappings
- **OLED display & LEDs** — Real-time telemetry and system information
- **Speaker** — Alerts and audible telemetry announcements

## Receiver

The receiver provides the interface between the wireless control system and the UAV.

- **4 × PWM outputs** — ESC control
- **MPU9250** — IMU measurements for motion and orientation sensing
- **Buzzer** — System and flight alerts
- **PWM LED output** — Nighttime UAV tracking

## Wireless Communication

The system uses **RFM69HCW 434 MHz** modules with dipole antennas for high-range, line-of-sight communication.

The **14 dBm transmit power** and 434 MHz operating frequency are intended to provide reliable long-range communication and improved propagation around obstacles compared with higher-frequency links.

## Battery Monitoring

Both the transmitter and receiver include onboard **voltage-divider circuits** for continuous battery-voltage monitoring.

This enables real-time battery-status monitoring and supports low-battery protection and warning mechanisms.

## Safety & Failsafe Systems

The system incorporates multiple safety mechanisms for reliable UAV operation:

- **Arming/disarming failsafe**
- **Communication-loss failsafe**
- **Low-battery detection on the UAV**
- Controlled response to loss of communication
- Flight-state monitoring through the transmitter interface

