# Automotive CAN Bus Simulator & DAQ System

### 📂 Quick Navigation
* [👉 Click here to view the bare-metal STM32 C code (main.c)](./CAN_Loopback_Project/Core/Src/main.c)
* [👉 Click here to view the Arduino DAQ C++ code](./Arduino_Dashboard/Arduino_Dashboard.ino)



![C](https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white)
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![STM32](https://img.shields.io/badge/STM32-03234B?style=for-the-badge&logo=stmicroelectronics&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)

## Overview
This project is a bare-metal embedded systems implementation of an automotive Drive-by-Wire (Electronic Throttle Control) network. It utilizes a bi-directional CAN 2.0A bus operating at 500 kbps to bridge an STM32 ARM Cortex-M microcontroller (acting as the Engine Control Unit) and an ATmega328P (acting as the Digital Dashboard and Data Acquisition node).

The system simulates real-time analog signal processing, physical hardware interrupts, priority-based fault arbitration, and outputs a continuous CSV telemetry stream for graphical analysis.

## System Architecture

### Node 1: Engine Control Unit (STM32 Nucleo)
* **Hardware:** STM32 Nucleo + SN65HVD230 3.3V CAN Transceiver.
* **Function:** Samples a 10kΩ potentiometer using the internal 12-bit SAR ADC to simulate a physical gas pedal. 
* **Processing:** Maps the raw ADC value (0-4095) to an RPM range (1000-7000) and packs it into a 16-bit payload using bitwise operations.
* **Fault Arbitration:** Implements continuous redline monitoring. If RPM exceeds 6500, the ECU immediately ceases standard telemetry (ID `0x103`) and transmits a high-priority hardware fault frame (ID `0x001`). Due to CAN physical layer arbitration (dominant vs. recessive bits), this fault frame inherently overrides lower-priority network traffic.

### Node 2: Digital Dash & DAQ (Arduino)
* **Hardware:** ATmega328P + MCP2515 5V SPI CAN Module.
* **Function:** Listens to the CAN bus using external Hardware Interrupts (INT pin) to prevent CPU blocking, ensuring zero dropped frames.
* **User Input:** Utilizes an internal pull-up resistor (`INPUT_PULLUP`) to read a physical momentary push-button, transmitting a "Sport Mode" state toggle back to the STM32 (ID `0x200`).
* **Data Acquisition (DAQ):** Reconstructs the high/low bytes of the CAN payload and serializes the system timestamp, RPM, Sport Mode state, and Fault state into a real-time CSV data stream for spreadsheet graphing.

## Key Engineering Concepts Demonstrated
* **Bare-Metal C / HAL:** Direct manipulation of microcontroller peripherals without an RTOS.
* **Hardware Interrupts (ISR):** Decoupling critical asynchronous events from the `while(1)` polling loop.
* **CAN Protocol:** Baud rate configuration (Prescalers & Time Quanta), Mailbox management, standard 11-bit Identifiers, and bus arbitration physics.
* **Bitwise Manipulation:** Shifting (`<<`) and masking (`|`) bytes to package multi-byte integers into 8-bit CAN data frames.
* **Pointer Memory Management:** Passing hardware struct addresses by reference to prevent stack overflow and directly access memory-mapped registers.

## Hardware Bill of Materials (BOM)
* STM32 Nucleo Development Board (F446RE)
* Arduino Nano / Uno
* SN65HVD230 CAN Bus Transceiver (3.3V Logic)
* MCP2515 CAN Bus Module with TJA1050 Transceiver (5V Logic)
* 10kΩ Rotary Potentiometer
* Push Button


![IMG_5553 - Edited](https://github.com/user-attachments/assets/2d2e0417-9ae8-4772-839f-30f3fc8b5021)
<img width="738" height="516" alt="Screenshot 2026-02-24 110205" src="https://github.com/user-attachments/assets/78de52ce-9a4d-435a-acf1-d40f022b3cc6" />
[Book1 (version 1).csv]()
