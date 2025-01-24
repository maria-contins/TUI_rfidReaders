# ESP32 Dynamic Leader Election System

## Overview
This project involves multiple ESP32 devices that dynamically elect a leader to establish and maintain a connection with a middleware application. The system uses Bluetooth Low Energy (BLE) for external communication and ESP-NOW for message passing between devices.

This is used to create a tangible user interface (TUI).

## Features
- **Dynamic Leader Election**: ESP32 devices vote to elect a leader responsible for BLE communication with the middleware.
- **BLE Communication**: Only the leader uses BLE to connect with the external middleware application.
- **Message Passing**: Devices communicate through ESP-NOW, enabling efficient peer-to-peer communication.
- **RFID Integration**: Each ESP32 is connected to one or more RFID readers for data capture.
- **User Interface**: A button and OLED screen are used to display the current module state.

## Components
- **ESP32 Modules**: The main microcontroller units, responsible for device management, leader election, and communication.
- **RFID Readers**: Attached to each ESP32 to read RFID tags.
- **OLED Screen**: Displays the current state of the device.
- **Button**: Allows user interaction for state changes or resets.

## Workflow
1. **Leader Election**: Devices use a protocol to dynamically elect one as the leader based on certain criteria (e.g., signal strength or random selection).
2. **BLE Communication**: Once elected, the leader uses BLE to connect to the middleware, while other devices continue communicating through ESP-NOW.
3. **Message Passing**: Devices exchange data through ESP-NOW to synchronize or pass information related to RFID reads.
4. **User Interface**: The OLED screen shows the current state, and the button allows for user input to interact with the system.
