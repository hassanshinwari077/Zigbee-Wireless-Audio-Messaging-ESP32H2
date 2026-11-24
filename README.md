# Zigbee-Based Wireless Audio Messaging System Using ESP32-H2

## Overview
This project implements a standalone wireless voice messaging system using two ESP32-H2 development boards communicating through Zigbee.

The transmitter records a short voice message using an INMP441 I2S microphone. The audio is stored as 16-bit PCM data and transmitted over Zigbee in chunks. The receiver reconstructs the complete audio message and plays it through a MAX98357A I2S audio amplifier and speaker.

## Hardware
- 2 × ESP32-H2 development boards
- INMP441 I2S digital microphone
- MAX98357A I2S audio amplifier
- 8 Ω speaker
- Breadboard and jumper wires
- USB power supplies

## Current Specifications
- Sampling rate: 16 kHz
- Audio format: 16-bit PCM
- Recording duration: 2 seconds
- Audio size: 64,000 bytes
- Zigbee transfer: 250 × 256-byte chunks
- Current transfer time: approximately 27 seconds

## System Flow
Voice → INMP441 → ESP32-H2 Producer → Audio Buffer → Zigbee → ESP32-H2 Consumer → Audio Reconstruction → MAX98357A → Speaker

## Software
- ESP-IDF
- ESP Zigbee SDK
- FreeRTOS
- I2S standard driver
- Custom Zigbee data-stream implementation

## Project Status
The complete prototype is operational. Voice recording, Zigbee transmission, message reconstruction, and speaker playback have been successfully demonstrated.

## Future Improvements
- Reduce Zigbee transfer latency
- Increase speaker output volume
- Support longer recordings
- Add persistent message storage
- Develop a multi-node Zigbee mesh network
- Add message indicators and user controls

## Third-Party Resources
Espressif ESP-IDF, ESP Zigbee SDK examples, FreeRTOS, manufacturer datasheets, and AI-assisted development/debugging tools were used during development.
