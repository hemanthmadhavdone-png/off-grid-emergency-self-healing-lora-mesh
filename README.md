# Firmware Documentation

## Overview

This directory contains the common Arduino firmware for the four-node ESP32-LoRa mesh network.

## Node Configuration

All four ESP32 nodes use the same firmware source file. Each board is assigned its own node identity using the configuration mechanism implemented in the sketch.

|    Device    | Node identity |
| ESP32 Node 1 |     NODE_1    |
| ESP32 Node 2 |     NODE_2    |
| ESP32 Node 3 |     NODE_3    |
| ESP32 Node 4 |     NODE_4    |

Replace these example identities with the actual IDs used by the firmware.

## Upload Procedure

1. Open `src/mesh_node.ino` in Arduino IDE.
2. Install the required ESP32 board support and libraries.
3. Configure the appropriate node identity.
4. Select the correct ESP32 board and serial port.
5. Upload the sketch to the device.
6. Repeat for all four nodes, assigning a unique identity to each.

## Dependencies and Hardware

Document the LoRa library, ESP32 board configuration, pin mappings, frequency settings, and other dependencies used by the actual implementation.

## Notes

The firmware source is shared across all nodes. Avoid maintaining duplicate copies unless a node genuinely requires different functionality.
