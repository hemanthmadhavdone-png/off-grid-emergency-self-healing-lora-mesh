# Firmware Documentation

## Overview

This directory contains the common Arduino firmware for the four-node ESP32-LoRa mesh network.

## Node Configuration

All four ESP32 nodes use the same firmware source file. Each board is assigned a unique node ID.

- **Node 1:** `NODE_1`
- **Node 2:** `NODE_2`
- **Node 3:** `NODE_3`
- **Node 4:** `NODE_4`

*Note: These are example IDs. Replace them with the actual IDs used in the firmware.*

## Upload Instructions

1. Open `mesh_node.ino` in Arduino IDE.
2. Install the required ESP32 board support and libraries.
3. Configure the node ID for the board.
4. Select the correct ESP32 board and serial port.
5. Upload the firmware.
6. Repeat for the other three nodes, assigning each a unique ID.

## Dependencies and Hardware

Document the LoRa library, ESP32 board configuration, pin mappings, frequency settings, and other dependencies used by the actual implementation.

## Notes

The firmware source is shared across all nodes. Avoid maintaining duplicate copies unless a node genuinely requires different functionality.
