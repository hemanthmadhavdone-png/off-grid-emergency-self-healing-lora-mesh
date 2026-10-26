# Off-Grid Emergency Messaging Using a Self-Healing ESP32-LoRa Mesh

A four-node wireless communication project exploring off-grid emergency messaging using ESP32 microcontrollers and SX1278 LoRa modules.

## Project Overview

This project explores how a self-healing mesh network can support emergency messaging when conventional communication infrastructure is unavailable.

The network consists of four ESP32-based nodes communicating over LoRa. The firmware supports the message-handling and forwarding behavior implemented by our team.

## Objectives

- Build a four-node wireless communication prototype.
- Implement message forwarding between nodes.
- Investigate recovery from node or communication-link failures.
- Evaluate network behavior under different test conditions.

## Hardware Components

- ESP32 development boards
- SX1278 LoRa modules operating at 433 MHz
- Suitable antennas
- Breadboards, jumper wires, and power supplies

## Repository Structure

- `firmware/` — Arduino firmware and configuration documentation
- `docs/` — Project report and architecture diagrams
- `tests/` — Test procedures and measured results

## Getting Started

1. Install Arduino IDE and ESP32 board support.
2. Install the libraries required by the firmware.
3. Open the Arduino sketch in `firmware/src/`.
4. Configure the node ID and hardware settings according to the firmware documentation.
5. Select the appropriate ESP32 board and port, then upload the sketch.
6. Repeat for each node with a unique ID.

## System Architecture

The system uses four wireless nodes. Add the architecture diagram in `docs/architecture.png` once it is available.

## Testing and Results

Document the experiments performed, including normal message delivery, node failure, recovery behavior, packet delivery, and limitations.

Only report results that were observed during actual testing.

## Project Report

The detailed technical report will be available in `docs/project-report.pdf`.

## Contributors

Add the names and GitHub profiles of all team members here.

## Technologies

ESP32 · LoRa · Arduino IDE · Embedded Systems · Wireless Mesh Networking
