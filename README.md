# Off-Grid Emergency Messaging Using a Self-Healing ESP32-LoRa Mesh

## 📌 Project Overview

During natural disasters, in remote areas, and in emergency situations, conventional communication systems such as cellular networks and internet connectivity may become unavailable or unreliable. This creates challenges for people who need to communicate when traditional infrastructure is inaccessible.

This project explores an **off-grid wireless messaging system using ESP32 microcontrollers and LoRa communication modules**. Multiple nodes communicate wirelessly and forward messages across the network, providing a platform for studying multi-hop communication and resilient messaging without relying on cellular connectivity.

## 🎯 Objectives

- Develop a multi-node wireless messaging system using ESP32 and LoRa.
- Explore communication between nodes without internet or cellular connectivity.
- Implement message forwarding and duplicate-packet suppression.
- Support message delivery and read acknowledgements.
- Monitor neighboring nodes and investigate communication resilience.
- Explore the design challenges involved in building a self-healing mesh network.

## ⚙️ Technology Stack

- **Microcontroller:** ESP32
- **Wireless communication:** SX1278 LoRa module
- **Operating frequency:** 433 MHz
- **Development environment:** Arduino IDE
- **Display:** OLED interface
- **User input:** Rotary encoder
- **Network approach:** Multi-node communication with flood-style message forwarding

## 🏗️ System Architecture

The system consists of multiple ESP32-based nodes equipped with LoRa modules. Each node can participate in wireless communication, and messages can be forwarded through other nodes to extend communication beyond a single direct link.

The architecture is designed to explore multi-hop message delivery, acknowledgement handling, duplicate suppression, and node-availability monitoring.

Refer to the architecture diagram in the `docs/` directory for a visual overview of the system.

## ✨ Key Features

- **Off-grid communication:** Wireless messaging without depending on cellular networks or internet access.
- **Multi-node network:** Communication between multiple ESP32-LoRa devices.
- **Message forwarding:** Intermediate nodes can forward messages through the network.
- **Duplicate suppression:** Helps prevent repeated processing of the same message.
- **Acknowledgements:** Supports message delivery and read-status acknowledgements.
- **Node monitoring:** Tracks neighboring nodes to help investigate network availability.
- **Modular firmware:** A common firmware codebase can be configured with different node IDs.

## 📂 Repository Structure

- [📄 Read the Project Report](docs/Project_Report_.pdf)
- [🗺️ View the Network Architecture](docs/network_architecture.png)
- [💻 Explore the Arduino Firmware](firmware/src/mesh_node.ino).


## 🚀 Project Status

This repository documents a working prototype and its development. The project provides a practical exploration of embedded systems, LoRa-based wireless communication, and multi-node network design.

Automatic route recovery, end-to-end reliability, and self-healing behavior should be considered demonstrated only to the extent verified through testing.

## 👥 Collaboration

This project was developed collaboratively, combining embedded programming, wireless communication, hardware integration, and system design.


