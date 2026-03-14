# VEX U Robotics Project - UDG

Welcome to the VEX U Robotics Project repository for the UDG organization! This repository contains the custom C++ firmware and software architecture designed for our VEX U competition robots. It is actively maintained by myself and members of the UDG team.

## Overview

This project leverages the [PROS](https://pros.cs.purdue.edu/) (Purdue Robotics Operating System) framework for the VEX V5 Brain, providing a robust, lightweight, and fast C++ environment for real-time robotic control.

Our codebase focuses on providing reliable autonomous and driver-control capabilities with a modular object-oriented approach.

## Repository Contents

*   **`src/`**: Contains the main C++ source code files for the robot's logic.
    *   `main.cpp`: The entry point for the PROS project. It defines the core behaviors for initialization, autonomous, and operator control (`opcontrol`) routines.
    *   `mylib/`: Custom library implementations for complex robot subsystems (e.g., `chassis.hpp` for drivetrain control).
*   **`include/`**: Header files for custom classes and libraries, as well as PROS and LVGL API headers.
*   **`firmware/`**: Compiled binaries and libraries (like `libpros.a` and `liblvgl.a`) required by the PROS toolchain.
*   **`project.pros`**: The configuration file for the PROS project detailing templates (e.g., `kernel@4.2.1` and `liblvgl@9.2.0`) and target environments.
*   **`Makefile` / `common.mk`**: Build system configurations to compile the project into the final `.bin` file for the V5 Brain.

## Key Features

*   **Custom Chassis Control**: A dedicated `Chassis` library module to handle complex motor groups and arcade drive functionality with minimal code duplication.
*   **Modular Architecture**: Designed for expandability to easily accommodate new sensors, mechanisms, and autonomous routines.
*   **PROS & LVGL Integration**: Fully integrated with the PROS API for low-level hardware access and the LVGL graphics library for advanced on-screen UI during matches.

## Getting Started

To compile and download this project to the VEX V5 Brain, you will need the PROS CLI installed.

1.  **Build the project**:
    ```bash
    pros build
    ```

2.  **Upload to the V5 Brain**:
    Ensure your VEX V5 Brain is connected via USB, then run:
    ```bash
    pros upload
    ```

## About Us

This repository represents the hard work and dedication of the **UDG** organization competing in the **VEX U** division. We strive to push the boundaries of collegiate robotics through innovative engineering and software design.
