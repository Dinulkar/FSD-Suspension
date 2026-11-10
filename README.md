# FSD-Suspension

![Project Status](https://img.shields.io/badge/Status-Completed-green)


This Simulink model uses a Quarter-Car plant to simulate Frequency Selective Damping. By integrating a high-pass filter, it dynamically adjusts the damping coefficient based on input frequency. This allows the system to reduce damping for high-frequency comfort while maintaining stiffness for low-frequency stability and handling.

## Mathematical Model (MATLAB/Simulink)
This repository contains a **2-DOF Quarter-Car Model** implemented in MATLAB and Simulink, specifically designed to simulate **Frequency Selective Damping (FSD)**. 

### Overview
Standard dampers use a fixed damping coefficient, forcing a compromise between handling and comfort. This model uses a **passive-adaptive hydraulic valve logic**—mathematically represented as a **transfer function-based filter**—to decouple high-frequency road noise from low-frequency body control.

### Key Features
*   **Dynamic Damping:** Implements a frequency-dependent $c(\omega)$ function.
*   **State-Space Integration:** Uses a plant model for sprung ($m_s$) and unsprung ($m_u$) masses.
*   **Frequency Filtering:** Includes a high-pass filter block to simulate the FSD bypass valve.
*   **Performance Analysis:** Scripts for comparing FSD vs. Standard dampers using Power Spectral Density (PSD) and RMS acceleration.

### Quick Start
1. Run `parameters.m` to load vehicle constants ($m, k, c, \tau$).
2. Open `fsd_suspension.slx` in Simulink.
3. Run the simulation to visualize the damping force response across varying road profiles.
