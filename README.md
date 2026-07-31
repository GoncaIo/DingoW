# Development of a Lightweight Hybrid Wheeled-Legged Robot

> **Master's Dissertation Project**  
> **Faculty of Engineering, University of Porto (FEUP)**  
> **Development of a Lightweight Hybrid Wheeled-Legged Robot**

This repository contains the hardware designs and software developed during my Master's dissertation at the Faculty of Engineering of the University of Porto (FEUP).

## Project Overview

Hybrid wheeled-legged robots combine the terrain adaptability of legged locomotion with the speed and efficiency of wheeled locomotion. The goal of this project is to develop a lightweight, cost-effective hybrid quadruped capable of traversing both smooth and rough terrain while remaining portable, energy-efficient and modular.

Potential applications include:

- Search and rescue
- Industrial inspection
- Environmental monitoring
- Reconnaissance

## Repository Structure

```
cad/
    CAD models and mechanical designs

software/
    ROS Noetic workspace (dingo_ws) and the Docker stack used to
    build and run it on the Raspberry Pi 5, plus start_robot.py,
    the PS4-button launcher that runs on the host

firmware/
    Microcontroller code: Raspberry Pi Pico wheel controller
    (MicroPython)

deploy/
    systemd unit that starts the launcher on boot

.devcontainer/
    VS Code dev container definition, wraps the same compose file
```

## Project Status

- Mechanical design
- Electronics integration
- Software development
- Experimental validation

## Acknowledgements

This project builds upon and adapts the excellent **DingoQuadruped** open-source project by Yerbert.

Original repository:
https://github.com/Yerbert/DingoQuadruped

The original project is distributed under the MIT License, and this repository complies with its licensing requirements.

## License

This project is released under the MIT License. See the `LICENSE` file for details.

## Author

**Gonçalo Paulino**

Master's Dissertation Project

Faculty of Engineering, University of Porto (FEUP)

2026