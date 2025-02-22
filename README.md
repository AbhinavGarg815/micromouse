# Micromouse
A complete solution for [micromouse](https://en.wikipedia.org/wiki/Micromouse) development including STM32-based hardware implementation and Micromouse Simulator integration. The project implements the Floodfill algorithm for efficient maze solving.

## Overview
This repository contains two main components:
- STM32-based Micromouse firmware for physical robot implementation using C
- Integration with [mms](https://github.com/mackorone/mms.git) (Micromouse Simulator) for algorithm testing and development

## Features
### Hardware Implementation
- Compatible with STM32F4xx series microcontrollers
- Motor control with PID feedback loops
- Sensor interface for wall detection
- Maze mapping using Floodfill algorithm
- Real-time maze solving capabilities
- Written entirely in C

### Simulator Integration
- Complete integration with MMS simulator
- Maze generation and exploration algorithms

### Floodfill Algorithm Implementation
The maze-solving logic uses the Floodfill algorithm with the following features:
- Dynamic distance calculation from current position to target
- Efficient wall memory mapping
- Automatic path optimization
- Dead-end detection and backtracking
- Update of distance values during exploration

## Prerequisites
- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [mms](https://github.com/mackorone/mms.git) (Micromouse Simulator)

## Project Structure
```
micromouse/
├── stm32Main.c             # STM32 firmware code
├── mmsMain.c               # Simulator code
└── assets/             
```
