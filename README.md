# Octopod Robot Spider Controller

A Raspberry Pi-powered controller for an 8-legged robot spider with 3 DOF per leg using MG996R servos.

## Features

- Inverse Kinematics (IK) based on 3-DOF leg model
- Tripod gait for efficient walking
- Modular and extensible design

## Requirements

- Raspberry Pi (3B+/4)
- PCA9685 PWM driver
- 24x MG996R servos
- External power supply (LiPo recommended)

## Installation

```bash
pip install -r requirements.txt
python3 octopod.py
