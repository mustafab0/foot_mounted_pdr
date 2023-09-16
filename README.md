# foot_mounted_pdr
This is a how to guide to build a foot mounted PDR device
## Overview
Pedestrian Dead-Reckoning (PDR) is a technique used to estimate the position and orientation of a person as they move. One of the fundamental components of PDR is an Inertial Navigation System (INS) that uses data from an Inertial Measurement Unit (IMU). While IMUs provide valuable data, they are prone to errors.  To address these challenges, a sophisticated technique is required. To improve the accuracy of the INS, an Error-State Extended Kalman Filter (EKF) is employed which corrects the errors in the INS everytime the foot is placed on the ground, hence foot mounted.
In this specific implementation, an RP2040 microcontroller and a BNO055 IMU are used to create a foot-mounted PDR system.
## Table of Contents
- [Getting Started](#getting-started)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
