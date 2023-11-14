# foot_mounted_pdr
This is a how to guide to build a foot mounted PDR device
## Overview
Pedestrian Dead-Reckoning (PDR) is a technique used to estimate the position and orientation of a person as they move. One of the fundamental components of PDR is an Inertial Navigation System (INS) that uses data from an Inertial Measurement Unit (IMU). While IMUs provide valuable data, they are prone to errors.  To address these challenges, a sophisticated technique is required. To improve the accuracy of the INS, an Error-State Extended Kalman Filter (EKF) is employed which corrects the errors in the INS everytime the foot is placed on the ground, hence foot mounted.
In this specific implementation, an RP2040 microcontroller and a BNO055 IMU are used to create a foot-mounted PDR system.
## Table of Contents
- [Getting Started](#getting-started)
### Hardware Required
- [ ] Acquire the necessary hardware components:
  - RP2040 microcontroller
  - SD Card reader
  - LED indicator
  - Button 
  - You can find additional files to print in th CAD folder (Step files are available for edit)

### Software Setup
- [ ] Clone or download the project repository from GitHub to your local machine.

- [ ] Ensure you have the env installed in the 'env.yaml' file
  - Arduino IDE is required for the hardware device
  
- [ ] Compile and upload the project code to the RP2040 microcontroller

- [Usage](#usage)
#### Hardware Setup

1. **Insert SD Card and Power On:**
   - Insert an SD card into the designated slot on the device.
   - Plug in the power from a portable power bank.

2. **Calibration Process:**
   - When the power is plugged in, a blue LED will turn on and start blinking, indicating calibration mode.
   - Keep your foot steady for 5 seconds during the blinking period.
   - The LED will then blink steadily, representing the percentage of calibration.
   - Alternate your foot in different ankle positions at a 45-degree angle to calibrate the accelerometer.
   - The LED will start blinking faster during this motion.
   - If the LED slows down, continue performing the motion until the LED becomes a stable red.
   - At this point, the device is calibrated and ready to use.

#### Recording Data

1. **Positioning:**
   - Stand at the starting point where you want to record data.

2. **Initiate Recording:**
   - Press the button on the device.

3. **Status Indicator:**
   - The LED will turn green, indicating that the device is now recording data.

4. **Recording Data:**
   - Walk or move around as needed. The device will record relevant data during this time.

5. **Stop Recording:**
   - To stop recording, press the button again.

#### Data Analysis

After recording data, follow these steps for analysis:

1. **Retrieve Data from the SD card**

2. **Analysis Tool:**
   - Utilize the `ekf_oop.ipynb` notebook for data analysis.
   - This notebook provides tools and scripts for processing and interpreting the recorded data.

3. **Follow Notebook Instructions:**
   - Open the `ekf_oop.ipynb` notebook in a Jupyter environment.
   - Follow the instructions within the notebook for loading and analyzing the recorded data.

4. **Output:**
   - The notebook will generate plot that shows X,Y position  (plan) of the test run.

5. **Adjust Parameters (if needed):**
   - Depending on the analysis, you may need to adjust parameters of the KF.

6. **Feedback and Contributions:**
   - If you discover improvements or have feedback on the data analysis process, consider contributing to the project.

### Additional Tips
- Ensure the device has a stable and secure fit on your foot during operation.
- Periodically check the battery level on the power bank to avoid data loss due to sudden power failure.
- Calibrate the device in a static position before each use for optimal performance.

- [Contributing](#contributing)
Thank you for using this project. There are a few tweaks that have to be made but I am trying to iron out as much detail as possible. 
Next Steps are to make the EKF run as an embedded system on the microcontroller.
Follow similar guidelines to contribute to this project as the general conduct.

- [References](#References)
- Foxlin, E. (2005). Pedestrian tracking with shoe-mounted inertial sensors. EEE Computer Graphics and Applications, 38 - 46
- Fischer, C., Sukumar, P. T., & Hazas, M. (2012). Tutorial: Implementing a Pedestrian Tracker Using Inertial Sensors. IEEE Pervasive Computing, 17 - 27.
