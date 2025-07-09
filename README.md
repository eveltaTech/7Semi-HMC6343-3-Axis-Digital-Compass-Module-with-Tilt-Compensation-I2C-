# 7Semi-HMC6343-3-Axis-Digital-Compass-Module-with-Tilt-Compensation-I2C Arduino Library
 
This Arduino library provides support for the **HMC6343** 3-axis digital compass sensor over I2C. It allows reading of tilt-compensated heading, pitch, and roll, and includes features like magnetic variation correction and low-power control.
 
 
## Hardware Required
 
- HMC6343 sensor module  

- Arduino-compatible board  

- I2C connection (SDA, SCL)  
 
 
## Getting Started
 
  ### 1. Library Installation
 
      - Download or clone the repository.

      - Copy the files `7semi_hmc6343.cpp` and `7semi_hmc6343.h` into a folder named `7semi_HMC6343` under your Arduino `libraries/` directory.

      - Restart the Arduino IDE.
 
  ### 2. Wiring
 
        | HMC6343 Pin | Arduino Pin |

        |-------------|-------------|

        | SDA         | A4 (Uno)    |

        | SCL         | A5 (Uno)    |

        | VCC         | 3.3V / 5V   |

        | GND         | GND         |
 

  ## Output

  HMC6343 Initialized

  Heading: 132 Pitch: -5 Roll: 3

  Heading: 133 Pitch: -6 Roll: 2

  Heading: 134 Pitch: -6 Roll: 1

  Heading: 135 Pitch: -7 Roll: 2

  Heading: 134 Pitch: -6 Roll: 2

 
