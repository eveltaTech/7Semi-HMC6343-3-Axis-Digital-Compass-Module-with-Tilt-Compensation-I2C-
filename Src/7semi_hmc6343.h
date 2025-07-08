/*******************************************************
 * @file 7semi_hmc6343.h
 *
 * @mainpage 7semi HMC6343 3-Axis Compass Sensor Library
 *
 * @section intro_sec Introduction
 *
 * This is the Arduino library for the HMC6343 3-axis digital compass sensor,
 * supporting heading, pitch, roll, temperature, and low-power features.
 *
 * The library communicates with the sensor over I2C and provides convenient methods
 * to access raw and processed sensor data, as well as configuration options.
 *
 * @section usage Usage
 *
 * Include this header in your Arduino sketch to access the HMC6343 functions.
 * Initialize the sensor using `begin()` and then call available methods like
 * `readHeading()`, `readAccel()`, or `sleep()`.
 *
 * @section author Author
 *
 * Developed by 7semi.
 *
 * @section license License
 *
 * @license MIT
 * Copyright (c) 2025 7semi
 ******************************************************/
 
#ifndef _7SEMI_HMC6343_H_
#define _7SEMI_HMC6343_H_

#include <Wire.h>
#include "Arduino.h"

// Default I2C address
#define HMC6343_DEFAULT_ADDR      0x19

// Command registers
#define READ_EEPROM_REG           0xE1
#define WRITE_REG                 0xF1
#define SLAVE_REG                 0x00
#define FACTORY_VALUE             0x32

// Data registers
#define ACCEL_REG                 0x40
#define MAG_REG                   0x45
#define HEADING_REG               0x50
#define TILT_REG                  0x55

// Control commands
#define ENTER_CAL_REG             0xF0
#define EXIT_CAL_REG              0xF5
#define RESET_REG                 0x82

// EEPROM configuration registers
#define ANGLE_DEVIATION_REG       0x0A
#define ANGLE_VARIATION_REG       0x0C

class hmc6343_7semi {
public:
  hmc6343_7semi(uint8_t i2c_address = HMC6343_DEFAULT_ADDR);

  bool begin();
  bool readAccel(int16_t &accelX, int16_t &accelY, int16_t &accelZ);
  bool readMag(int16_t &magX, int16_t &magY, int16_t &magZ);
  bool readHeading(int16_t &heading, int16_t &pitch, int16_t &roll);
  bool readTemperature(float &tempC);

  void enterCalibration();
  void exitCalibration();
  void reset();

  void setDeviationAngle(float angle_deg);
  void setVariationAngle(float angle_deg);
  void setOrientationMode(uint8_t mode);
  void enableTempFilter(bool enable);

  void sleep();
  void wake();

private:
  uint8_t _addr;

  bool writeReg(uint8_t reg);
  void writeEEPROM(uint8_t reg, int16_t value);
  uint16_t* i2c_read(uint8_t reg);
};

#endif  // _7SEMI_HMC6343_H_
