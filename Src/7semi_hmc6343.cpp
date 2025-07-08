/********************************************************
 * @file 7semi_hmc6343.cpp
 *
 * @mainpage 7semi HMC6343 3-Axis Compass Sensor Library
 *
 * @section intro_sec Introduction
 *
 * This is the Arduino library for the HMC6343 3-axis digital compass sensor.
 * It supports tilt-compensated heading, magnetometer, accelerometer, temperature readings.
 *
 * This library is designed to work with I2C communication. Two pins (SCL and SDA)
 * are required to interface with the sensor.
 *
 * This library is developed and maintained by 7semi, intended for prototyping and evaluation
 * purposes. It may not be fully calibrated or accurate for critical applications.
 *
 * @section features Features
 * - Read heading, pitch, and roll
 * - Read magnetometer and accelerometer axes
 * - Read internal temperature
 * - Enable tilt orientation mode
 * - Enable/disable temperature filtering
 * - Sleep and wake support for low-power applications
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
 
 #include "7semi_hmc6343.h"

// Constructor: Initialize with given I2C address
hmc6343_7semi::hmc6343_7semi(uint8_t i2c_address)
  : _addr(i2c_address) {}

// Initialize the sensor
bool hmc6343_7semi::begin() {
  Wire.begin();
  delay(500); // Allow time for sensor to power up

  Wire.beginTransmission(_addr);
  Wire.write(READ_EEPROM_REG);
  delay(1);
  Wire.write(SLAVE_REG);
  Wire.endTransmission();

  Wire.requestFrom(_addr, (uint8_t)1);
  if (Wire.available()) {
    uint8_t data = Wire.read();
    return (data == FACTORY_VALUE);  // Check if sensor is responding with correct ID
  }
  return false;
}

// Read 3x16-bit values from given register (usually 6 bytes total)
uint16_t* hmc6343_7semi::i2c_read(uint8_t reg) {
  static uint8_t raw_data[6];
  static uint16_t data[3] = { 0, 0, 0 };

  if (writeReg(reg)) {
    delay(1);
    Wire.requestFrom(_addr, (uint8_t)6);
    for (int i = 0; i < 6; i++) {
      raw_data[i] = Wire.read();
    }
    for (int i = 0; i < 3; i++) {
      data[i] = ((raw_data[i * 2] << 8) | raw_data[i * 2 + 1]);
    }
    return data;
  }
  return nullptr;
}

// Read accelerometer data (X, Y, Z)
bool hmc6343_7semi::readAccel(int16_t &accelX, int16_t &accelY, int16_t &accelZ) {
  uint16_t* accel_data = i2c_read(ACCEL_REG);
  if (accel_data) {
    accelX = accel_data[0];
    accelY = accel_data[1];
    accelZ = accel_data[2];
    return true;
  }
  return false;
}

// Read magnetometer data (X, Y, Z)
bool hmc6343_7semi::readMag(int16_t &magX, int16_t &magY, int16_t &magZ) {
  uint16_t* mag_data = i2c_read(MAG_REG);
  if (mag_data) {
    magX = mag_data[0];
    magY = mag_data[1];
    magZ = mag_data[2];
    return true;
  }
  return false;
}

// Read heading, pitch, and roll
bool hmc6343_7semi::readHeading(int16_t &heading, int16_t &pitch, int16_t &roll) {
  int16_t* heading_data = (int16_t*)i2c_read(HEADING_REG);
  if (heading_data) {
    heading = heading_data[0] / 10; // Convert to degrees
    pitch = heading_data[1] / 10;
    roll = heading_data[2] / 10;
    return true;
  }
  return false;
}

// Read temperature in Celsius
bool hmc6343_7semi::readTemperature(float &tempC) {
  uint16_t* temp_data = i2c_read(TILT_REG);
  if (temp_data) {
    tempC = temp_data[2] / 10.0; // Convert to °C
    return true;
  }
  return false;
}

// Enter magnetic calibration mode
void hmc6343_7semi::enterCalibration() {
  writeReg(ENTER_CAL_REG);
  delay(300); // Wait for sensor to enter calibration
}

// Exit calibration mode
void hmc6343_7semi::exitCalibration() {
  writeReg(EXIT_CAL_REG);
  delay(50);
}

// Software reset the device
void hmc6343_7semi::reset() {
  writeReg(RESET_REG);
  delay(500); // Allow time to reboot
}

// Set magnetic deviation angle (offset correction)
void hmc6343_7semi::setDeviationAngle(float angle_deg) {
  writeEEPROM(ANGLE_DEVIATION_REG, (int16_t)(angle_deg * 10)); // Convert degrees to deci-degrees
}

// Set magnetic variation angle (declination)
void hmc6343_7semi::setVariationAngle(float angle_deg) {
  writeEEPROM(ANGLE_VARIATION_REG, (int16_t)(angle_deg * 10));
}

// Write a 16-bit value to two EEPROM registers
void hmc6343_7semi::writeEEPROM(uint8_t reg, int16_t value) {
  // Write low byte
  Wire.beginTransmission(_addr);
  Wire.write(WRITE_REG);
  Wire.write(reg);
  Wire.write(value & 0xFF); // LSB
  Wire.endTransmission();
  delay(10);

  // Write high byte
  Wire.beginTransmission(_addr);
  Wire.write(WRITE_REG);
  Wire.write(reg + 1);
  Wire.write((value >> 8) & 0xFF); // MSB
  Wire.endTransmission();
  delay(10);
}

// Set orientation mode: 0 = level, 1 = tilt-compensated
void hmc6343_7semi::setOrientationMode(uint8_t mode) {
  Wire.beginTransmission(_addr);
  Wire.write(WRITE_REG);
  Wire.write(0x04); // Orientation register
  Wire.write(mode);
  Wire.endTransmission();
  delay(10);
}

// Enable or disable temperature filtering
void hmc6343_7semi::enableTempFilter(bool enable) {
  Wire.beginTransmission(_addr);
  Wire.write(WRITE_REG);
  Wire.write(0x06); // Temp filter register
  Wire.write(enable ? 1 : 0); // 1 = enable
  Wire.endTransmission();
  delay(10);
}

// Enter low-power sleep mode
void hmc6343_7semi::sleep() {
  writeReg(0x94); // Sleep command
  delay(100);
}

// Wake from sleep mode
void hmc6343_7semi::wake() {
  writeReg(0xA0); // Wake command
  delay(100);
}

// Send a single byte command to the sensor
bool hmc6343_7semi::writeReg(uint8_t reg) {
  Wire.beginTransmission(_addr);
  Wire.write(reg);
  return (Wire.endTransmission() == 0); // Return true if success
}
