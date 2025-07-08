/*******************************************************
 * @file Basic_HMC6343_Example.ino
 *
 * @brief Basic example demonstrating usage of the 7semi HMC6343 library.
 *
 * This example initializes the HMC6343 3-axis digital compass sensor and reads
 * heading, pitch, and roll in a loop.
 *
 * Key features demonstrated:
 * - Tilt-compensated orientation mode
 * - Magnetic declination (variation) adjustment
 * - Deviation angle compensation
 *
 * @note This example requires the 7semi HMC6343 library to be installed.
 *
 * @section author Author
 * Written by 7semi
 *
 * @section license License
 * @license MIT
 * Copyright (c) 2025 7semi
 *******************************************************/
 
 #include <7semi_hmc6343.h>

hmc6343_7semi compass;

void setup() {
  Serial.begin(9600);
  if (compass.begin()) {
    Serial.println("HMC6343 Initialized");

    compass.setOrientationMode(0x01);    // Tilt orientation
    compass.enableTempFilter(true);      // Enable temperature filter
    compass.setVariationAngle(2.5);      // Magnetic declination example
    compass.setDeviationAngle(0.0);      // Mechanical offset (if needed)

  } else {
    Serial.println("HMC6343 Init Failed");
    while (1);
  }
}

void loop() {
  int16_t heading, pitch, roll;
  float tempC;

  if (compass.readHeading(heading, pitch, roll)) {
    Serial.print("Heading: "); Serial.print(heading);
    Serial.print(" Pitch: ");  Serial.print(pitch);
    Serial.print(" Roll: ");   Serial.println(roll);
  }
  delay(1000);
}
