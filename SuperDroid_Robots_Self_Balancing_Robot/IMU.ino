// the i2c.ino is used here to read the imu data

void beginIMU(){
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
 // Wire.setClock(400000UL); // Set I2C frequency to 400kHz
 
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  while (i2cWrite(0x6B, 0x80, true)); // Reset device, this resets all internal registers to their default values
  do {
    while (i2cRead(0x6B, i2cData, 1));
  } while (i2cData[0] & 0x80); // Wait for the bit to clear
  delay(5);
  while (i2cWrite(0x6B, 0x0A, true)); // PLL with Y axis gyroscope reference, disable temperature sensor and disable sleep mode


  i2cData[0] = 15; // Set the sample rate to 500Hz - 8kHz/(15+1) = 500Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, true)); // Write to all four registers at once
  
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3D, i2cData, 4));
  accY = (int16_t)((i2cData[2] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[3]);

  roll  = atan2(accY, accZ) * RAD_TO_DEG;

  kalmanX.setAngle(roll); // Set starting angle

  timer = micros();  
}


void ReadIMU(){
  /* Update all the values */
  while (i2cRead(0x3D, i2cData, 8));
  accY = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  gyroX = (int16_t)((i2cData[6] << 8) | i2cData[7]);

  dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  gyroXrate = gyroX / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  AngleInput = kalAngleX;  // Input for angle PID
 
}
