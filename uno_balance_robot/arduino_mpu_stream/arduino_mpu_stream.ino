/*
 * arduino_mpu_stream.ino (Arduino Uno Q Version)
 *
 * Arduino Uno Q firmware that runs on the STM32 MCU. It reads from the MPU-6050
 * IMU over hardware I2C, performs gyroscope offset calibration, calculates
 * pitch using a Complementary Filter (X-axis rotation / Roll mapping), and
 * registers the telemetry data with the Qualcomm Linux MPU via the internal
 * Bridge library.
 *
 * Hardware Connections (Internal to Uno Q):
 *   MPU-6050 SDA -> Arduino Pin A4 (I2C SDA)
 *   MPU-6050 SCL -> Arduino Pin A5 (I2C SCL)
 */

#include "Arduino_RouterBridge.h"
#include <Wire.h>

// I2C Address of MPU-6050 (usually 0x68)
const int MPU_ADDR = 0x68;

// Scale factors for raw sensor data (at default ranges)
const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE = 131.0;

// Filter constant for complementary filter (alpha)
const float ALPHA = 0.98;

// Gyro calibration offsets (determined at startup)
float gyro_x_offset = 0.0;
float gyro_y_offset = 0.0;
float gyro_z_offset = 0.0;

// Orientation values
float pitch = 0.0;
unsigned long last_time = 0;

// String buffer to pass data across the Bridge to Linux (Python)
char data_buffer[90];

// Check connection to MPU-6050
bool initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Power management register (0x6B): wake up the sensor
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configure Gyroscope scale (0x1B): FS_SEL = 0 (+/- 250 deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  // Configure Accelerometer scale (0x1C): AFS_SEL = 0 (+/- 2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  return true;
}

// Perform gyro calibration by averaging readings while robot is static
void calibrateGyro() {
  Monitor.println(F("[*] Calibrating gyroscope. KEEP ROBOT STILL..."));

  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 300;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Gyro X starting register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);

    if (Wire.available() >= 6) {
      gx_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      gy_sum += (int16_t)(Wire.read() << 8 | Wire.read());
      gz_sum += (int16_t)(Wire.read() << 8 | Wire.read());
    }
    delay(3);
  }

  gyro_x_offset = (float)gx_sum / samples / GYRO_SCALE;
  gyro_y_offset = (float)gy_sum / samples / GYRO_SCALE;
  gyro_z_offset = (float)gz_sum / samples / GYRO_SCALE;

  Monitor.print(F("[✓] Gyro calibration complete. Offsets: X="));
  Monitor.print(gyro_x_offset, 2);
  Monitor.print(F(" Y="));
  Monitor.print(gyro_y_offset, 2);
  Monitor.print(F(" Z="));
  Monitor.println(gyro_z_offset, 2);
}

// Bridge API callback function requested by the Python script
const char *get_mpu_data() {
  unsigned long current_time = micros();
  float dt = (float)(current_time - last_time) / 1000000.0;
  if (dt < 0.001)
    dt = 0.001; // Guard against divide by zero or negative time
  last_time = current_time;

  // Request 14 registers from MPU-6050 (0x3B ACCEL_XOUT_H to 0x48 GYRO_ZOUT_L)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() >= 14) {
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    int16_t raw_temp = (Wire.read() << 8) | Wire.read(); // Temp (ignored)
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    float ax = (float)raw_ax / ACCEL_SCALE;
    float ay = (float)raw_ay / ACCEL_SCALE;
    float az = (float)raw_az / ACCEL_SCALE;

    float gx = ((float)raw_gx / GYRO_SCALE) - gyro_x_offset;
    float gy = ((float)raw_gy / GYRO_SCALE) - gyro_y_offset;
    float gz = ((float)raw_gz / GYRO_SCALE) - gyro_z_offset;

    // Pitch is rotation around the X-axis (forward/backward tilt)
    float pitch_acc = atan2(ay, az) * 180.0 / M_PI;

    // Complementary Filter: X-gyro (gx) + Accel Pitch
    pitch = ALPHA * (pitch + gx * dt) + (1.0 - ALPHA) * pitch_acc;

    // Convert values and format package using Arduino String (ensures STM32 compatibility)
    String dataStr = "PITCH:" + String(pitch, 2) +
                     ",ROLL:0.0" +
                     ",GYRO_Y:" + String(gx, 2) +
                     ",GYRO_X:" + String(gy, 2) +
                     ",ACC_X:" + String(ax, 3) +
                     ",ACC_Y:" + String(ay, 3) +
                     ",ACC_Z:" + String(az, 3) +
                     ",GYR_Z:" + String(gz, 2);

    // Copy to the data_buffer array for Bridge return
    strncpy(data_buffer, dataStr.c_str(), sizeof(data_buffer) - 1);
    data_buffer[sizeof(data_buffer) - 1] = '\0';
  }
  return data_buffer;
}

void setup() {
  // Start Bridge & Monitor
  Bridge.begin();
  Monitor.begin();

  Monitor.println(F("\n==========================================="));
  Monitor.println(F(" Arduino Uno Q MPU-6050 Bridge Streamer"));
  Monitor.println(F("==========================================="));

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000L);

  int retries = 5;
  while (!initMPU() && retries > 0) {
    Monitor.println(F("[✗] MPU-6050 failed! Retrying..."));
    retries--;
    delay(1000);
  }

  if (retries == 0) {
    Monitor.println(F("[CRITICAL] MPU-6050 not detected."));
    while (true) {
      delay(2000);
    }
  }

  Monitor.println(F("[✓] MPU-6050 detected and initialized."));

  // Calibrate Gyro
  calibrateGyro();

  // Provide the callback to the Router Bridge for Python access
  Bridge.provide_safe("get_mpu_data", get_mpu_data);
  Monitor.println(F("[✓] Bridge API registered. Ready."));

  last_time = micros();
}

void loop() {
  // The Bridge calls get_mpu_data callback directly in the background.
  // We keep loop empty to maximize responsiveness of the RPC link.
  delay(10);
}
