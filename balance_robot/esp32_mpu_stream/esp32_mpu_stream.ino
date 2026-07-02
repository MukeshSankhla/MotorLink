#include <Wire.h>

// I2C Address of MPU-6050 (usually 0x68, or 0x69 if AD0 pin is HIGH)
const int MPU_ADDR = 0x68;

const float ACCEL_SCALE = 16384.0;

const float GYRO_SCALE = 131.0;

const float ALPHA = 0.98;

float gyro_x_offset = 0.0;
float gyro_y_offset = 0.0;
float gyro_z_offset = 0.0;

// Orientation values
float pitch = 0.0;
float roll = 0.0;

unsigned long last_time = 0;

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
  Serial.println("[*] Calibrating gyroscope. KEEP ROBOT COMPLETELY STILL...");
  Serial1.println("[*] Calibrating gyroscope. KEEP ROBOT COMPLETELY STILL...");

  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  const int samples = 500;

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

  // Output calibration results to both standard USB serial and custom UART
  String calMsg =
      "[✓] Gyro calibration complete. Offsets: X=" + String(gyro_x_offset, 4) +
      " Y=" + String(gyro_y_offset, 4) + " Z=" + String(gyro_z_offset, 4);
  Serial.println(calMsg);
  Serial1.println(calMsg);
}

void setup() {
  // Initialize default USB Serial (for debugging/monitoring)
  Serial.begin(115200);

  // Initialize Serial1 on GPIO 21 (RX) and GPIO 22 (TX) for TTL UART stream
  Serial1.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  delay(500);

  Serial.println("\n===========================================");
  Serial.println(" ESP32 MPU-6050 UART Pitch Streamer (GPIO 21/22)");
  Serial.println("===========================================");
  Serial1.println("\n===========================================");
  Serial1.println(" ESP32 MPU-6050 UART Pitch Streamer (GPIO 21/22)");
  Serial1.println("===========================================");

  // Initialize I2C with alternate pins (GPIO 18 SDA, GPIO 19 SCL)
  // because GPIO 21 and 22 are now allocated for Serial1 UART.
  Wire.begin();
  Wire.setClock(400000); // 400kHz Fast Mode I2C

  // Try to connect to MPU-6050
  int retries = 5;
  while (!initMPU() && retries > 0) {
    Serial.println("[✗] MPU-6050 initialization failed! Retrying in 1s...");
    Serial1.println("[✗] MPU-6050 initialization failed! Retrying in 1s...");
    retries--;
    delay(1000);
  }

  if (retries == 0) {
    String errMsg =
        "[CRITICAL] MPU-6050 not detected. Check I2C wiring (SDA=18, SCL=19).";
    while (true) {
      Serial.println(errMsg);
      Serial1.println(errMsg);
      delay(2000);
    }
  }

  Serial.println("[✓] MPU-6050 detected and initialized.");
  Serial1.println("[✓] MPU-6050 detected and initialized.");

  // Calibrate Gyro
  calibrateGyro();

  last_time = micros();
  Serial.println(
      "[*] Starting UART streaming on GPIO 21/22. Loop target: 100Hz.");
  Serial1.println(
      "[*] Starting UART streaming on GPIO 21/22. Loop target: 100Hz.");
  Serial.println("--- STREAMING START ---");
  Serial1.println("--- STREAMING START ---");
}

void loop() {
  unsigned long current_time = micros();
  float dt = (float)(current_time - last_time) / 1000000.0;

  if (dt < 0.005) {
    delayMicroseconds(500);
    return;
  }

  last_time = current_time;

  // Request Accel and Gyro data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  if (Wire.available() >= 14) {
    int16_t raw_ax = (Wire.read() << 8) | Wire.read();
    int16_t raw_ay = (Wire.read() << 8) | Wire.read();
    int16_t raw_az = (Wire.read() << 8) | Wire.read();
    int16_t raw_temp = (Wire.read() << 8) | Wire.read();
    int16_t raw_gx = (Wire.read() << 8) | Wire.read();
    int16_t raw_gy = (Wire.read() << 8) | Wire.read();
    int16_t raw_gz = (Wire.read() << 8) | Wire.read();

    float ax = (float)raw_ax / ACCEL_SCALE;
    float ay = (float)raw_ay / ACCEL_SCALE;
    float az = (float)raw_az / ACCEL_SCALE;

    float gx = ((float)raw_gx / GYRO_SCALE) - gyro_x_offset;
    float gy = ((float)raw_gy / GYRO_SCALE) - gyro_y_offset;
    float gz = ((float)raw_gz / GYRO_SCALE) - gyro_z_offset;

    // Pitch is now mapped to rotation around the X-axis (forward/backward
    // tilt). We compute it using only the Y and Z axes of the accelerometer.
    // This isolates the measurement to the X-axis rotation plane.
    float pitch_acc = atan2(ay, az) * 180.0 / M_PI;

    // Complementary Filter: X-axis Gyro (pitch rate) + Accel Pitch
    pitch = ALPHA * (pitch + gx * dt) + (1.0 - ALPHA) * pitch_acc;
    roll = 0.0; // Disabled: we focus exclusively on the forward/backward axis

    // Build the data string including raw accelerometer (G) and gyroscope
    // (deg/s) values Note: We stream X-gyro (gx) as GYRO_Y and Y-gyro (gy) as
    // GYRO_X for Python script mapping.
    String dataStr = "PITCH:" + String(pitch, 2) + ",ROLL:" + String(roll, 2) +
                     ",GYRO_Y:" + String(gx, 2) + ",GYRO_X:" + String(gy, 2) +
                     ",ACC_X:" + String(ax, 3) + ",ACC_Y:" + String(ay, 3) +
                     ",ACC_Z:" + String(az, 3) + ",GYR_Z:" + String(gz, 2);

    // Write to standard Serial (USB) for debugging
    Serial.println(dataStr);

    // Write to Serial1 (GPIO 22 TX pin) for actual control stream
    Serial1.println(dataStr);
  }

  delay(10);
}
