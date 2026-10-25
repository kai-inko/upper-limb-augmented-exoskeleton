#include "IMU_GY521.h"

IMU_GY521::IMU_GY521(uint8_t i2cAddr)
    : address(i2cAddr),
      sensorConnected(false),
      gyroBiasY(0.0f),
      angleOffsetDeg(0.0f),
      filteredAngleDeg(90.0f),
      currentGyroRate(0.0f),
      lastUpdateTimeMicros(0),
      isFirstRun(true) {}

bool IMU_GY521::begin() {
    Wire.begin();
    Wire.setClock(400000); // 400kHz Fast I2C mode on Arduino Nano

    // Test connection by reading WHO_AM_I register (0x75)
    Wire.beginTransmission(address);
    Wire.write(0x75);
    if (Wire.endTransmission(true) != 0) {
        sensorConnected = false;
        return false;
    }

    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        uint8_t c = Wire.read();
        if (c != 0x68 && c != 0x70 && c != 0x72) { // 0x68 is default MPU6050 WHO_AM_I
            // Unexpected device signature, but may be compatible clone
        }
    }

    // Wake up MPU-6050 (clear SLEEP bit in PWR_MGMT_1)
    Wire.beginTransmission(address);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(0x00);
    if (Wire.endTransmission(true) != 0) {
        sensorConnected = false;
        return false;
    }
    delay(10);

    // Set Accelerometer full-scale range: ±4g (FS_SEL = 1) -> 8192 LSB/g
    Wire.beginTransmission(address);
    Wire.write(MPU6050_REG_ACCEL_CFG);
    Wire.write(0x08);
    Wire.endTransmission(true);

    // Set Gyroscope full-scale range: ±500 deg/s (FS_SEL = 1) -> 65.5 LSB/(deg/s)
    Wire.beginTransmission(address);
    Wire.write(MPU6050_REG_GYRO_CFG);
    Wire.write(0x08);
    Wire.endTransmission(true);

    sensorConnected = true;
    lastUpdateTimeMicros = micros();
    return true;
}

void IMU_GY521::calibrateGyro(int samples) {
    if (!sensorConnected) return;

    long sumGy = 0;
    int validSamples = 0;

    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        if (readRawData(ax, ay, az, gx, gy, gz)) {
            sumGy += gy;
            validSamples++;
        }
        delay(3);
    }

    if (validSamples > 0) {
        // Gyro sensitivity at ±500 dps is 65.5 LSB per deg/s
        gyroBiasY = (float)sumGy / (float)validSamples / 65.5f;
    }
}

bool IMU_GY521::readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
    Wire.beginTransmission(address);
    Wire.write(MPU6050_REG_ACCEL_XOUT);
    if (Wire.endTransmission(false) != 0) {
        sensorConnected = false;
        return false;
    }

    // Burst read 14 bytes: Ax(2), Ay(2), Az(2), Temp(2), Gx(2), Gy(2), Gz(2)
    uint8_t bytesReceived = Wire.requestFrom(address, (uint8_t)14, (uint8_t)true);
    if (bytesReceived != 14) {
        sensorConnected = false;
        return false;
    }

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());
    int16_t rawTemp = (int16_t)((Wire.read() << 8) | Wire.read()); // unused
    (void)rawTemp;
    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());

    sensorConnected = true;
    return true;
}

bool IMU_GY521::update() {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRawData(ax, ay, az, gx, gy, gz)) {
        return false;
    }

    unsigned long now = micros();
    float dt = (now - lastUpdateTimeMicros) / 1000000.0f;
    lastUpdateTimeMicros = now;

    // Guard against timer rollover or initial glitch
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.02f; // Default 50Hz period
    }

    // Convert raw gyro Y to degrees per second (elbow pitch axis)
    currentGyroRate = ((float)gy / 65.5f) - gyroBiasY;

    // Calculate inclination angle from Accelerometer:
    // With GY-521 mounted flat on the forearm along the longitudinal axis:
    // When forearm is horizontal: pitch = 90 deg.
    // When forearm hangs straight down: pitch = 0 deg.
    // When forearm is flexed upward: pitch > 90 deg.
    float accelPitchDeg = atan2((float)ax, sqrt((float)ay * ay + (float)az * az)) * (180.0f / PI) + 90.0f;

    if (isFirstRun) {
        filteredAngleDeg = accelPitchDeg;
        isFirstRun = false;
    } else {
        // High-performance Complementary Filter:
        // 96% weight on integrated gyro angular rate (eliminates high-frequency acceleration vibrations)
        // 4% weight on accelerometer pitch (eliminates long-term drift)
        const float alpha = 0.96f;
        filteredAngleDeg = alpha * (filteredAngleDeg + currentGyroRate * dt) + (1.0f - alpha) * accelPitchDeg;
    }

    return true;
}
