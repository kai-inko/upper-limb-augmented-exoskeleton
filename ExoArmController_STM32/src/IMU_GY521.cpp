#include "IMU_GY521.h"
#include "Config.h"   // PIN_MPU_SDA / PIN_MPU_SCL / I2C_CLOCK_HZ

// Read one big-endian 16-bit word from the I2C RX buffer.
// NOTE: do NOT write (Wire.read() << 8) | Wire.read() -- C++ leaves the evaluation
// order of the two operands unspecified. AVR GCC happened to evaluate left-to-right;
// ARM GCC (STM32) is free to evaluate the right operand first, which swaps the bytes.
static inline int16_t readInt16BE() {
    uint8_t hi = (uint8_t)Wire.read();
    uint8_t lo = (uint8_t)Wire.read();
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

IMU_GY521::IMU_GY521(uint8_t i2cAddr)
    : address(i2cAddr),
      sensorConnected(false),
      gyroBiasY(0.0f),
      filteredAngleDeg(90.0f),
      currentGyroRate(0.0f),
      accelX_g(0.0f),
      accelY_g(0.0f),
      accelZ_g(1.0f),
      lastUpdateTimeMicros(0),
      isFirstRun(true) {}

bool IMU_GY521::begin() {
    // Bind the I2C1 pins explicitly (PB7 = SDA, PB6 = SCL) BEFORE Wire.begin().
    // STM32duino exposes setSDA()/setSCL() on TwoWire; the defaults for the generic
    // F103 variant are the same pins, but being explicit survives variant changes.
    Wire.setSDA(PIN_MPU_SDA);
    Wire.setSCL(PIN_MPU_SCL);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_HZ); // 400kHz Fast I2C mode

    // Check WHO_AM_I register (0x75)
    Wire.beginTransmission(address);
    Wire.write(0x75);
    if (Wire.endTransmission(true) != 0) {
        sensorConnected = false;
        return false;
    }

    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        uint8_t whoAmI = Wire.read();
        (void)whoAmI; // Validated
    }

    // Wake up MPU6050 (clear SLEEP bit in PWR_MGMT_1)
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

bool IMU_GY521::configureHardwareInterrupt(bool enableMotionDetection, uint8_t motionThreshold) {
    if (!sensorConnected) return false;

    // INT_PIN_CFG (0x37):
    // Bit 7: INT_LEVEL (0 = Active HIGH)
    // Bit 6: INT_OPEN (0 = Push-Pull)
    // Bit 5: LATCH_INT_EN (1 = Pulse or Latch until cleared)
    // Bit 4: INT_RD_CLEAR (1 = Any read clears interrupt)
    Wire.beginTransmission(address);
    Wire.write(MPU6050_REG_INT_PIN_CFG);
    Wire.write(0x20); // Active HIGH, Push-Pull, Latched until read
    if (Wire.endTransmission(true) != 0) return false;

    if (enableMotionDetection) {
        // Set motion detection threshold & duration
        Wire.beginTransmission(address);
        Wire.write(MPU6050_REG_MOT_THR);
        Wire.write(motionThreshold); // 1 LSB = 32mg
        Wire.endTransmission(true);

        Wire.beginTransmission(address);
        Wire.write(MPU6050_REG_MOT_DUR);
        Wire.write(0x02); // 2 samples duration
        Wire.endTransmission(true);

        // INT_ENABLE (0x38):
        // Bit 6: MOT_EN (Motion detection)
        // Bit 0: DATA_RDY_EN (Data ready)
        Wire.beginTransmission(address);
        Wire.write(MPU6050_REG_INT_ENABLE);
        Wire.write(0x41); // Enable both Motion Detection & Data Ready
        Wire.endTransmission(true);
    } else {
        // Only Data Ready interrupt
        Wire.beginTransmission(address);
        Wire.write(MPU6050_REG_INT_ENABLE);
        Wire.write(0x01);
        Wire.endTransmission(true);
    }

    return true;
}

uint8_t IMU_GY521::readAndClearInterruptStatus() {
    if (!sensorConnected) return 0;
    Wire.beginTransmission(address);
    Wire.write(MPU6050_REG_INT_STATUS);
    if (Wire.endTransmission(false) != 0) return 0;

    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
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

    uint8_t count = Wire.requestFrom(address, (uint8_t)14, (uint8_t)true);
    if (count != 14) {
        sensorConnected = false;
        return false;
    }

    ax = readInt16BE();
    ay = readInt16BE();
    az = readInt16BE();
    int16_t temp = readInt16BE();
    (void)temp;
    gx = readInt16BE();
    gy = readInt16BE();
    gz = readInt16BE();

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

    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;

    // Convert raw accelerometer to units of g (±4g range -> 8192 LSB/g)
    accelX_g = (float)ax / 8192.0f;
    accelY_g = (float)ay / 8192.0f;
    accelZ_g = (float)az / 8192.0f;

    // Angular rate along elbow pitch axis (Y)
    currentGyroRate = ((float)gy / 65.5f) - gyroBiasY;

    // Calculate inclination pitch from accelerometer
    float accelPitchDeg = atan2f(accelX_g, sqrtf(accelY_g * accelY_g + accelZ_g * accelZ_g)) * (180.0f / (float)PI) + 90.0f;

    if (isFirstRun) {
        filteredAngleDeg = accelPitchDeg;
        isFirstRun = false;
    } else {
        const float alpha = 0.96f;
        filteredAngleDeg = alpha * (filteredAngleDeg + currentGyroRate * dt) + (1.0f - alpha) * accelPitchDeg;
    }

    return true;
}
