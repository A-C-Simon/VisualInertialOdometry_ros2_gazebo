/*
 * HW-290 MPU6050 stream for OpenVINS.
 * Arduino Nano, MPU6050 at I2C address 0x68, serial output at 115200 baud.
 * The ROS2 bridge expects the MPU a/g text record below.
 */
#include <Wire.h>

static const uint8_t MPU = 0x68;
static const uint32_t PERIOD_US = 10000UL;  // 100 Hz
uint32_t next_sample;

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(value); Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000UL);
  delay(100);
  writeReg(0x6B, 0x00);  // wake up
  writeReg(0x1B, 0x00);  // gyro +/-250 deg/s, 131 LSB/(deg/s)
  writeReg(0x1C, 0x00);  // accel +/-2 g, 16384 LSB/g
  writeReg(0x1A, 0x03);  // digital low-pass filter
  Serial.println("HW-290 OpenVINS IMU 100Hz");
  next_sample = micros();
}

void loop() {
  uint32_t now = micros();
  if ((int32_t)(now - next_sample) < 0) return;
  next_sample += PERIOD_US;
  Wire.beginTransmission(MPU); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU, (uint8_t)14);
  if (Wire.available() < 14) return;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  Serial.print("MPU a/g: "); Serial.print(ax); Serial.print(','); Serial.print(ay); Serial.print(','); Serial.print(az);
  Serial.print(" | "); Serial.print(gx); Serial.print(','); Serial.print(gy); Serial.print(','); Serial.print(gz); Serial.println();
}
