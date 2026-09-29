/* MPU6050 or WHO_AM_I=0x98 ICM20689-compatible device, Nano, 115200 baud. IMU2 packets: acquisition micros + sequence +
 * raw +/-4g, +/-500deg/s axes + temperature, followed by XOR checksum.
 * Register definitions: TDK MPU-6000/6050 Register Map revision 4.2.
 */
#include <Wire.h>
static const uint8_t MPU = 0x68;
uint32_t sequence = 0;
uint32_t next_poll = 0;
uint32_t health_start = 0, health_samples = 0, read_errors = 0;
uint8_t chip_identity = 0;

bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
bool readBytes(uint8_t reg, uint8_t count, uint8_t *data) {
  Wire.beginTransmission(MPU); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}
int16_t signedWord(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}
// Resetting the Nano mid-I2C transaction can leave the slave holding SDA.
// Release (never drive high) the bus, clock at most nine bits, then send STOP.
void recoverBus() {
  Wire.end();
  pinMode(SDA, INPUT); pinMode(SCL, INPUT);
  delayMicroseconds(10);
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; ++i) {
    digitalWrite(SCL, LOW); pinMode(SCL, OUTPUT); delayMicroseconds(10);
    pinMode(SCL, INPUT); delayMicroseconds(10);
  }
  digitalWrite(SCL, LOW); pinMode(SCL, OUTPUT);
  digitalWrite(SDA, LOW); pinMode(SDA, OUTPUT); delayMicroseconds(10);
  pinMode(SCL, INPUT); delayMicroseconds(10);
  pinMode(SDA, INPUT); delayMicroseconds(10);
  Wire.begin(); Wire.setClock(100000UL); // short packets at 100 Hz, long jumper wires
  Wire.setWireTimeout(3000, true);
}
bool verifyRegister(uint8_t reg, uint8_t expected) {
  uint8_t value = 0;
  bool ok = readBytes(reg, 1, &value);
  if (!ok || value != expected) {
    Serial.print("REGISTER fault addr=0x"); Serial.print(reg, HEX);
    Serial.print(" expected=0x"); Serial.print(expected, HEX);
    Serial.print(" observed=0x"); Serial.print(value, HEX);
    Serial.print(" read_ok="); Serial.println(ok);
    return false;
  }
  return true;
}
bool verifyConfiguration() {
  bool ok = true;
  ok &= verifyRegister(0x75, chip_identity);
  ok &= verifyRegister(0x6B, 0x01);
  ok &= verifyRegister(0x6C, 0x00);
  ok &= verifyRegister(0x19, 0x09);
  ok &= verifyRegister(0x1A, 0x03);
  ok &= verifyRegister(0x1B, 0x08);
  ok &= verifyRegister(0x1C, 0x08);
  ok &= verifyRegister(0x38, 0x01);
  if (chip_identity == 0x98) {
    ok &= verifyRegister(0x1D, 0x03);
    ok &= verifyRegister(0x1E, 0x00);
  }
  return ok;
}
bool configureSensor() {
  uint8_t who = 0;
  bool read_ok = readBytes(0x75, 1, &who);
  Serial.print("WHO_AM_I=0x"); Serial.print(who, HEX);
  Serial.print(" read_ok="); Serial.println(read_ok);
  if (!read_ok || (who != 0x68 && who != 0x98)) return false;
  chip_identity = who;
  // Nano resets do not necessarily reset the separately powered IMU. Start
  // from known device state instead of retaining old low-power/filter state.
  if (!writeReg(0x6B, 0x80)) return false;
  delay(100);
  bool ok = writeReg(0x6B, 0x01); // PLL when ready, awake
  delay(100); // gyro startup/settling before configuring its output
  ok &= writeReg(0x6C, 0x00); // enable all accelerometer/gyro axes
  ok &= writeReg(0x6A, 0x00); // no FIFO, DMP or I2C master
  ok &= writeReg(0x1B, 0x08); // +/-500 deg/s
  ok &= writeReg(0x1C, 0x08); // +/-4 g
  ok &= writeReg(0x1A, 0x03); // gyro DLPF
  if (who == 0x98) {
    ok &= writeReg(0x1D, 0x03); // accelerometer filter
    ok &= writeReg(0x1E, 0x00); // continuous gyro, no low-power cycling
  }
  ok &= writeReg(0x19, 0x09); // 1 kHz / (1+9) = 100 Hz
  ok &= writeReg(0x37, 0x00); // clear data-ready only on INT_STATUS read
  ok &= writeReg(0x38, 0x01);
  return verifyConfiguration() && ok;
}
void haltSensor(const char *reason) {
  Serial.print("ERROR IMU "); Serial.println(reason);
  // Do not silently restart a clock/calibration under a running estimator.
  while (true) delay(1000);
}
void setup() {
  Serial.begin(115200);
  delay(100);
  bool ok = false;
  for (uint8_t attempt = 0; attempt < 10 && !ok; ++attempt) {
    recoverBus(); delay(50);
    ok = configureSensor();
    if (!ok) { Serial.println("Retrying I2C setup"); delay(100); }
  }
  if (!ok) {
    Serial.println("ERROR IMU setup failed; check power and I2C wiring");
    while (true) delay(1000);
  }
  Serial.println("HW290 IMU2 100Hz accel=8192 gyro=65.5 verified_reset=1");
  health_start = millis();
}
void loop() {
  uint32_t now_ms = millis();
  if ((uint32_t)(now_ms - health_start) >= 1000) {
    uint32_t elapsed = now_ms - health_start;
    Serial.print("HEALTH samples="); Serial.print(health_samples);
    Serial.print(" elapsed_ms="); Serial.print(elapsed);
    Serial.print(" read_errors="); Serial.println(read_errors);
    bool config_ok = verifyConfiguration();
    if (!config_ok) haltSensor("configuration/I2C fault; restart after checking connection");
    if (health_samples * 1000UL < elapsed * 80UL || health_samples * 1000UL > elapsed * 120UL)
      haltSensor("source rate outside 80-120 Hz; restart after checking connection");
    health_start = millis(); health_samples = 0; read_errors = 0;
  }
  uint32_t now = micros();
  if ((int32_t)(now - next_poll) < 0) return;
  next_poll = now + 250; // no catch-up bursts after a delayed loop
  uint8_t status, data[14];
  if (!readBytes(0x3A, 1, &status)) { ++read_errors; return; }
  if (!(status & 1)) return;
  uint32_t acquired_us = micros(); // data-ready observed, before I2C/serial
  uint32_t sample_sequence = sequence++;
  if (!readBytes(0x3B, 14, data)) { ++read_errors; return; }
  char packet[100];
  int length = snprintf(packet, sizeof(packet), "IMU2,%lu,%lu,%d,%d,%d,%d,%d,%d,%d",
      (unsigned long)sample_sequence, (unsigned long)acquired_us,
      signedWord(data), signedWord(data+2), signedWord(data+4),
      signedWord(data+8), signedWord(data+10), signedWord(data+12), signedWord(data+6));
  if (length <= 0 || length >= (int)sizeof(packet)) return;
  uint8_t checksum = 0;
  for (int i = 0; i < length; ++i) checksum ^= (uint8_t)packet[i];
  ++health_samples;
  Serial.print(packet); Serial.print('*');
  if (checksum < 16) Serial.print('0');
  Serial.println(checksum, HEX);
}
