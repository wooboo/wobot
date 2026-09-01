// Wobot motor controller for the TSCINBUNY ZYC0076 ATmega328P board.
//
// Safety properties:
// - motors are off during startup;
// - only framed commands with a valid CRC and explicit ARM flag are applied;
// - malformed or unsupported commands stop the motors;
// - a local watchdog stops the motors when commands cease;
// - commands that cannot be represented by the board's two PWM channels stop
//   the motors instead of silently producing a different motion.

#include <Arduino.h>
#include "motor_output.h"

constexpr uint8_t PWM_LEFT_PIN = 5;
constexpr uint8_t PWM_RIGHT_PIN = 6;
constexpr uint8_t SHIFT_CLOCK_PIN = 2;
constexpr uint8_t ENABLE_PIN = 7;
constexpr uint8_t SHIFT_DATA_PIN = 8;
constexpr uint8_t LATCH_CLOCK_PIN = 4;

constexpr uint8_t FRONT_LEFT_FORWARD = 0x40;
constexpr uint8_t FRONT_LEFT_REVERSE = 0x80;
constexpr uint8_t REAR_LEFT_FORWARD = 0x10;
constexpr uint8_t REAR_LEFT_REVERSE = 0x20;
constexpr uint8_t FRONT_RIGHT_FORWARD = 0x04;
constexpr uint8_t FRONT_RIGHT_REVERSE = 0x02;
constexpr uint8_t REAR_RIGHT_FORWARD = 0x08;
constexpr uint8_t REAR_RIGHT_REVERSE = 0x01;

constexpr uint8_t FRAME_START = 0xA5;
constexpr uint8_t FRAME_END = 0x5A;
constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr uint8_t FLAG_ARMED = 1 << 0;
constexpr uint8_t FRAME_LENGTH = 14;
constexpr uint16_t COMMAND_TIMEOUT_MS = 150;

uint8_t frame[FRAME_LENGTH];
uint8_t frameLength = 0;
unsigned long lastCommandAt = 0;
bool commandIsLive = false;
MotorOutputController motorOutput;

uint8_t crc8(const uint8_t* bytes, uint8_t length) {
  uint8_t crc = 0;
  for (uint8_t index = 0; index < length; ++index) {
    crc ^= bytes[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) != 0 ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

void writeDirection(uint8_t direction) {
  digitalWrite(LATCH_CLOCK_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, direction);
  digitalWrite(LATCH_CLOCK_PIN, HIGH);
}

void stopMotors() {
  analogWrite(PWM_LEFT_PIN, 0);
  analogWrite(PWM_RIGHT_PIN, 0);
  writeDirection(0);
  commandIsLive = false;
  motorOutput.reset();
}

int16_t readI16(const uint8_t* bytes) {
  return static_cast<int16_t>(
      static_cast<uint16_t>(bytes[0]) |
      (static_cast<uint16_t>(bytes[1]) << 8));
}

bool inRange(int16_t speed) {
  return speed >= -1000 && speed <= 1000;
}

uint16_t magnitudeOf(int16_t speed) {
  return speed < 0 ? static_cast<uint16_t>(-static_cast<int32_t>(speed))
                   : static_cast<uint16_t>(speed);
}

bool hasSameMagnitude(int16_t first, int16_t second) {
  return magnitudeOf(first) == magnitudeOf(second);
}

uint8_t directionFor(int16_t speed, uint8_t forwardBit, uint8_t reverseBit) {
  if (speed > 0) {
    return forwardBit;
  }
  if (speed < 0) {
    return reverseBit;
  }
  return 0;
}

bool applyFrame(const uint8_t* command) {
  if (command[0] != FRAME_START || command[1] != PROTOCOL_VERSION ||
      command[13] != FRAME_END || crc8(&command[1], 11) != command[12]) {
    stopMotors();
    return false;
  }

  const int16_t requestedFrontLeft = readI16(&command[4]);
  const int16_t requestedFrontRight = readI16(&command[6]);
  const int16_t requestedRearLeft = readI16(&command[8]);
  const int16_t requestedRearRight = readI16(&command[10]);

  if (!inRange(requestedFrontLeft) || !inRange(requestedFrontRight) ||
      !inRange(requestedRearLeft) || !inRange(requestedRearRight) ||
      !hasSameMagnitude(requestedFrontLeft, requestedRearLeft) ||
      !hasSameMagnitude(requestedFrontRight, requestedRearRight)) {
    stopMotors();
    return false;
  }

  if ((command[3] & FLAG_ARMED) == 0) {
    stopMotors();
    lastCommandAt = millis();
    return true;
  }
  const int16_t frontLeft =
      MotorOutputController::effectiveSpeed(requestedFrontLeft);
  const int16_t frontRight =
      MotorOutputController::effectiveSpeed(requestedFrontRight);
  const int16_t rearLeft =
      MotorOutputController::effectiveSpeed(requestedRearLeft);
  const int16_t rearRight =
      MotorOutputController::effectiveSpeed(requestedRearRight);

  const uint8_t direction =
      directionFor(frontLeft, FRONT_LEFT_FORWARD, FRONT_LEFT_REVERSE) |
      directionFor(rearLeft, REAR_LEFT_FORWARD, REAR_LEFT_REVERSE) |
      directionFor(frontRight, FRONT_RIGHT_FORWARD, FRONT_RIGHT_REVERSE) |
      directionFor(rearRight, REAR_RIGHT_FORWARD, REAR_RIGHT_REVERSE);

  const unsigned long now = millis();
  const MotorPwm pwm = motorOutput.update(now, frontLeft, frontRight);

  // Remove power before changing bridge direction, then apply the new duty.
  analogWrite(PWM_LEFT_PIN, 0);
  analogWrite(PWM_RIGHT_PIN, 0);
  writeDirection(direction);
  analogWrite(PWM_LEFT_PIN, pwm.left);
  analogWrite(PWM_RIGHT_PIN, pwm.right);

  lastCommandAt = now;
  commandIsLive = pwm.left != 0 || pwm.right != 0;
  return true;
}

void acceptByte(uint8_t byte) {
  if (frameLength == 0) {
    if (byte == FRAME_START) {
      frame[frameLength++] = byte;
    }
    return;
  }

  frame[frameLength++] = byte;
  if (frameLength != FRAME_LENGTH) {
    return;
  }

  applyFrame(frame);
  frameLength = 0;

  // If the last byte also looks like a start marker, retain it. This makes
  // recovery from a truncated stream deterministic without scanning memory.
  if (byte == FRAME_START) {
    frame[frameLength++] = byte;
  }
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(PWM_LEFT_PIN, OUTPUT);
  pinMode(PWM_RIGHT_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  pinMode(LATCH_CLOCK_PIN, OUTPUT);
  stopMotors();

  Serial.begin(115200);
  digitalWrite(ENABLE_PIN, LOW);
}

void loop() {
  while (Serial.available() > 0) {
    acceptByte(static_cast<uint8_t>(Serial.read()));
  }

  if (commandIsLive &&
      static_cast<unsigned long>(millis() - lastCommandAt) > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }
}
