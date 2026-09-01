#ifndef WOBOT_MOTOR_OUTPUT_H
#define WOBOT_MOTOR_OUTPUT_H

#include <stdint.h>

struct MotorPwm {
  uint8_t left;
  uint8_t right;
};

class MotorOutputController {
 public:
  static const uint16_t ACTIVE_COMMAND_THRESHOLD = 10;
  static const uint8_t SUSTAIN_PWM = 26;
  static const uint8_t STARTUP_BOOST_PWM = 210;
  static const uint16_t STARTUP_BOOST_MS = 300;

  MotorOutputController() : robotWasActive_(false), boostUntil_(0) {}

  void reset() {
    robotWasActive_ = false;
    boostUntil_ = 0;
  }

  static int16_t effectiveSpeed(int16_t speed) {
    return magnitudeOf(speed) >= ACTIVE_COMMAND_THRESHOLD ? speed : 0;
  }

  MotorPwm update(uint32_t now, int16_t leftSpeed, int16_t rightSpeed) {
    uint8_t left = pwmForSpeed(leftSpeed);
    uint8_t right = pwmForSpeed(rightSpeed);
    const bool robotIsActive = left != 0 || right != 0;

    // Boost only a transition of the whole robot from stopped to moving.
    // Steering one side through zero must never trigger a second kick.
    if (robotIsActive && !robotWasActive_) {
      boostUntil_ = now + STARTUP_BOOST_MS;
    } else if (!robotIsActive) {
      boostUntil_ = 0;
    }

    if (robotIsActive && beforeDeadline(now, boostUntil_)) {
      if (left != 0 && left < STARTUP_BOOST_PWM) {
        left = STARTUP_BOOST_PWM;
      }
      if (right != 0 && right < STARTUP_BOOST_PWM) {
        right = STARTUP_BOOST_PWM;
      }
    }

    robotWasActive_ = robotIsActive;
    return MotorPwm{left, right};
  }

 private:
  bool robotWasActive_;
  uint32_t boostUntil_;

  static uint16_t magnitudeOf(int16_t speed) {
    return speed < 0 ? static_cast<uint16_t>(-static_cast<int32_t>(speed))
                     : static_cast<uint16_t>(speed);
  }

  static uint8_t pwmForSpeed(int16_t speed) {
    const uint16_t magnitude = magnitudeOf(speed);
    if (magnitude < ACTIVE_COMMAND_THRESHOLD) {
      return 0;
    }
    const uint8_t requested =
        static_cast<uint8_t>((magnitude * 255UL) / 1000UL);
    return requested < SUSTAIN_PWM ? SUSTAIN_PWM : requested;
  }

  static bool beforeDeadline(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(deadline - now) > 0;
  }
};

#endif
