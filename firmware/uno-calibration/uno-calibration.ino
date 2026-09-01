// Safe, temporary motor-output calibration for the TSCINBUNY ZYC0076.
// The sketch never starts a motor on boot. Each accepted command runs for a
// bounded 5 seconds at low duty, then forces all outputs off.

constexpr uint8_t PWM1_PIN = 5;
constexpr uint8_t PWM2_PIN = 6;
constexpr uint8_t SHIFT_CLOCK_PIN = 2;
constexpr uint8_t ENABLE_PIN = 7;
constexpr uint8_t SHIFT_DATA_PIN = 8;
constexpr uint8_t LATCH_CLOCK_PIN = 4;

constexpr uint8_t DIRECTION_STOP = 0;
constexpr uint8_t DIRECTION_FORWARD = 92;
constexpr uint8_t CALIBRATION_DUTY = 140;
constexpr uint16_t CALIBRATION_MS = 5000;

void writeDirection(uint8_t direction) {
  digitalWrite(LATCH_CLOCK_PIN, LOW);
  shiftOut(SHIFT_DATA_PIN, SHIFT_CLOCK_PIN, MSBFIRST, direction);
  digitalWrite(LATCH_CLOCK_PIN, HIGH);
}

void stopMotors() {
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
  writeDirection(DIRECTION_STOP);
}

void runBoundedTest(uint8_t pwm1, uint8_t pwm2) {
  stopMotors();
  writeDirection(DIRECTION_FORWARD);
  analogWrite(PWM1_PIN, pwm1);
  analogWrite(PWM2_PIN, pwm2);
  delay(CALIBRATION_MS);
  stopMotors();
}

void setup() {
  // Disable the expansion board before configuring any other output.
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(SHIFT_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_DATA_PIN, OUTPUT);
  pinMode(LATCH_CLOCK_PIN, OUTPUT);
  stopMotors();

  digitalWrite(ENABLE_PIN, LOW);
  Serial.begin(115200);
  Serial.println(F("WOBOT CALIBRATION READY"));
  Serial.println(F("Send 1 for D5, 2 for D6; each test is 5 s at duty 140/255"));
}

void loop() {
  if (!Serial.available()) {
    return;
  }

  const char command = Serial.read();
  switch (command) {
    case '1':
      Serial.println(F("TEST D5 BEGIN"));
      runBoundedTest(CALIBRATION_DUTY, 0);
      Serial.println(F("TEST D5 END - STOPPED"));
      break;
    case '2':
      Serial.println(F("TEST D6 BEGIN"));
      runBoundedTest(0, CALIBRATION_DUTY);
      Serial.println(F("TEST D6 END - STOPPED"));
      break;
    case 's':
    case 'S':
      stopMotors();
      Serial.println(F("STOPPED"));
      break;
    default:
      // Ignore line endings and all unknown input. Motors remain stopped.
      break;
  }
}
