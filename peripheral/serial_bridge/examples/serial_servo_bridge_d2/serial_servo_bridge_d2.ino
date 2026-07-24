#include <Servo.h>

namespace {
constexpr int kServoPin = 2;
constexpr int kDefaultAngle = 90;
constexpr int kMinAngle = 0;
constexpr int kMaxAngle = 180;
constexpr long kBaudrate = 115200;
constexpr unsigned long kStartupDelayMs = 2000;

constexpr uint8_t kCommandCenter = 0xFD;
constexpr uint8_t kCommandEnable = 0xFE;
constexpr uint8_t kCommandDisable = 0xFF;
}

Servo servo_motor;
bool servo_enabled = true;
int current_angle = kDefaultAngle;

void set_servo_angle(int angle) {
  if (angle < kMinAngle) {
    angle = kMinAngle;
  }
  if (angle > kMaxAngle) {
    angle = kMaxAngle;
  }

  current_angle = angle;
  if (servo_enabled) {
    servo_motor.write(current_angle);
  }
}

void enable_servo(bool enabled) {
  if (enabled && !servo_enabled) {
    servo_motor.attach(kServoPin);
    servo_motor.write(current_angle);
    servo_enabled = true;
    return;
  }

  if (!enabled && servo_enabled) {
    servo_motor.detach();
    servo_enabled = false;
  }
}

void setup() {
  Serial.begin(kBaudrate);
  servo_motor.attach(kServoPin);
  servo_motor.write(kDefaultAngle);
  current_angle = kDefaultAngle;
  servo_enabled = true;
  delay(kStartupDelayMs);
}

void loop() {
  while (Serial.available() > 0) {
    int raw = Serial.read();
    if (raw < 0) {
      return;
    }

    uint8_t command = static_cast<uint8_t>(raw);

    if (command <= kMaxAngle) {
      set_servo_angle(static_cast<int>(command));
      continue;
    }

    if (command == kCommandCenter) {
      set_servo_angle(kDefaultAngle);
      continue;
    }

    if (command == kCommandEnable) {
      enable_servo(true);
      continue;
    }

    if (command == kCommandDisable) {
      enable_servo(false);
      continue;
    }
  }
}
