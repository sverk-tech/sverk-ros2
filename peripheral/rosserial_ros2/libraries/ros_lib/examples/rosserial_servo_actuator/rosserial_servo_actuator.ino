#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt16.h>

const uint8_t SERVO_PIN = 9;
const uint16_t MIN_ANGLE = 0;
const uint16_t MAX_ANGLE = 180;

ros::NodeHandle_<SerialHardware, 1, 1, 128, 128> nh;

Servo servo;
std_msgs::UInt16 servo_status_msg;
ros::Publisher servo_status_pub("servo_status", &servo_status_msg);

uint16_t current_angle = 90;
unsigned long last_status_ms = 0;

uint16_t clampAngle(uint16_t angle)
{
  if (angle < MIN_ANGLE) {
    return MIN_ANGLE;
  }
  if (angle > MAX_ANGLE) {
    return MAX_ANGLE;
  }
  return angle;
}

void publishStatus()
{
  servo_status_msg.data = current_angle;
  servo_status_pub.publish(&servo_status_msg);
}

void servoAngleCallback(const std_msgs::UInt16 &msg)
{
  current_angle = clampAngle(msg.data);
  servo.write(current_angle);
  publishStatus();
}

ros::Subscriber<std_msgs::UInt16> servo_angle_sub("servo_angle", &servoAngleCallback);

void setup()
{
  servo.attach(SERVO_PIN);
  servo.write(current_angle);

  nh.initNode();
  nh.advertise(servo_status_pub);
  nh.subscribe(servo_angle_sub);
}

void loop()
{
  nh.spinOnce();

  if (millis() - last_status_ms > 1000) {
    last_status_ms = millis();
    publishStatus();
  }

  delay(10);
}
