#include <ros.h>
#include <rosserial_test_msgs/McuStatus.h>

ros::NodeHandle_<SerialHardware, 1, 1, 160, 220> nh;

rosserial_test_msgs::McuStatus status_msg;
ros::Publisher status_pub("mcu_status", &status_msg);

char label[] = "nano_custom";
int16_t ticks = 0;
unsigned long last_publish_ms = 0;

void setup()
{
  pinMode(LED_BUILTIN, INPUT_PULLUP);

  status_msg.label = label;

  nh.initNode();
  nh.advertise(status_pub);
}

void loop()
{
  nh.spinOnce();

  if (millis() - last_publish_ms < 1000) {
    return;
  }
  last_publish_ms = millis();

  ticks += 7;
  status_msg.uptime_ms = millis();
  status_msg.encoder_ticks = ticks;
  status_msg.distance_m = (float)(ticks % 100) / 100.0f;
  status_msg.button_pressed = digitalRead(LED_BUILTIN) == LOW;
  status_msg.label = label;
  status_pub.publish(&status_msg);
}
