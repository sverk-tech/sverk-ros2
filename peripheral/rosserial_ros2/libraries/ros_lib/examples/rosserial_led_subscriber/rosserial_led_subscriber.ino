#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

std_msgs::Bool led_state_msg;
ros::Publisher led_state_pub("led_state", &led_state_msg);

bool led_state = false;
unsigned long last_publish_ms = 0;

void ledCallback(const std_msgs::Bool &msg)
{
  led_state = msg.data;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);
  led_state_msg.data = led_state;
  led_state_pub.publish(&led_state_msg);
}

ros::Subscriber<std_msgs::Bool> led_sub("led", &ledCallback);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  nh.initNode();
  nh.advertise(led_state_pub);
  nh.subscribe(led_sub);
}

void loop()
{
  nh.spinOnce();

  if (millis() - last_publish_ms > 1000) {
    last_publish_ms = millis();
    led_state_msg.data = led_state;
    led_state_pub.publish(&led_state_msg);
  }

  delay(10);
}
