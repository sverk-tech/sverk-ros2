#include <ros2.h>
#include <std_msgs/String.h>

ros::Node_<SerialHardware, 4, 4, 220, 220> node;

std_msgs::String status_msg;
ros::Publisher status_pub("diag_status", &status_msg);

char status_buffer[120];
char label_buffer[16] = "";
char * label_values[1] = {label_buffer};

bool sent_report = false;
unsigned long connected_at = 0;

void setup()
{
  node.init();
  node.create_publisher(status_pub);
}

void loop()
{
  node.spin_some();

  if (!node.is_connected())
  {
    sent_report = false;
    connected_at = 0;
    delay(10);
    return;
  }

  if (connected_at == 0)
  {
    connected_at = millis();
    node.loginfo("diagnostic sketch connected");
  }

  if (sent_report || millis() - connected_at < 1500)
  {
    delay(10);
    return;
  }

  int period_values[1] = {0};
  float gain_values[1] = {0.0};
  bool enabled_values[1] = {false};

  bool got_period = node.get_parameter("period_ms", period_values, 1);
  bool got_gain = node.get_parameter("gain", gain_values, 1);
  bool got_label = node.get_parameter("label", label_values, 1);
  bool got_enabled = node.get_parameter("enabled", enabled_values, 1);

  ros::Time now = node.now();
  int gain_milli = (int)(gain_values[0] * 1000.0);

  snprintf(
    status_buffer,
    sizeof(status_buffer),
    "time=%lu period=%d/%d gain_milli=%d/%d label=%s/%d enabled=%d/%d",
    (unsigned long)now.sec,
    period_values[0],
    got_period ? 1 : 0,
    gain_milli,
    got_gain ? 1 : 0,
    label_buffer,
    got_label ? 1 : 0,
    enabled_values[0] ? 1 : 0,
    got_enabled ? 1 : 0
  );

  status_msg.data = status_buffer;
  status_pub.publish(&status_msg);
  node.loginfo(status_buffer);

  bool values_ok =
    got_period &&
    got_gain &&
    got_label &&
    got_enabled &&
    period_values[0] == 250 &&
    gain_milli == 1500 &&
    strcmp(label_buffer, "nano_diag") == 0 &&
    enabled_values[0] &&
    now.sec > 1000000000UL;

  if (values_ok)
  {
    node.loginfo("time/log/parameter diagnostic passed");
  }
  else
  {
    node.logwarn("time/log/parameter diagnostic incomplete");
  }

  sent_report = true;
}
