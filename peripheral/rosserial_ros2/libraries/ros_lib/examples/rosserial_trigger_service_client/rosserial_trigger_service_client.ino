#include <ros.h>
#include <std_srvs/Trigger.h>

ros::NodeHandle_<SerialHardware, 2, 2, 128, 256> nh;

std_srvs::TriggerRequest trigger_request;
std_srvs::TriggerResponse trigger_response;
ros::ServiceClient<std_srvs::TriggerRequest, std_srvs::TriggerResponse> trigger_client("nano_trigger");

unsigned long last_call_ms = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  nh.initNode();
  nh.serviceClient(trigger_client);
}

void loop()
{
  nh.spinOnce();

  if (!nh.connected() || millis() - last_call_ms < 3000) {
    return;
  }
  last_call_ms = millis();

  trigger_client.call(trigger_request, trigger_response);
  digitalWrite(LED_BUILTIN, trigger_response.success ? HIGH : LOW);
}
