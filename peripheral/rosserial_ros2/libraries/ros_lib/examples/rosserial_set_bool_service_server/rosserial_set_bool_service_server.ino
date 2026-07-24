#include <ros.h>
#include <std_srvs/SetBool.h>

ros::NodeHandle_<SerialHardware, 2, 2, 128, 256> nh;

void setLed(const std_srvs::SetBoolRequest & request, std_srvs::SetBoolResponse & response)
{
  digitalWrite(LED_BUILTIN, request.data ? HIGH : LOW);
  response.success = true;
  response.message = request.data ? (char *)"LED on" : (char *)"LED off";
}

ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> set_led_service("set_led", setLed);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  nh.initNode();
  nh.advertiseService(set_led_service);
}

void loop()
{
  nh.spinOnce();
}
