/*
 * Backward-compatible name for sketches written against classic rosserial.
 * New code should prefer SerialHardware, which describes the transport.
 */

#ifndef ROS_ARDUINO_HARDWARE_H_
#define ROS_ARDUINO_HARDWARE_H_

#include "SerialHardware.h"

typedef SerialHardware ArduinoHardware;

#endif
