/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _ROS_H_
#define _ROS_H_

#include "ros/node_handle.h"

#if defined(ROSSERIAL_ARDUINO_TCP)
  #ifndef ROSSERIAL_HARDWARE_HEADER
    #define ROSSERIAL_HARDWARE_HEADER "ArduinoTcpHardware.h"
  #endif
  #ifndef ROSSERIAL_HARDWARE_CLASS
    #define ROSSERIAL_HARDWARE_CLASS ArduinoTcpHardware
  #endif
#endif

#ifndef ROSSERIAL_HARDWARE_HEADER
  #define ROSSERIAL_HARDWARE_HEADER "SerialHardware.h"
#endif

#ifndef ROSSERIAL_HARDWARE_CLASS
  #define ROSSERIAL_HARDWARE_CLASS SerialHardware
#endif

#include ROSSERIAL_HARDWARE_HEADER

namespace ros
{
#if defined(__AVR_ATmega8__) or defined(__AVR_ATmega168__)
  /* downsize our buffers */
  typedef Node_<ROSSERIAL_HARDWARE_CLASS, 6, 6, 150, 150> Node;

#elif defined(__AVR_ATmega328P__)

  typedef Node_<ROSSERIAL_HARDWARE_CLASS, 25, 25, 280, 280> Node;

#elif defined(SPARK)

  typedef Node_<ROSSERIAL_HARDWARE_CLASS, 10, 10, 2048, 2048> Node;

#else

  typedef Node_<ROSSERIAL_HARDWARE_CLASS> Node; // default 25, 25, 512, 512

#endif

typedef Node NodeHandle;
}

#endif
