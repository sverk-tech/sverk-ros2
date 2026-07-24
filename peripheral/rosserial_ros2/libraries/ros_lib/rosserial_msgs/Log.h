#ifndef _ROS_rosserial_msgs_Log_h
#define _ROS_rosserial_msgs_Log_h

#include <stdint.h>
#include <string.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

class Log : public ros::Msg
{
public:
  enum { ROSDEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, FATAL = 4 };

  uint8_t level;
  char * msg;

  Log() : level(0), msg((char *)"") {}

  virtual int serialize(unsigned char *outbuffer) const override
  {
    int offset = 0;
    outbuffer[offset++] = this->level;
    uint32_t length = strlen(this->msg);
    ros::Msg::varToArr(outbuffer + offset, length);
    offset += 4;
    memcpy(outbuffer + offset, this->msg, length);
    offset += length;
    return offset;
  }

  virtual int deserialize(unsigned char *inbuffer) override
  {
    int offset = 0;
    this->level = inbuffer[offset++];
    uint32_t length;
    ros::Msg::arrToVar(length, inbuffer + offset);
    offset += 4;
    for (uint32_t k = offset; k < offset + length; ++k)
      inbuffer[k - 1] = inbuffer[k];
    inbuffer[offset + length - 1] = 0;
    this->msg = (char *)(inbuffer + offset - 1);
    offset += length;
    return offset;
  }

  virtual int serializedLength() const override
  {
    return 1 + 4 + strlen(this->msg);
  }

  virtual const char * getType() override { return "rosserial_msgs/Log"; }
  virtual const char * getMD5() override { return "acffd30cd6b6de30f120938c17c593fb"; }
};

}

#endif
