#ifndef _ROS_rosserial_msgs_RequestParam_h
#define _ROS_rosserial_msgs_RequestParam_h

#include <stdint.h>
#include <string.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

class RequestParamRequest : public ros::Msg
{
public:
  char * name;

  RequestParamRequest() : name((char *)"") {}

  virtual int serialize(unsigned char *outbuffer) const override
  {
    uint32_t length = strlen(this->name);
    ros::Msg::varToArr(outbuffer, length);
    memcpy(outbuffer + 4, this->name, length);
    return 4 + length;
  }

  virtual int deserialize(unsigned char *inbuffer) override
  {
    uint32_t length;
    ros::Msg::arrToVar(length, inbuffer);
    for (uint32_t k = 4; k < 4 + length; ++k)
      inbuffer[k - 1] = inbuffer[k];
    inbuffer[4 + length - 1] = 0;
    this->name = (char *)(inbuffer + 3);
    return 4 + length;
  }

  virtual int serializedLength() const override
  {
    return 4 + strlen(this->name);
  }

  virtual const char * getType() override { return "rosserial_msgs/RequestParamRequest"; }
  virtual const char * getMD5() override { return "d8a784ec699a10f3d4a5950526fe5f7c"; }
};

class RequestParamResponse : public ros::Msg
{
public:
  enum { MAX_STRINGS = 16 };

  uint8_t ints_length;
  int32_t st_om_ints;
  int32_t * ints;
  uint8_t floats_length;
  int32_t st_om_floats;
  float * floats;
  uint8_t strings_length;
  char * st_om_strings[MAX_STRINGS];
  char ** strings;

  RequestParamResponse() :
    ints_length(0),
    st_om_ints(0),
    ints(nullptr),
    floats_length(0),
    st_om_floats(0),
    floats(nullptr),
    strings_length(0),
    strings(st_om_strings) {}

  virtual int serialize(unsigned char *outbuffer) const override
  {
    int offset = 0;
    ros::Msg::varToArr(outbuffer + offset, (uint32_t)this->ints_length);
    offset += 4;
    for (uint8_t i = 0; i < this->ints_length; ++i)
    {
      ros::Msg::varToArr(outbuffer + offset, this->ints[i]);
      offset += 4;
    }

    ros::Msg::varToArr(outbuffer + offset, (uint32_t)this->floats_length);
    offset += 4;
    for (uint8_t i = 0; i < this->floats_length; ++i)
    {
      memcpy(outbuffer + offset, &this->floats[i], 4);
      offset += 4;
    }

    ros::Msg::varToArr(outbuffer + offset, (uint32_t)this->strings_length);
    offset += 4;
    for (uint8_t i = 0; i < this->strings_length; ++i)
    {
      uint32_t length = strlen(this->strings[i]);
      ros::Msg::varToArr(outbuffer + offset, length);
      offset += 4;
      memcpy(outbuffer + offset, this->strings[i], length);
      offset += length;
    }
    return offset;
  }

  virtual int deserialize(unsigned char *inbuffer) override
  {
    int offset = 0;

    uint32_t length = 0;
    ros::Msg::arrToVar(length, inbuffer + offset);
    offset += 4;
    ints_length = length > 255 ? 255 : (uint8_t)length;
    ints = reinterpret_cast<int32_t *>(inbuffer + offset);
    for (uint32_t i = 0; i < length; ++i)
    {
      if (i < ints_length)
        ros::Msg::arrToVar(ints[i], inbuffer + offset);
      offset += 4;
    }

    ros::Msg::arrToVar(length, inbuffer + offset);
    offset += 4;
    floats_length = length > 255 ? 255 : (uint8_t)length;
    floats = reinterpret_cast<float *>(inbuffer + offset);
    for (uint32_t i = 0; i < length; ++i)
    {
      if (i < floats_length)
        memcpy(&floats[i], inbuffer + offset, 4);
      offset += 4;
    }

    ros::Msg::arrToVar(length, inbuffer + offset);
    offset += 4;
    strings_length = length > MAX_STRINGS ? MAX_STRINGS : (uint8_t)length;
    strings = st_om_strings;
    for (uint32_t i = 0; i < length; ++i)
    {
      uint32_t string_length = 0;
      ros::Msg::arrToVar(string_length, inbuffer + offset);
      offset += 4;
      if (string_length == 0)
      {
        if (i < strings_length)
          strings[i] = (char *)"";
      }
      else
      {
        for (uint32_t k = offset; k < offset + string_length; ++k)
          inbuffer[k - 1] = inbuffer[k];
        inbuffer[offset + string_length - 1] = 0;
        if (i < strings_length)
          strings[i] = (char *)(inbuffer + offset - 1);
      }
      offset += string_length;
    }
    return offset;
  }

  virtual int serializedLength() const override
  {
    int offset = 0;
    offset += 4 + 4 * this->ints_length;
    offset += 4 + 4 * this->floats_length;
    offset += 4;
    for (uint8_t i = 0; i < this->strings_length; ++i)
      offset += 4 + strlen(this->strings[i]);
    return offset;
  }

  virtual const char * getType() override { return "rosserial_msgs/RequestParamResponse"; }
  virtual const char * getMD5() override { return "d8a784ec699a10f3d4a5950526fe5f7c"; }
};

}

#endif
