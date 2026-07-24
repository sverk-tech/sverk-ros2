#ifndef _ROS_rosserial_msgs_TopicInfo_h
#define _ROS_rosserial_msgs_TopicInfo_h

#include <stdint.h>
#include <string.h>
#include "ros/msg.h"

namespace rosserial_msgs
{

class TopicInfo : public ros::Msg
{
public:
  enum
  {
    ID_PUBLISHER = 0,
    ID_SUBSCRIBER = 1,
    ID_SERVICE_SERVER = 2,
    ID_SERVICE_CLIENT = 4,
    ID_PARAMETER_REQUEST = 6,
    ID_LOG = 7,
    ID_TIME = 10,
    ID_TX_STOP = 11
  };

  uint16_t topic_id;
  char * topic_name;
  char * message_type;
  char * md5sum;
  int32_t buffer_size;

  TopicInfo() :
    topic_id(0),
    topic_name((char *)""),
    message_type((char *)""),
    md5sum((char *)""),
    buffer_size(0) {}

  virtual int serialize(unsigned char *outbuffer) const override
  {
    int offset = 0;
    ros::Msg::varToArr(outbuffer + offset, this->topic_id);
    offset += 2;
    offset += serializeString(outbuffer + offset, this->topic_name);
    offset += serializeString(outbuffer + offset, this->message_type);
    offset += serializeString(outbuffer + offset, this->md5sum);
    ros::Msg::varToArr(outbuffer + offset, this->buffer_size);
    offset += 4;
    return offset;
  }

  virtual int deserialize(unsigned char *inbuffer) override
  {
    int offset = 0;
    ros::Msg::arrToVar(this->topic_id, inbuffer + offset);
    offset += 2;
    offset += deserializeString(inbuffer + offset, this->topic_name);
    offset += deserializeString(inbuffer + offset, this->message_type);
    offset += deserializeString(inbuffer + offset, this->md5sum);
    ros::Msg::arrToVar(this->buffer_size, inbuffer + offset);
    offset += 4;
    return offset;
  }

  virtual int serializedLength() const override
  {
    return 2
      + serializedStringLength(this->topic_name)
      + serializedStringLength(this->message_type)
      + serializedStringLength(this->md5sum)
      + 4;
  }

  virtual const char * getType() override { return "rosserial_msgs/TopicInfo"; }
  virtual const char * getMD5() override { return "0ad51f88fc44892f8c10684077646005"; }

private:
  static int serializeString(unsigned char *outbuffer, const char *value)
  {
    uint32_t length = strlen(value);
    ros::Msg::varToArr(outbuffer, length);
    memcpy(outbuffer + 4, value, length);
    return 4 + length;
  }

  static int serializedStringLength(const char *value)
  {
    return 4 + strlen(value);
  }

  static int deserializeString(unsigned char *inbuffer, char *&value)
  {
    uint32_t length;
    ros::Msg::arrToVar(length, inbuffer);
    for (uint32_t k = 4; k < 4 + length; ++k)
      inbuffer[k - 1] = inbuffer[k];
    inbuffer[4 + length - 1] = 0;
    value = (char *)(inbuffer + 3);
    return 4 + length;
  }
};

}

#endif
