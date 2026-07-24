#ifndef _ROS_std_msgs_Time_h
#define _ROS_std_msgs_Time_h

#include <stdint.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace std_msgs
{

class Time : public ros::Msg
{
public:
  ros::Time data;

  virtual int serialize(unsigned char *outbuffer) const override
  {
    int offset = 0;
    ros::Msg::varToArr(outbuffer + offset, this->data.sec);
    offset += 4;
    ros::Msg::varToArr(outbuffer + offset, this->data.nsec);
    offset += 4;
    return offset;
  }

  virtual int deserialize(unsigned char *inbuffer) override
  {
    int offset = 0;
    ros::Msg::arrToVar(this->data.sec, inbuffer + offset);
    offset += 4;
    ros::Msg::arrToVar(this->data.nsec, inbuffer + offset);
    offset += 4;
    return offset;
  }

  virtual const char * getType() override { return "std_msgs/Time"; }
  virtual const char * getMD5() override { return "cd7166c74c552c311fbcc2fe5a7bc289"; }
};

}

#endif
