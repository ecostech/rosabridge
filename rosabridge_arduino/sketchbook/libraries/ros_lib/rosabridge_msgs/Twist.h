#ifndef _ROS_rosabridge_msgs_Twist_h
#define _ROS_rosabridge_msgs_Twist_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rosabridge_msgs/Vector3.h"

namespace rosabridge_msgs
{

  class Twist : public ros::Msg
  {
    public:
      rosabridge_msgs::Vector3 linear;
      rosabridge_msgs::Vector3 angular;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->linear.serialize(outbuffer + offset);
      offset += this->angular.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear.deserialize(inbuffer + offset);
      offset += this->angular.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "rosabridge_msgs/Twist"; };
    const char * getMD5(){ return "6d107193b261039abb32b01ddb75189b"; };

  };

}
#endif