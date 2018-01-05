#ifndef _ROS_SERVICE_RequestImuCovariances_h
#define _ROS_SERVICE_RequestImuCovariances_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Empty.h"
#include "rosabridge_msgs/ImuCovariances.h"

namespace rosabridge_msgs
{

static const char REQUESTIMUCOVARIANCES[] = "rosabridge_msgs/RequestImuCovariances";

  class RequestImuCovariancesRequest : public ros::Msg
  {
    public:
      std_msgs::Empty empty;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->empty.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->empty.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTIMUCOVARIANCES; };
    const char * getMD5(){ return "6aac6c697d5414bc0fcede8c33981d0e"; };

  };

  class RequestImuCovariancesResponse : public ros::Msg
  {
    public:
      rosabridge_msgs::ImuCovariances imu_covariances;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->imu_covariances.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->imu_covariances.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTIMUCOVARIANCES; };
    const char * getMD5(){ return "583c0cfa1603ebb13313e6a504689faf"; };

  };

  class RequestImuCovariances {
    public:
    typedef RequestImuCovariancesRequest Request;
    typedef RequestImuCovariancesResponse Response;
  };

}
#endif
