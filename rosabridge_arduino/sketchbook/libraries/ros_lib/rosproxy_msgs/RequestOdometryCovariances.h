#ifndef _ROS_SERVICE_RequestOdometryCovariances_h
#define _ROS_SERVICE_RequestOdometryCovariances_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Empty.h"
#include "rosabridge_msgs/OdometryCovariances.h"

namespace rosabridge_msgs
{

static const char REQUESTODOMETRYCOVARIANCES[] = "rosabridge_msgs/RequestOdometryCovariances";

  class RequestOdometryCovariancesRequest : public ros::Msg
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

    const char * getType(){ return REQUESTODOMETRYCOVARIANCES; };
    const char * getMD5(){ return "6aac6c697d5414bc0fcede8c33981d0e"; };

  };

  class RequestOdometryCovariancesResponse : public ros::Msg
  {
    public:
      rosabridge_msgs::OdometryCovariances odometry_covariances;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->odometry_covariances.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->odometry_covariances.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return REQUESTODOMETRYCOVARIANCES; };
    const char * getMD5(){ return "560766e7723436a6430cd33a6d8796bb"; };

  };

  class RequestOdometryCovariances {
    public:
    typedef RequestOdometryCovariancesRequest Request;
    typedef RequestOdometryCovariancesResponse Response;
  };

}
#endif
