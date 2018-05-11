#ifndef _ROS_rosabridge_msgs_OdometryCovariances_h
#define _ROS_rosabridge_msgs_OdometryCovariances_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosabridge_msgs
{

  class OdometryCovariances : public ros::Msg
  {
    public:
      float pose_covariance[36];
      float twist_covariance[36];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_pose_covariancei;
      u_pose_covariancei.real = this->pose_covariance[i];
      *(outbuffer + offset + 0) = (u_pose_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pose_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pose_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pose_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pose_covariance[i]);
      }
      for( uint8_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_twist_covariancei;
      u_twist_covariancei.real = this->twist_covariance[i];
      *(outbuffer + offset + 0) = (u_twist_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_twist_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_twist_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_twist_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->twist_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_pose_covariancei;
      u_pose_covariancei.base = 0;
      u_pose_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pose_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pose_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pose_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pose_covariance[i] = u_pose_covariancei.real;
      offset += sizeof(this->pose_covariance[i]);
      }
      for( uint8_t i = 0; i < 36; i++){
      union {
        float real;
        uint32_t base;
      } u_twist_covariancei;
      u_twist_covariancei.base = 0;
      u_twist_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_twist_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_twist_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_twist_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->twist_covariance[i] = u_twist_covariancei.real;
      offset += sizeof(this->twist_covariance[i]);
      }
     return offset;
    }

    const char * getType(){ return "rosabridge_msgs/OdometryCovariances"; };
    const char * getMD5(){ return "a4b306391e5fe1ada3a3b38b968daf06"; };

  };

}
#endif