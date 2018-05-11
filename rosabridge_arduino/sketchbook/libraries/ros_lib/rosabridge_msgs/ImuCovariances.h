#ifndef _ROS_rosabridge_msgs_ImuCovariances_h
#define _ROS_rosabridge_msgs_ImuCovariances_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace rosabridge_msgs
{

  class ImuCovariances : public ros::Msg
  {
    public:
      float orientation_covariance[9];
      float angular_velocity_covariance[9];
      float linear_acceleration_covariance[9];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_orientation_covariancei;
      u_orientation_covariancei.real = this->orientation_covariance[i];
      *(outbuffer + offset + 0) = (u_orientation_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_covariance[i]);
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_covariancei;
      u_angular_velocity_covariancei.real = this->angular_velocity_covariance[i];
      *(outbuffer + offset + 0) = (u_angular_velocity_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angular_velocity_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angular_velocity_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angular_velocity_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angular_velocity_covariance[i]);
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_covariancei;
      u_linear_acceleration_covariancei.real = this->linear_acceleration_covariance[i];
      *(outbuffer + offset + 0) = (u_linear_acceleration_covariancei.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_linear_acceleration_covariancei.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_linear_acceleration_covariancei.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_linear_acceleration_covariancei.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->linear_acceleration_covariance[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_orientation_covariancei;
      u_orientation_covariancei.base = 0;
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->orientation_covariance[i] = u_orientation_covariancei.real;
      offset += sizeof(this->orientation_covariance[i]);
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_angular_velocity_covariancei;
      u_angular_velocity_covariancei.base = 0;
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angular_velocity_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angular_velocity_covariance[i] = u_angular_velocity_covariancei.real;
      offset += sizeof(this->angular_velocity_covariance[i]);
      }
      for( uint8_t i = 0; i < 9; i++){
      union {
        float real;
        uint32_t base;
      } u_linear_acceleration_covariancei;
      u_linear_acceleration_covariancei.base = 0;
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_linear_acceleration_covariancei.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->linear_acceleration_covariance[i] = u_linear_acceleration_covariancei.real;
      offset += sizeof(this->linear_acceleration_covariance[i]);
      }
     return offset;
    }

    const char * getType(){ return "rosabridge_msgs/ImuCovariances"; };
    const char * getMD5(){ return "abc17e56a180069d18f7856848657acb"; };

  };

}
#endif