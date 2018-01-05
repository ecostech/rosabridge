#ifndef _ROS_rosabridge_msgs_Odometry_h
#define _ROS_rosabridge_msgs_Odometry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "rosabridge_msgs/Pose.h"
#include "rosabridge_msgs/Twist.h"

namespace rosabridge_msgs
{

  class Odometry : public ros::Msg
  {
    public:
      std_msgs::Header header;
      const char* child_frame_id;
      rosabridge_msgs::Pose pose;
      float pose_covariance[6];
      rosabridge_msgs::Twist twist;
      float twist_covariance[6];

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_child_frame_id = strlen(this->child_frame_id);
      memcpy(outbuffer + offset, &length_child_frame_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->child_frame_id, length_child_frame_id);
      offset += length_child_frame_id;
      offset += this->pose.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 6; i++){
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
      offset += this->twist.serialize(outbuffer + offset);
      for( uint8_t i = 0; i < 6; i++){
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
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_child_frame_id;
      memcpy(&length_child_frame_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_child_frame_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_child_frame_id-1]=0;
      this->child_frame_id = (char *)(inbuffer + offset-1);
      offset += length_child_frame_id;
      offset += this->pose.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 6; i++){
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
      offset += this->twist.deserialize(inbuffer + offset);
      for( uint8_t i = 0; i < 6; i++){
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

    const char * getType(){ return "rosabridge_msgs/Odometry"; };
    const char * getMD5(){ return "3c88d21ba93f27fc21da1718473d3e4e"; };

  };

}
#endif