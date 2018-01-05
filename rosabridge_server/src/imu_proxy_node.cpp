#include <ros/ros.h>
#include "rosabridge_msgs/Imu.h"
#include "rosabridge_msgs/ImuCovariances.h"
#include "rosabridge_msgs/RequestImuCovariances.h"
#include <sensor_msgs/Imu.h>
#include <signal.h>
#include <string>

// CBA Define following to enable service for obtaining covariances
// (not currently supported by rosserial on Arduino)
//#define _IMU_COVAR

void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}

class ImuProxyNode
{

public:
  ImuProxyNode();

public:
  void imu_callback(const rosabridge_msgs::Imu::ConstPtr& imuMsg);
  int run();

protected:
  ros::NodeHandle nh;
  ros::Publisher imu_pub;
  ros::Subscriber imu_sub;
  sensor_msgs::Imu imu_msg;

#ifdef _IMU_COVAR
  ros::ServiceClient imu_client;
#endif

  std::string imu_frame;
  std::string imu_topic;

};

ImuProxyNode::ImuProxyNode()
{

  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param<std::string>("imu_frame", imu_frame, "");
  ROS_INFO_STREAM("imu_frame: " << imu_frame);
  nhLocal.param<std::string>("imu_topic", imu_topic, "imu/data");
  ROS_INFO_STREAM("imu_topic: " << imu_topic);

  // CBA Initially zero out covariances
  imu_msg.orientation_covariance.assign(0);
  imu_msg.angular_velocity_covariance.assign(0);
  imu_msg.linear_acceleration_covariance.assign(0);

}

void ImuProxyNode::imu_callback(const rosabridge_msgs::Imu::ConstPtr& imuMsg)
{
  imu_msg.header.stamp = imuMsg->header.stamp;
  if ( !imu_frame.empty() )
    imu_msg.header.frame_id = imu_frame;
  else
    imu_msg.header.frame_id = imuMsg->header.frame_id;
  imu_msg.orientation.x = imuMsg->orientation.x;
  imu_msg.orientation.y = imuMsg->orientation.y;
  imu_msg.orientation.z = imuMsg->orientation.z;
  imu_msg.orientation.w = imuMsg->orientation.w;
  imu_msg.linear_acceleration.x = imuMsg->linear_acceleration.x;
  imu_msg.linear_acceleration.y = imuMsg->linear_acceleration.y;
  imu_msg.linear_acceleration.z = imuMsg->linear_acceleration.z;
  imu_msg.angular_velocity.x = imuMsg->angular_velocity.x;
  imu_msg.angular_velocity.y = imuMsg->angular_velocity.y;
  imu_msg.angular_velocity.z = imuMsg->angular_velocity.z;
  imu_msg.orientation_covariance[0] = imuMsg->orientation_covariance[0];
  imu_msg.orientation_covariance[4] = imuMsg->orientation_covariance[1];
  imu_msg.orientation_covariance[8] = imuMsg->orientation_covariance[2];
  imu_msg.linear_acceleration_covariance[0] = imuMsg->linear_acceleration_covariance[0];
  imu_msg.linear_acceleration_covariance[4] = imuMsg->linear_acceleration_covariance[1];
  imu_msg.linear_acceleration_covariance[8] = imuMsg->linear_acceleration_covariance[2];
  imu_msg.angular_velocity_covariance[0] = imuMsg->angular_velocity_covariance[0];
  imu_msg.angular_velocity_covariance[4] = imuMsg->angular_velocity_covariance[1];
  imu_msg.angular_velocity_covariance[8] = imuMsg->angular_velocity_covariance[2];
  imu_pub.publish(imu_msg);
}

int ImuProxyNode::run()
{

#ifdef _IMU_COVAR
  ROS_INFO("Requesting Imu Covariances");
  imu_client = nh.serviceClient<rosabridge_msgs::RequestImuCovariances>("rosabridge/imu_covar_srv");
  rosabridge_msgs::RequestImuCovariances imuCovarSrv;
  if (imu_client.call(imuCovarSrv))
  {
    imu_msg.orientation_covariance[0] = imuCovarSrv.response.orientation_covariance[0];
    imu_msg.orientation_covariance[1] = imuCovarSrv.response.orientation_covariance[1];
    imu_msg.orientation_covariance[2] = imuCovarSrv.response.orientation_covariance[2];
    imu_msg.orientation_covariance[3] = imuCovarSrv.response.orientation_covariance[3];
    imu_msg.orientation_covariance[4] = imuCovarSrv.response.orientation_covariance[4];
    imu_msg.orientation_covariance[5] = imuCovarSrv.response.orientation_covariance[5];
    imu_msg.orientation_covariance[6] = imuCovarSrv.response.orientation_covariance[6];
    imu_msg.orientation_covariance[7] = imuCovarSrv.response.orientation_covariance[7];
    imu_msg.orientation_covariance[8] = imuCovarSrv.response.orientation_covariance[8];
    imu_msg.linear_acceleration_covariance[0] = imuCovarSrv.response.linear_acceleration_covariance[0];
    imu_msg.linear_acceleration_covariance[1] = imuCovarSrv.response.linear_acceleration_covariance[1];
    imu_msg.linear_acceleration_covariance[2] = imuCovarSrv.response.linear_acceleration_covariance[2];
    imu_msg.linear_acceleration_covariance[3] = imuCovarSrv.response.linear_acceleration_covariance[3];
    imu_msg.linear_acceleration_covariance[4] = imuCovarSrv.response.linear_acceleration_covariance[4];
    imu_msg.linear_acceleration_covariance[5] = imuCovarSrv.response.linear_acceleration_covariance[5];
    imu_msg.linear_acceleration_covariance[6] = imuCovarSrv.response.linear_acceleration_covariance[6];
    imu_msg.linear_acceleration_covariance[7] = imuCovarSrv.response.linear_acceleration_covariance[7];
    imu_msg.linear_acceleration_covariance[8] = imuCovarSrv.response.linear_acceleration_covariance[8];
    imu_msg.angular_velocity_covariance[0] = imuCovarSrv.response.angular_velocity_covariance[0];
    imu_msg.angular_velocity_covariance[1] = imuCovarSrv.response.angular_velocity_covariance[1];
    imu_msg.angular_velocity_covariance[2] = imuCovarSrv.response.angular_velocity_covariance[2];
    imu_msg.angular_velocity_covariance[3] = imuCovarSrv.response.angular_velocity_covariance[3];
    imu_msg.angular_velocity_covariance[4] = imuCovarSrv.response.angular_velocity_covariance[4];
    imu_msg.angular_velocity_covariance[5] = imuCovarSrv.response.angular_velocity_covariance[5];
    imu_msg.angular_velocity_covariance[6] = imuCovarSrv.response.angular_velocity_covariance[6];
    imu_msg.angular_velocity_covariance[7] = imuCovarSrv.response.angular_velocity_covariance[7];
    imu_msg.angular_velocity_covariance[8] = imuCovarSrv.response.angular_velocity_covariance[8];
  }
  else
  {
    ROS_WARN("Failed to retrieve Imu Covariances");
  }
#endif

  ROS_INFO_STREAM("Publishing to topic " << imu_topic);
  imu_pub = nh.advertise<sensor_msgs::Imu>(imu_topic, 1000);
  ROS_INFO("Subscribing to topic rosabridge/imu");
  imu_sub = nh.subscribe("rosabridge/imu", 1000, &ImuProxyNode::imu_callback, this);

  ROS_INFO("Relaying between topics...");
  ros::spin();

  ROS_INFO("Exiting");

  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "imu_proxy_node");

  ImuProxyNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}

