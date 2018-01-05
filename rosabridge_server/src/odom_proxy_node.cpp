#include <ros/ros.h>
#include "rosabridge_msgs/Odometry.h"
#include "rosabridge_msgs/OdometryCovariances.h"
#include "rosabridge_msgs/RequestOdometryCovariances.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <string>

// CBA Define following to enable service for obtaining covariances
// (not currently supported by rosserial on Arduino)
//#define _ODOM_COVAR

void mySigintHandler(int sig)
{
  ROS_INFO("Received SIGINT signal, shutting down...");
  ros::shutdown();
}

class OdomProxyNode
{

public:
  OdomProxyNode();

public:
  void odom_callback(const rosabridge_msgs::Odometry::ConstPtr& odomMsg);
  int run();

protected:
  ros::NodeHandle nh;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Publisher odom_pub;
  ros::Subscriber odom_sub;
  geometry_msgs::TransformStamped trans_msg;
  nav_msgs::Odometry odom_msg;

#ifdef _ODOM_COVAR
  ros::ServiceClient odom_client;
#endif

  bool pub_odom_tf;
  std::string odom_frame;
  std::string base_frame;
  std::string odom_topic;

};

OdomProxyNode::OdomProxyNode()
{

  // CBA Read local params (from launch file)
  ros::NodeHandle nhLocal("~");
  nhLocal.param("pub_odom_tf", pub_odom_tf, true);
  ROS_INFO_STREAM("pub_odom_tf: " << pub_odom_tf);
  nhLocal.param<std::string>("odom_frame", odom_frame, "");
  ROS_INFO_STREAM("odom_frame: " << odom_frame);
  nhLocal.param<std::string>("base_frame", base_frame, "");
  ROS_INFO_STREAM("base_frame: " << base_frame);
  nhLocal.param<std::string>("odom_topic", odom_topic, "odom");
  ROS_INFO_STREAM("odom_topic: " << odom_topic);

  // CBA Initially zero out covariances
  odom_msg.pose.covariance.assign(0);
  odom_msg.twist.covariance.assign(0);

}

void OdomProxyNode::odom_callback(const rosabridge_msgs::Odometry::ConstPtr& odomMsg)
{
  if ( pub_odom_tf )
  {
  trans_msg.header.stamp = odomMsg->header.stamp;
  if ( !odom_frame.empty() )
    trans_msg.header.frame_id = odom_frame;
  else
    trans_msg.header.frame_id = odomMsg->header.frame_id;
  if ( !base_frame.empty() )
    trans_msg.child_frame_id = base_frame;
  else
    trans_msg.child_frame_id = odomMsg->child_frame_id;
  trans_msg.transform.translation.x = odomMsg->pose.position.x;
  trans_msg.transform.translation.y = odomMsg->pose.position.y;
  trans_msg.transform.translation.z = odomMsg->pose.position.z;
  trans_msg.transform.rotation.x = odomMsg->pose.orientation.x;
  trans_msg.transform.rotation.y = odomMsg->pose.orientation.y;
  trans_msg.transform.rotation.z = odomMsg->pose.orientation.z;
  trans_msg.transform.rotation.w = odomMsg->pose.orientation.w;
  odom_broadcaster.sendTransform(trans_msg);
  }

  odom_msg.header.stamp = odomMsg->header.stamp;
  if ( !odom_frame.empty() )
    odom_msg.header.frame_id = odom_frame;
  else
    odom_msg.header.frame_id = odomMsg->header.frame_id;
  if ( !base_frame.empty() )
    odom_msg.child_frame_id = base_frame;
  else
    odom_msg.child_frame_id = odomMsg->child_frame_id;
  odom_msg.pose.pose.position.x = odomMsg->pose.position.x;
  odom_msg.pose.pose.position.y = odomMsg->pose.position.y;
  odom_msg.pose.pose.position.z = odomMsg->pose.position.z;
  odom_msg.pose.pose.orientation.x = odomMsg->pose.orientation.x;
  odom_msg.pose.pose.orientation.y = odomMsg->pose.orientation.y;
  odom_msg.pose.pose.orientation.z = odomMsg->pose.orientation.z;
  odom_msg.pose.pose.orientation.w = odomMsg->pose.orientation.w;
  odom_msg.twist.twist.linear.x = odomMsg->twist.linear.x;
  odom_msg.twist.twist.linear.y = odomMsg->twist.linear.y;
  odom_msg.twist.twist.linear.z = odomMsg->twist.linear.z;
  odom_msg.twist.twist.angular.x = odomMsg->twist.angular.x;
  odom_msg.twist.twist.angular.y = odomMsg->twist.angular.y;
  odom_msg.twist.twist.angular.z = odomMsg->twist.angular.z;
  odom_msg.pose.covariance[0] = odomMsg->pose_covariance[0];
  odom_msg.pose.covariance[7] = odomMsg->pose_covariance[1];
  odom_msg.pose.covariance[14] = odomMsg->pose_covariance[2];
  odom_msg.pose.covariance[21] = odomMsg->pose_covariance[3];
  odom_msg.pose.covariance[28] = odomMsg->pose_covariance[4];
  odom_msg.pose.covariance[35] = odomMsg->pose_covariance[5];
  odom_msg.twist.covariance[0] = odomMsg->twist_covariance[0];
  odom_msg.twist.covariance[7] = odomMsg->twist_covariance[1];
  odom_msg.twist.covariance[14] = odomMsg->twist_covariance[2];
  odom_msg.twist.covariance[21] = odomMsg->twist_covariance[3];
  odom_msg.twist.covariance[28] = odomMsg->twist_covariance[4];
  odom_msg.twist.covariance[35] = odomMsg->twist_covariance[5];
  odom_pub.publish(odom_msg);
}

int OdomProxyNode::run()
{

#ifdef _ODOM_COVAR
  ROS_INFO("Requesting Odometry Covariances");
  odom_client = nh.serviceClient<rosabridge_msgs::RequestOdometryCovariances>("rosabridge/odom_covar_srv");
  rosabridge_msgs::RequestOdometryCovariances odomCovarSrv;
  if (odom_client.call(odomCovarSrv))
  {
    odom_msg.pose.covariance[0] = odomCovarSrv.response.pose_covariance[0];
    odom_msg.pose.covariance[1] = odomCovarSrv.response.pose_covariance[1];
    odom_msg.pose.covariance[2] = odomCovarSrv.response.pose_covariance[2];
    odom_msg.pose.covariance[3] = odomCovarSrv.response.pose_covariance[3];
    odom_msg.pose.covariance[4] = odomCovarSrv.response.pose_covariance[4];
    odom_msg.pose.covariance[5] = odomCovarSrv.response.pose_covariance[5];
    odom_msg.pose.covariance[6] = odomCovarSrv.response.pose_covariance[6];
    odom_msg.pose.covariance[7] = odomCovarSrv.response.pose_covariance[7];
    odom_msg.pose.covariance[8] = odomCovarSrv.response.pose_covariance[8];
    odom_msg.pose.covariance[9] = odomCovarSrv.response.pose_covariance[9];
    odom_msg.pose.covariance[10] = odomCovarSrv.response.pose_covariance[10];
    odom_msg.pose.covariance[11] = odomCovarSrv.response.pose_covariance[11];
    odom_msg.pose.covariance[12] = odomCovarSrv.response.pose_covariance[12];
    odom_msg.pose.covariance[13] = odomCovarSrv.response.pose_covariance[13];
    odom_msg.pose.covariance[14] = odomCovarSrv.response.pose_covariance[14];
    odom_msg.pose.covariance[15] = odomCovarSrv.response.pose_covariance[15];
    odom_msg.pose.covariance[16] = odomCovarSrv.response.pose_covariance[16];
    odom_msg.pose.covariance[17] = odomCovarSrv.response.pose_covariance[17];
    odom_msg.pose.covariance[18] = odomCovarSrv.response.pose_covariance[18];
    odom_msg.pose.covariance[19] = odomCovarSrv.response.pose_covariance[19];
    odom_msg.pose.covariance[20] = odomCovarSrv.response.pose_covariance[20];
    odom_msg.pose.covariance[21] = odomCovarSrv.response.pose_covariance[21];
    odom_msg.pose.covariance[22] = odomCovarSrv.response.pose_covariance[22];
    odom_msg.pose.covariance[23] = odomCovarSrv.response.pose_covariance[23];
    odom_msg.pose.covariance[24] = odomCovarSrv.response.pose_covariance[24];
    odom_msg.pose.covariance[25] = odomCovarSrv.response.pose_covariance[25];
    odom_msg.pose.covariance[26] = odomCovarSrv.response.pose_covariance[26];
    odom_msg.pose.covariance[27] = odomCovarSrv.response.pose_covariance[27];
    odom_msg.pose.covariance[28] = odomCovarSrv.response.pose_covariance[28];
    odom_msg.pose.covariance[29] = odomCovarSrv.response.pose_covariance[29];
    odom_msg.pose.covariance[30] = odomCovarSrv.response.pose_covariance[30];
    odom_msg.pose.covariance[31] = odomCovarSrv.response.pose_covariance[31];
    odom_msg.pose.covariance[32] = odomCovarSrv.response.pose_covariance[32];
    odom_msg.pose.covariance[33] = odomCovarSrv.response.pose_covariance[33];
    odom_msg.pose.covariance[34] = odomCovarSrv.response.pose_covariance[34];
    odom_msg.pose.covariance[35] = odomCovarSrv.response.pose_covariance[35];
    odom_msg.twist.covariance[0] = odomCovarSrv.response.twist_covariance[0];
    odom_msg.twist.covariance[1] = odomCovarSrv.response.twist_covariance[1];
    odom_msg.twist.covariance[2] = odomCovarSrv.response.twist_covariance[2];
    odom_msg.twist.covariance[3] = odomCovarSrv.response.twist_covariance[3];
    odom_msg.twist.covariance[4] = odomCovarSrv.response.twist_covariance[4];
    odom_msg.twist.covariance[5] = odomCovarSrv.response.twist_covariance[5];
    odom_msg.twist.covariance[6] = odomCovarSrv.response.twist_covariance[6];
    odom_msg.twist.covariance[7] = odomCovarSrv.response.twist_covariance[7];
    odom_msg.twist.covariance[8] = odomCovarSrv.response.twist_covariance[8];
    odom_msg.twist.covariance[9] = odomCovarSrv.response.twist_covariance[9];
    odom_msg.twist.covariance[10] = odomCovarSrv.response.twist_covariance[10];
    odom_msg.twist.covariance[11] = odomCovarSrv.response.twist_covariance[11];
    odom_msg.twist.covariance[12] = odomCovarSrv.response.twist_covariance[12];
    odom_msg.twist.covariance[13] = odomCovarSrv.response.twist_covariance[13];
    odom_msg.twist.covariance[14] = odomCovarSrv.response.twist_covariance[14];
    odom_msg.twist.covariance[15] = odomCovarSrv.response.twist_covariance[15];
    odom_msg.twist.covariance[16] = odomCovarSrv.response.twist_covariance[16];
    odom_msg.twist.covariance[17] = odomCovarSrv.response.twist_covariance[17];
    odom_msg.twist.covariance[18] = odomCovarSrv.response.twist_covariance[18];
    odom_msg.twist.covariance[19] = odomCovarSrv.response.twist_covariance[19];
    odom_msg.twist.covariance[20] = odomCovarSrv.response.twist_covariance[20];
    odom_msg.twist.covariance[21] = odomCovarSrv.response.twist_covariance[21];
    odom_msg.twist.covariance[22] = odomCovarSrv.response.twist_covariance[22];
    odom_msg.twist.covariance[23] = odomCovarSrv.response.twist_covariance[23];
    odom_msg.twist.covariance[24] = odomCovarSrv.response.twist_covariance[24];
    odom_msg.twist.covariance[25] = odomCovarSrv.response.twist_covariance[25];
    odom_msg.twist.covariance[26] = odomCovarSrv.response.twist_covariance[26];
    odom_msg.twist.covariance[27] = odomCovarSrv.response.twist_covariance[27];
    odom_msg.twist.covariance[28] = odomCovarSrv.response.twist_covariance[28];
    odom_msg.twist.covariance[29] = odomCovarSrv.response.twist_covariance[29];
    odom_msg.twist.covariance[30] = odomCovarSrv.response.twist_covariance[30];
    odom_msg.twist.covariance[31] = odomCovarSrv.response.twist_covariance[31];
    odom_msg.twist.covariance[32] = odomCovarSrv.response.twist_covariance[32];
    odom_msg.twist.covariance[33] = odomCovarSrv.response.twist_covariance[33];
    odom_msg.twist.covariance[34] = odomCovarSrv.response.twist_covariance[34];
    odom_msg.twist.covariance[35] = odomCovarSrv.response.twist_covariance[35];
  }
  else
  {
    ROS_WARN("Failed to retrieve Odometry Covariances");
  }
#endif

  ROS_INFO("Broadcasting odom tf");
  ROS_INFO_STREAM("Publishing to topic " << odom_topic);
  odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 1000);
  ROS_INFO("Subscribing to topic rosabridge/odom");
  odom_sub = nh.subscribe("rosabridge/odom", 1000, &OdomProxyNode::odom_callback, this);

  ROS_INFO("Relaying between topics...");
  ros::spin();

  ROS_INFO("Exiting");

  return 0;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_proxy_node");

  OdomProxyNode node;

  // Override the default ros sigint handler.
  // This must be set after the first NodeHandle is created.
  signal(SIGINT, mySigintHandler);

  return node.run();
}

