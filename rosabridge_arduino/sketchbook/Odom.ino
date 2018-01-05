
// Define following to enable debug output
//#define _ODOM_DEBUG

// Define following to use abbreviated data types
#define _ODOM_PROXY

#ifdef _ODOM_PROXY
// Define following to enable service for returning covariance
//#define _ODOM_COVAR_SERVER
#endif

#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#ifdef _ODOM_PROXY
#include <rosabridge_msgs/Odometry.h>
#include <rosabridge_msgs/RequestOdometryCovariances.h>
#else
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#endif

#include <SoftwareSerial.h>

#ifdef _ODOM_PROXY
rosabridge_msgs::Odometry odom_msg;
ros::Publisher odom_pub("rosabridge/odom", &odom_msg);
#else
geometry_msgs::TransformStamped trans_msg;
nav_msgs::Odometry odom_msg;
tf::TransformBroadcaster broadcaster;
ros::Publisher odom_pub("odom", &odom_msg);
#endif
const char base_link[] = "base_link";
const char odom[] = "odom";

#define NORMALIZE(z) (atan2(sin(z), cos(z)))

// buffer for reading encoder counts
int odom_idx = 0;
char odom_buf[24];

int32_t odom_encoder_left = 0;
int32_t odom_encoder_right = 0;

float odom_x = 0.0;
float odom_y = 0.0;
float odom_yaw = 0.0;
float odom_last_x = 0.0;
float odom_last_y = 0.0;
float odom_last_yaw = 0.0;

uint32_t odom_last_time = 0;

#ifdef _ODOM_COVAR_SERVER
void odom_covar_callback(const rosabridge_msgs::RequestOdometryCovariancesRequest& req, rosabridge_msgs::RequestOdometryCovariancesResponse& res)
{
  res.odometry_covariances.pose.pose.covariance[0] = 0.001;
  res.odometry_covariances.pose.pose.covariance[7] = 0.001;
  res.odometry_covariances.pose.pose.covariance[14] = 1000000;
  res.odometry_covariances.pose.pose.covariance[21] = 1000000;
  res.odometry_covariances.pose.pose.covariance[28] = 1000000;
  res.odometry_covariances.pose.pose.covariance[35] = 1000;

  res.odometry_covariances.twist.twist.covariance[0] = 0.001;
  res.odometry_covariances.twist.twist.covariance[7] = 0.001;
  res.odometry_covariances.twist.twist.covariance[14] = 1000000;
  res.odometry_covariances.twist.twist.covariance[21] = 1000000;
  res.odometry_covariances.twist.twist.covariance[28] = 1000000;
  res.odometry_covariances.twist.twist.covariance[35] = 1000;
}
ros::ServiceServer<rosabridge_msgs::RequestOdometryCovariancesRequest, rosabridge_msgs::RequestOdometryCovariancesResponse> odom_covar_server("rosabridge/odom_covar_srv",&odom_covar_callback);
#endif

void odom_setup()
{
#ifdef _ODOM_DEBUG
Serial.println("Starting Odom...");
Serial.flush();
#endif

#ifdef _ODOM_PROXY
  nh.advertise(odom_pub);
#else
  broadcaster.init(nh);
  nh.advertise(odom_pub);
#endif
#ifdef _ODOM_COVAR_SERVER
  nh.advertiseService(odom_covar_server);
#endif

  odom_msg.header.seq = 0;
  odom_msg.header.frame_id = odom;
  odom_msg.child_frame_id = base_link;

  // CBA Set Odom covariances  
#ifdef _ODOM_PROXY
  odom_msg.pose_covariance[0] = 0.001;
  odom_msg.pose_covariance[1] = 0.001;
  odom_msg.pose_covariance[2] = 1000000;
  odom_msg.pose_covariance[3] = 1000000;
  odom_msg.pose_covariance[4] = 1000000;
  odom_msg.pose_covariance[5] = 1000;

  odom_msg.twist_covariance[0] = 0.001;
  odom_msg.twist_covariance[1] = 0.001;
  odom_msg.twist_covariance[2] = 1000000;
  odom_msg.twist_covariance[3] = 1000000;
  odom_msg.twist_covariance[4] = 1000000;
  odom_msg.twist_covariance[5] = 1000;
#else
  trans_msg.header.seq = 0;
  trans_msg.header.frame_id = odom;
  trans_msg.child_frame_id = base_link;

  memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.001;
  odom_msg.pose.covariance[14] = 1000000;
  odom_msg.pose.covariance[21] = 1000000;
  odom_msg.pose.covariance[28] = 1000000;
  odom_msg.pose.covariance[35] = 1000;

  memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[7] = 0.001;
  odom_msg.twist.covariance[14] = 1000000;
  odom_msg.twist.covariance[21] = 1000000;
  odom_msg.twist.covariance[28] = 1000000;
  odom_msg.twist.covariance[35] = 1000;
#endif

  odom_last_time = millis();
}

void odom_publish()
{

  // determine delta time in seconds
  uint32_t nowtime = millis();
  float dt = (float)DELTAT(nowtime,odom_last_time) / 1000.0;
  odom_last_time = nowtime;

#ifdef _ODOM_DEBUG
/*
Serial.print("right: ");
Serial.print(odom_encoder_right);
Serial.print(" left: ");
Serial.print(odom_encoder_left);
Serial.print(" dt: ");
Serial.print(dt);
Serial.println("");
Serial.flush();
*/
#endif

  // determine deltas of distance and angle
  float linear = ((float)odom_encoder_right / ROBOT_ENCODER_CPR * ROBOT_WHEEL_CIRCUMFERENCE + (float)odom_encoder_left / ROBOT_ENCODER_CPR * ROBOT_WHEEL_CIRCUMFERENCE) / 2.0;
  float angular = ((float)odom_encoder_right / ROBOT_ENCODER_CPR * ROBOT_WHEEL_CIRCUMFERENCE - (float)odom_encoder_left / ROBOT_ENCODER_CPR * ROBOT_WHEEL_CIRCUMFERENCE) / ROBOT_AXLE_LENGTH;
#ifdef _ODOM_DEBUG
/*
Serial.print("linear: ");
Serial.print(linear);
Serial.print(" angular: ");
Serial.print(angular);
Serial.println("");
Serial.flush();
*/
#endif

  // Update odometry
  odom_x += linear * cos(odom_yaw);        // m
  odom_y += linear * sin(odom_yaw);        // m
  odom_yaw = NORMALIZE(odom_yaw + angular);  // rad
#ifdef _ODOM_DEBUG
/**/
Serial.print("x: ");
Serial.print(odom_x);
Serial.print(" y: ");
Serial.print(odom_y);
Serial.print(" yaw: ");
Serial.print(odom_yaw);
Serial.println("");
Serial.flush();
/**/
#endif

  // Calculate velocities
  float vx = (odom_x - odom_last_x) / dt;
  float vy = (odom_y - odom_last_y) / dt;
  float vyaw = (odom_yaw - odom_last_yaw) / dt;
  odom_last_x = odom_x;
  odom_last_y = odom_y;
  odom_last_yaw = odom_yaw;
#ifdef _ODOM_DEBUG
/*
Serial.print("vx: ");
Serial.print(vx);
Serial.print(" vy: ");
Serial.print(vy);
Serial.print(" vyaw: ");
Serial.print(vyaw);
Serial.println("");
Serial.flush();
*/
#endif

  // CBA tf:createQuaternionMsgFromYaw missing (from Arduino?)
//  geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odom_yaw);
  geometry_msgs::Quaternion quat = tf::createQuaternionFromYaw(odom_yaw);

#ifdef _ODOM_PROXY
  odom_msg.header.seq++;
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.position.x = odom_x;
  odom_msg.pose.position.y = odom_y;
  odom_msg.pose.position.z = 0.0;
  odom_msg.pose.orientation.x = quat.x;
  odom_msg.pose.orientation.y = quat.y;
  odom_msg.pose.orientation.z = quat.z;
  odom_msg.pose.orientation.w = quat.w;
  odom_msg.twist.linear.x = vx;
  odom_msg.twist.linear.y = vy;
  odom_msg.twist.linear.z = 0.0;
  odom_msg.twist.angular.x = 0.0;
  odom_msg.twist.angular.y = 0.0;
  odom_msg.twist.angular.z = vyaw;
  odom_pub.publish( &odom_msg );
#else
  trans_msg.header.seq++;
  trans_msg.header.stamp = nh.now();
  trans_msg.transform.translation.x = odom_x;
  trans_msg.transform.translation.y = odom_y;
  trans_msg.transform.translation.z = 0.0;
  trans_msg.transform.rotation = quat;
  broadcaster.sendTransform(trans_msg);

  odom_msg.header.seq++;
  odom_msg.header.stamp = nh.now();
  odom_msg.pose.pose.position.x = odom_x;
  odom_msg.pose.pose.position.y = odom_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = vyaw;
  odom_pub.publish( &odom_msg );
#endif

}
