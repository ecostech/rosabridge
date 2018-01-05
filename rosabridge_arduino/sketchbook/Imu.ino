
// Define following to use abbreviated data types
#define _IMU_PROXY

#ifdef _IMU_PROXY
// Define following to enable service for returning covariance
//#define _IMU_COVAR_SERVER
#endif

#ifdef _IMU_PROXY
#include <rosabridge_msgs/Imu.h>
#include <rosabridge_msgs/RequestImuCovariances.h>
#else
#include <sensor_msgs/Imu.h>
#endif

#ifdef _IMU_PROXY
rosabridge_msgs::Imu imu_msg;
ros::Publisher imu_pub("rosabridge/imu", &imu_msg);
#else
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);
#endif
const char base_imu_link[] = "base_imu_link";

#ifdef _IMU_COVAR_SERVER
void imu_covar_callback(const rosabridge_msgs::RequestImuCovariancesRequest& req, rosabridge_msgs::RequestImuCovariancesResponse& res)
{
  res.imu_covariances.orientation_covariance[0] = 0.0025;
  res.imu_covariances.orientation_covariance[4] = 0.0025;
  res.imu_covariances.orientation_covariance[8] = 0.0025;
  
  res.imu_covariances.angular_velocity_covariance[0] = 0.02;
  res.imu_covariances.angular_velocity_covariance[4] = 0.02;
  res.imu_covariances.angular_velocity_covariance[8] = 0.02;
  
  res.imu_covariances.linear_acceleration_covariance[0] = 0.04;
  res.imu_covariances.linear_acceleration_covariance[4] = 0.04;
  res.imu_covariances.linear_acceleration_covariance[8] = 0.04;
}
ros::ServiceServer<rosabridge_msgs::RequestImuCovariancesRequest, rosabridge_msgs::RequestImuCovariancesResponse> imu_covar_server("rosabridge/imu_covar_srv",&imu_covar_callback);
#endif

void imu_setup()
{

  imu_msg.header.seq = 0;
  imu_msg.header.frame_id = base_imu_link;

  // CBA Set IMU covariances  
#ifdef _IMU_PROXY
  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0.0025;
  imu_msg.orientation_covariance[2] = 0.0025;
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0.02;
  imu_msg.angular_velocity_covariance[2] = 0.02;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0.04;
  imu_msg.linear_acceleration_covariance[2] = 0.04;
#else
  memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[8] = 0.0025;
  memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[8] = 0.02;
  memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;
#endif

  nh.advertise(imu_pub);
#ifdef _IMU_COVAR_SERVER
  nh.advertiseService(imu_covar_server);
#endif

}

void imu_publish()
{

  {
    imu_msg.header.seq++;
    imu_msg.header.stamp = nh.now();

    imu_msg.orientation.x = ahrs.x;
    imu_msg.orientation.y = ahrs.y;
    imu_msg.orientation.z = ahrs.z;
    imu_msg.orientation.w = ahrs.w;
    imu_msg.angular_velocity.x = ahrs.gyro[0];
    imu_msg.angular_velocity.y = ahrs.gyro[1];
    imu_msg.angular_velocity.z = ahrs.gyro[2];
    imu_msg.linear_acceleration.x = ahrs.accel[0];
    imu_msg.linear_acceleration.y = ahrs.accel[1];
    imu_msg.linear_acceleration.z = ahrs.accel[2];

    imu_pub.publish( &imu_msg );
  }

}
