cmake_minimum_required(VERSION 2.8.3)
project(rosabridge_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES
                  Mag.msg
                  Point.msg
                  Vector3.msg
                  Quaternion.msg
                  Pose.msg
                  Twist.msg
                  Odometry.msg
                  OdometryCovariances.msg
                  Imu.msg
                  ImuCovariances.msg
                  Duplex.msg
                 )

add_service_files(FILES
                  RequestParam.srv
                  RequestOdometryCovariances.srv
                  RequestImuCovariances.srv
                 )

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
