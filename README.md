# rosabridge

ROS proxy nodes to expand and relay abridged messages received over bandwidth limited connections.

Useful for resource limited devices like arduino or anything using rosserial to be able to publish messages that may otherwise be too large or bandwidth intensive to publish.

Currently implements proxy nodes for Odom messages (nav_msgs/Odometry.h) and Imu messages (sensor_msgs/Imu.h).

## Usage

Clone to src directory of catkin workspace, then `catkin_make`.

Copy rosabridge_arduino/sketchbook/libraries/ros_lib/rosabridge_msgs to the ros_lib directory of your Arduino libraries.

Sample launch files in rosabridge_server/launch.

Sample Arduino code in rosabridge_arduino/sketchbook.

Custom messages in rosabridge_msgs are not yet built for Arduino by catkin.  If changed then rebuild headers manually:

```
rosrun rosserial_arduino make_libraries.py ~/sketchbook/libraries
```

## Authors

* **Chad Attermann** - *Initial work* - [Ecos Technologies](https://github.com/ecostech)

## License

This project is licensed under the BSD License.

