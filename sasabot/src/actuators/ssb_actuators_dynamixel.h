#ifndef SSB_ACTUATORS_DYNAMIXEL_H_
#define SSB_ACTUATORS_DYNAMIXEL_H_

#include "ssb_common_common.h"
#include "dynamixel_msgs/MotorStateList.h"
#include "std_msgs/Float64.h"

namespace ssb_actuators_dynamixel {

class Dynamixel : public ssb_common_subscriber::Subscriber<dynamixel_msgs::MotorStateList> {
 public:
  Dynamixel() {};
  virtual ~Dynamixel() {};
  inline void publish(ros::Publisher pub, float value) {
    std_msgs::Float64 msg;
    msg.data = value;
    pub.publish(msg);
  };
};


class NeckDynamixel : public Dynamixel {
 public:
  explicit NeckDynamixel(ros::NodeHandle &nh) {
    commandx = nh.advertise<std_msgs::Float64>("/controller1/command", 100);
    commandy = nh.advertise<std_msgs::Float64>("/controller2/command", 100);
    commandz = nh.advertise<std_msgs::Float64>("/controller3/command", 100);
  };
  inline void publish(ssb_common_vec::VecNeck vec) {
    Dynamixel::publish(commandx, vec.roll);
    Dynamixel::publish(commandy, vec.pitch);
    Dynamixel::publish(commandz, vec.yaw);
  };
 private:
  ros::Publisher commandx, commandy, commandz;
};

} // namespace ssb_actuators_dynamixel

#endif
