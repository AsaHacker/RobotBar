#include "ros/ros.h"
#include "std_msgs/String.h"

namespace simple {

struct table {
  std::string key;
  std_msgs::String output;
};

class simple {
 public:
  explicit simple(ros::NodeHandle nh) {
    pub_ = nh.advertise<std_msgs::String>("output", 1000);
    sub_ = nh.subscribe("input", 100, &simple::Callback, this);
  }
  virtual ~simple() {};
 private:
  int size_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  std::vector<table> table_;
  void Callback(const std_msgs::String::ConstPtr& input);
};

}
