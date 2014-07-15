#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>

namespace simple_pub{

  class SimplePub{
public:
    SimplePub(ros::NodeHandle nh){
      input_pub = nh.advertise<std_msgs::String>("input",1);
    }
    virtual ~SimplePub(){};
    void input_func();
private:
    std_msgs::String str_pub;
  ros::Publisher input_pub;
  void publisher_func(std::string *str);
  };
}











