#include "simple.h"

namespace simple {

void simple::Callback(const std_msgs::String::ConstPtr& input) {
  for (int i=0; i<size_; ++i)
    if (input->data == table_[i].key)
      pub_.publish(table_[i].output);
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  simple::simple test(nh);
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}
