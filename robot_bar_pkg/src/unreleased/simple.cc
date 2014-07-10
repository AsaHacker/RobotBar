#include "simple.h"

namespace simple {

void simple::LoadScenario() {
  std::ifstream file(filename_.c_str());
  std::string text;
  file >> size_;
  table_.resize(size_);
  for (int i=0; i<size_; ++i) {
    std::string text;
    file >> text;
    char* text_c = new char[text.size()+1];
    std::copy(text.begin(), text.end(), text_c);
    text_c[text.size()] = '\0';
    table_[i].key = std::string(strtok(text_c, ","));
    table_[i].output.data = std::string(strtok(NULL, "\n"));
    delete text_c;
  }
  file.close();
}

void simple::Callback(const std_msgs::String::ConstPtr& input) {
  for (int i=0; i<size_; ++i)
    if (strstr(input->data.c_str(), table_[i].key.c_str()) != NULL) {
      pub_.publish(table_[i].output);
      return;
    }
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  simple::simple test(nh);
  test.LoadScenario();
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}
