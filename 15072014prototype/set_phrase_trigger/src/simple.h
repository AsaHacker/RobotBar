#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include <fstream>

namespace simple {

struct table {
  std::string key;
  std_msgs::String output;
};

class simple {
 public:
  explicit simple(ros::NodeHandle nh) {
    pub_ = nh.advertise<std_msgs::String>("output", 1000);
    //   sub_ = nh.subscribe("input", 100, &simple::Callback, this);
   sub_ = nh.subscribe("input", 100, &simple::SetPhrasePublisher, this);
    sub_level = nh.subscribe("level", 100, &simple::LevelCallback, this);
    pub_event_topic = nh.advertise<std_msgs::String>("event_topic",10);
    nh.getParam("filename", filename_);
  }
  virtual ~simple() {};
  void LoadScenario();
  void LoadDatabase();

  private:
  int size_;
  std::string filename_;
  std::string database_name;
  ros::Publisher pub_;
  ros::Publisher pub_event_topic;
  ros::Subscriber sub_;
  ros::Subscriber sub_level;
  std::vector<table> table_;
  std::vector<table> set_phrase_;
  void Callback(const std_msgs::String::ConstPtr& input);
   void LevelCallback(const std_msgs::Int16::ConstPtr& level);
   void SetPhrasePublisher(const std_msgs::String::ConstPtr& input);
};

}

















