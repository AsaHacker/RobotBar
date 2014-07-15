#include "simple.h"
#include <string>
#include <sstream>

namespace simple {

void simple::LoadScenario() {
  std::cout << filename_.c_str() << std::endl ;
  std::ifstream file(filename_.c_str());
  std::string text;
  file >> size_;
  table_.resize(size_);
  for (int i=0; i<size_; ++i) {
    std::string text;
    file >> text;
    // char* text_c = new char[text.size()+1];
    // std::copy(text.begin(), text.end(), text_c);
    // text_c[text.size()] = '\0';
    int comma_position = text.find(",") ;
    table_[i].key = text.substr(0,comma_position) ;
    table_[i].output.data = text.substr(comma_position+1, text.size()) ; // std::string(strtok(text_c, "\n")
    // table_[i].key = std::string(strtok(text_c, ","));
    // table_[i].output.data = std::string(strtok(NULL, "\n"));
    // delete text_c;
  }
  file.close();
}

  void simple::LoadDatabase(){
  std::ifstream file(database_name.c_str());
file >> size_;
  set_phrase_.resize(size_);
  for (int i=0; i<size_; ++i) {
    std::string text;
    file >> text;
    int comma_position = text.find(",") ;
    set_phrase_[i].key = text.substr(0,comma_position) ;
    set_phrase_[i].output.data = text.substr(comma_position+1, text.size()) ; 
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

  void simple::LevelCallback(const std_msgs::Int16::ConstPtr& level){
    std::stringstream database_name_ss;
    database_name_ss<<"database_level"<<level->data<<".txt";
    database_name = database_name_ss.str();
      std::cout <<"Load " << database_name.c_str() << std::endl ;
    LoadDatabase();
    return;
}

  void simple::SetPhrasePublisher(const std_msgs::String::ConstPtr& input){
  for (int i=0; i<size_; ++i)
    if (strstr(input->data.c_str(), set_phrase_[i].key.c_str()) != NULL) {
    pub_event_topic.publish(set_phrase_[i].output);
    return;
    }

  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  simple::simple test(nh);
  //  test.LoadScenario();
  test.LoadDatabase();
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}







