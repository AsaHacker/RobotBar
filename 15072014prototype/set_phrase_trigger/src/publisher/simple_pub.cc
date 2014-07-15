#include "simple_pub.h"

namespace simple_pub{

  void SimplePub::input_func(){
    std::string str_input;
    do{
      std::cout<<"Input a sentence"<<std::endl;
      std::cin>>str_input;
      publisher_func(&str_input);
  }while(ros::ok());


  }

  void SimplePub::publisher_func(std::string *str){
    std::cout<<"Publish: "<<*str<<std::endl;
    str_pub.data = str->c_str();
    input_pub.publish(str_pub);
  }



}


int main(int argc, char **argv){
  ros::init(argc, argv, "sentence_publisher");
  ros::NodeHandle nh;
  simple_pub::SimplePub sp(nh);
  sp.input_func();
  while (ros::ok()) {
    ros::spinOnce();
  }
  return 0;
}










