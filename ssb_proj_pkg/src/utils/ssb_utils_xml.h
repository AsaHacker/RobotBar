#ifndef SSB_UTILS_XML_H_
#define SSB_UTILS_XML_H_

#include "ssb_common_common.h"
#include "ofxXmlSettings.h"

namespace ssb_utils_xml {

template<typename T_arg>
void ArgSetFunction(ofxXmlSettings &settings, int which_id, T_arg &arg, std::string tag) {};

template<typename T_arg>
void ArgGetFunction(ofxXmlSettings &settings, int which_id, T_arg &arg, std::string tag) {};

// Xml class reads and writes xml files for original robot part structures.
// The class only has two functions: read and write.
// Basically the functions are specialized for each robot part structure,
// and yes you will have to define new ones for any new parts.
// Note that the functions are not included in the basic vector structures,
// despite its specialization. The reason for this, is so that
// you can save some other variables with the basic vector, such as an image name.
// The saved-with variable can be any type (int, float, ...) as long as
// the ArgSetFunction and ArgGetFunction are defined for that type.
// For more than one saved-with variable, please define an original struct.
// For multiple tags, please use stringstreams.
// The class does not use "typename..." so that it does not have to be
// compiled in C++11.
// Like mentioned above, the class uses an ad-hoc compile-time polymorphism,
// or in other words "overloading." This is because Xml handling should be
// simple enough, thus should not have much room for flexibility and
// efficiency is prior.
template<typename T, typename T_arg> struct Xml {
  Xml() {};
  virtual ~Xml() {}; 
  virtual void setXmlSettings(ofxXmlSettings &settings, int which_id,
                              T &item, T_arg &arg, std::string tag) {};
  virtual void getXmlSettings(ofxXmlSettings &settings, int which_id,
                              T &item, T_arg &arg, std::string tag) {};
};

template<typename T_arg> struct Xml<ssb_common_vec::VecTentacle, T_arg> {
  void setXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecTentacle &item, T_arg &arg, std::string tag);
  void getXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecTentacle &item, T_arg &arg, std::string tag);
};

template<typename T_arg> struct Xml<ssb_common_vec::VecEye, T_arg> {
  void setXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecEye &item, T_arg &arg, std::string tag);
  void getXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecEye &item, T_arg &arg, std::string tag);
};

template<typename T_arg> struct Xml<ssb_common_vec::VecNeck, T_arg> {
  void setXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecNeck &item, T_arg &arg, std::string tag);
  void getXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecNeck &item, T_arg &arg, std::string tag);
};

template<typename T_arg> struct Xml<ssb_common_vec::VecTime, T_arg> {
  void setXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecTime &item, T_arg &arg, std::string tag);
  void getXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecTime &item, T_arg &arg, std::string tag);
};

template<typename T_arg> struct Xml<ssb_common_vec::VecVoice, T_arg> {
  void setXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecVoice &item, T_arg &arg, std::string tag);
  void getXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecVoice &item, T_arg &arg, std::string tag);
};

template<typename T_arg> struct Xml<ssb_common_vec::VecInterpolation, T_arg> {
  void setXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecInterpolation &item, T_arg &arg, std::string tag);
  void getXmlSettings(ofxXmlSettings &settings, int which_id,
                      ssb_common_vec::VecInterpolation &item, T_arg &arg, std::string tag);
};

} // namespace ssb_utils_xml
  
#endif
