#ifndef SSB_UTILS_XML_INL_H_
#define SSB_UTILS_XML_INL_H_

#include "ssb_utils_xml.h"

namespace ssb_utils_xml {

template<>
void ArgSetFunction<std::string>(ofxXmlSettings &settings, int which_id,
                                 std::string &arg, std::string tag) {
  settings.addValue(tag, arg);
}

template<>
void ArgGetFunction<std::string>(ofxXmlSettings &settings, int which_id,
                                 std::string &arg, std::string tag) {
  arg = settings.getValue(tag, "");
}

template<typename T_arg>
void Xml<ssb_common_vec::VecTentacle, T_arg>::setXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecTentacle &item, T_arg &arg, std::string tag) {
  settings.addTag("tentacle");
  settings.pushTag("tentacle", which_id);
  ArgSetFunction<T_arg>(settings, which_id, arg, tag);
  for (int i = 0; i < 7; ++i)
    settings.addValue("jointDegree", item.joint[i]);
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecTentacle, T_arg>::getXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecTentacle &item, T_arg &arg, std::string tag) {
  settings.pushTag("tentacle", which_id);
  ArgGetFunction<T_arg>(settings, which_id, arg, tag);
  for (int i = 0; i < 7; ++i)
    item.joint[i] = settings.getValue("jointDegree", 0.0, i);
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecEye, T_arg>::setXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecEye &item, T_arg &arg, std::string tag) {
  settings.addTag("eye");
  settings.pushTag("eye", which_id);
  ArgSetFunction<T_arg>(settings, which_id, arg, tag);
  settings.addValue("verticalPosition", item.vertical);
  settings.addValue("horizontalPosition", item.horizontal);
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecEye, T_arg>::getXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecEye &item, T_arg &arg, std::string tag) {
  settings.pushTag("eye", which_id);
  ArgGetFunction<T_arg>(settings, which_id, arg, tag);
  item.vertical = settings.getValue("verticalPosition", 0.0);
  item.horizontal = settings.getValue("horizontalPosition", 0.0);
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecNeck, T_arg>::setXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecNeck &item, T_arg &arg, std::string tag) {
  settings.addTag("neck");
  settings.pushTag("neck", which_id);
  ArgSetFunction<T_arg>(settings, which_id, arg, tag);
  settings.addValue("roll", item.roll);
  settings.addValue("pitch", item.pitch);
  settings.addValue("yaw", item.yaw);
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecNeck, T_arg>::getXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecNeck &item, T_arg &arg, std::string tag) {
  settings.pushTag("neck", which_id);
  ArgGetFunction<T_arg>(settings, which_id, arg, tag);
  item.roll = settings.getValue("roll", 0.0);
  item.pitch = settings.getValue("pitch", 0.0);
  item.yaw = settings.getValue("yaw", 0.0);
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecTime, T_arg>::setXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecTime &item, T_arg &arg, std::string tag) {
  ArgSetFunction<T_arg>(settings, which_id, arg, tag);
  settings.addValue("start", item.start_time);
  settings.addValue("time", item.play_time);
}

template<typename T_arg>
void Xml<ssb_common_vec::VecTime, T_arg>::getXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecTime &item, T_arg &arg, std::string tag) {
  ArgGetFunction<T_arg>(settings, which_id, arg, tag);
  item.start_time = settings.getValue("start", 0.0);
  item.play_time = settings.getValue("time", 0.0);
}

template<typename T_arg>
void Xml<ssb_common_vec::VecVoice, T_arg>::setXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecVoice &item, T_arg &arg, std::string tag) {
  settings.addTag("voice");
  settings.pushTag("voice", which_id);
  ArgSetFunction<T_arg>(settings, which_id, arg, tag);
  settings.addValue("file", item.msg);
  settings.addValue("start", item.start_time);
  settings.addValue("end", item.end_time);
  settings.addValue("volume", item.args);
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecVoice, T_arg>::getXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecVoice &item, T_arg &arg, std::string tag) {
  settings.pushTag("voice", which_id);
  ArgGetFunction<T_arg>(settings, which_id, arg, tag);
  item.msg = settings.getValue("file", "");
  item.start_time = settings.getValue("start", 0.0);
  item.end_time = settings.getValue("end", 0.0);
  item.args = settings.getValue("volume", "");
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecInterpolation, T_arg>::setXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecInterpolation &item, T_arg &arg, std::string tag) {
  if (item.type == 0)
    return;
  settings.addTag("interpolation");
  settings.pushTag("interpolation", which_id);
  ArgSetFunction<T_arg>(settings, which_id, arg, tag);
  settings.addValue("type", item.type);
  for (int i = 0; i < item.p.size(); ++i) {
    settings.addTag("p");
    settings.pushTag("p", i);
    settings.addValue("x", item.p.at(i).x);
    settings.addValue("y", item.p.at(i).y);
    settings.popTag();
  }
  settings.popTag();
}

template<typename T_arg>
void Xml<ssb_common_vec::VecInterpolation, T_arg>::getXmlSettings(
    ofxXmlSettings &settings, int which_id,
    ssb_common_vec::VecInterpolation &item, T_arg &arg, std::string tag) {
  if (settings.getNumTags("interpolation") <= 0)
    return;
  settings.pushTag("interpolation", which_id);
  ArgGetFunction<T_arg>(settings, which_id, arg, tag);
  item.type = settings.getValue("type", 0);
  int points = settings.getNumTags("p");
  for (int i = 0; i < settings.getNumTags("p"); ++i) {
    settings.pushTag("p", i);
    ofVec2f tmp(settings.getValue("x", 0.0), settings.getValue("y", 0.0));
    item.p.push_back(tmp);
    settings.popTag();
  }
  settings.popTag();
}

} // namespace ssb_utils_xml

#endif
