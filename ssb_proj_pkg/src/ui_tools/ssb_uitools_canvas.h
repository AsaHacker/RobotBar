#ifndef SSB_UITOOLS_CANVAS_H_
#define SSB_UITOOLS_CANVAS_H_

#include "ssb_common_common.h"
#include "ssb_common_files.h"
#include "ssb_utils_xml.h"
#include "ssb_utils_interpolation.h"
#include "ssb_utils_model.h"
#include "ofxUI.h"

namespace ssb_uitools_canvas {

// Usage:
// Template class for creating GUIs that control robot part structures.
// Valid template types are the robot-part-structure-vectors.
// (These vectors should be defined in ssb_common_vec.h) 
// Note that the template type refers to the input types, or the values
// achieved from the GUI. (For example, a VecEye for x-y grids)
// You will need to create a derived class from the Canvas class, and
// in the derived class, should define a second type for outputs.
// (Eyes get a x-value and y-value, but have four output values since there
// are two eyes. Therefore has a QuadOEyeModel type for outputs.)
// So the basic steps for using Canvas classes are:
//  1. Create the input vector class type. (ssb_common_vec.h)
//  2. Create the output model class type. (ssb_utils_model.h)
//  3. Create a derived Canvas class.
// Design:
// Some of the functions are interfaces, due to the difference between robot
// parts, and therfore will need to be defined in the derived classes.
// The guiSet, guiEvent, and guiReset are all interfaces, due to the fact
// that the kind of graphical interface suitable for each part may differ:
// some work well with sliders, others might work better with grids.
// Also, how the values from the UI are dealt differ between parts,
// therefore the sets and gets of UI values are also interfaces.
// By the way, you should not have multiple GUIs in derived classes, since
// this will cause extra coding for overloading functions.
// Although, you do have an exception for creating multiple GUIs for
// such cases as having memory allocating problems with single GUIs.
// Unlike the XML class, the Canvas class is a type of dynamic polymorphism.
// The reason for this, is because Canvas classes can get rather complex
// and will most likely need flexibility when coding.
template<typename T> class Canvas {
 public:
  virtual void guiSet() = 0;
  virtual void guiEvent(ofxUIEventArgs &e) = 0;
  virtual void guiReset() = 0;
  void guiLoad(T load) { setUIValue(load); };
  virtual void guiEnd() { delete gui_; };
  inline void set_position(ofVec2f pos) { position_ = pos; };
  inline void set_color(ofxUIColor valid_back_color, ofxUIColor valid_widget_color,
                       ofxUIColor not_valid_back_color, ofxUIColor not_valid_widget_color) {
    valid_back_color_ = valid_back_color;
    valid_widget_color_ = valid_widget_color;
    not_valid_back_color_ = not_valid_back_color;
    not_valid_widget_color_ = not_valid_widget_color;
  };
  inline void set_icon_grab(int icon_grab_x, int icon_grab_y, int icon_grab_w, int icon_grab_h) {
    icon_grab_x_ = icon_grab_x;
    icon_grab_y_ = icon_grab_y;
    icon_grab_w_ = icon_grab_w;
    icon_grab_h_ = icon_grab_h;
  };
  virtual void setUIValue(T value) = 0;
  virtual T getUIValue() = 0;
  virtual void sendUIValue() = 0;
  virtual void setVisible(bool visible) { gui_->setVisible(visible); };
  virtual bool isVisible() { return gui_->isVisible(); };
  virtual void setValid(bool valid);
  virtual bool isValid() { return valid_; };
  virtual void CreateIcon(ofLight &light) = 0;
  virtual void SaveIcon(string filename) {
    ofImage img;
    img.grabScreen(icon_grab_x_, icon_grab_y_, icon_grab_w_, icon_grab_h_);
    img.saveImage(filename);
  };
 protected:
  string canvas_name_;
  ofVec2f position_;
  ofxUIColor valid_back_color_, valid_widget_color_;
  ofxUIColor not_valid_back_color_, not_valid_widget_color_;
  ofxUISuperCanvas *gui_;
  int icon_grab_x_, icon_grab_y_, icon_grab_w_, icon_grab_h_;
  bool valid_;
};


template<typename T_outputmodel>
class TentacleCanvas : public Canvas<ssb_common_vec::VecTentacle> {
 public:
  explicit TentacleCanvas(ros::NodeHandle &nh);
  ~TentacleCanvas() {};
  void guiSet();
  void guiEvent(ofxUIEventArgs &e);
  void guiReset();
  void setUIValue(ssb_common_vec::VecTentacle value);
  ssb_common_vec::VecTentacle getUIValue();
  void sendUIValue();
  void CreateIcon(ofLight &light); 
 private:
  ofVec2f canvas_position_;
  string widget_name_[7];
  T_outputmodel model_tentacle_;
  // Variables and functions for Icon Creation.
  ofVec3f virtual_rotation_point_[8];
  float virtual_link_length_[7];
  float total_link_length_;
  void setVirtuals();
  // Variables and functions for registered tentacles
  ofVec2f canvas_position_registered_;
  std::vector<ssb_common_vec::VecTentacle> vecs_registered_;
  std::vector<std::string> imgnames_registered_;
  void setRegistered();
  void guiSetRegistered();
  void guiEventRegistered(ofxUIEventArgs &e);
  void UpdateVirtualRotationPoint();
};


template<typename T_outputmodel>
class EyeCanvas : public Canvas<ssb_common_vec::VecEye> {
 public:
  explicit EyeCanvas(ros::NodeHandle &nh);
  ~EyeCanvas() {};
  void guiSet();
  void guiEvent(ofxUIEventArgs &e);
  void guiReset();
  void setUIValue(ssb_common_vec::VecEye value);
  ssb_common_vec::VecEye getUIValue();
  void sendUIValue();
  void CreateIcon(ofLight &light); 
 private:
  string widget_name_;
  T_outputmodel model_eye;
};


template<typename T_outputmodel>
class NeckCanvas : public Canvas<ssb_common_vec::VecNeck> {
 public:
  explicit NeckCanvas(ros::NodeHandle &nh);
  ~NeckCanvas() {};
  void guiSet();
  void guiEvent(ofxUIEventArgs &e);
  void guiReset();
  void setUIValue(ssb_common_vec::VecNeck value);
  ssb_common_vec::VecNeck getUIValue();
  void sendUIValue();
  void CreateIcon(ofLight &light); 
 private:
  string widget_name_[3];
  T_outputmodel model_neck_;
};


class TimeCanvas : public Canvas<ssb_common_vec::VecTime> {
 public:
  TimeCanvas(float &_time_span, int &_timeline_width);
  ~TimeCanvas() {};
  void guiSet();
  void guiEvent(ofxUIEventArgs &e);
  void guiReset();
  void setUIValue(ssb_common_vec::VecTime value);
  ssb_common_vec::VecTime getUIValue();
  void sendUIValue() {};
  void CreateIcon(ofLight &light) {};
  void SaveIcon(string filename) {};
  inline void ResetTrigger() { canvas_close_trigger_ = false; };
  inline bool CheckTrigger() { return canvas_close_trigger_; };
  void ScrollTimeline(int to_x);
 private:
  ssb_common_vec::VecTime time_vec_;
  string start_widget_name_;
  string time_widget_name_;
  bool canvas_close_trigger_;
  float &time_span;
  int &timeline_width;
};


class VoiceCanvas : public Canvas<ssb_common_vec::VecVoice> {
 public:
  VoiceCanvas(ros::NodeHandle &nh);
  ~VoiceCanvas() {};
  void guiSet();
  void guiEvent(ofxUIEventArgs &e);
  void guiReset();
  void setUIValue(ssb_common_vec::VecVoice value);
  ssb_common_vec::VecVoice getUIValue();
  void sendUIValue();
  void setValid(bool valid);
  void CreateIcon(ofLight &light) {};
  void SaveIcon(string filename) {};
  float get_voice_length(string voice) const;
 private:
  vector<string> widget_names_;
  map<string, float> voice_length_;
  ssb_utils_model::VoiceModel model_voice_;
};


class InterpolationCanvas : public Canvas<ssb_common_vec::VecInterpolation> {
 public:
  InterpolationCanvas();
  ~InterpolationCanvas() {};
  void guiSet();
  void guiEvent(ofxUIEventArgs &e);
  void guiReset();
  void setUIValue(ssb_common_vec::VecInterpolation value);
  ssb_common_vec::VecInterpolation getUIValue();
  void sendUIValue() {};
  void CreateIcon(ofLight &light) {};
  void SaveIcon(string filename) {};
  void set_draw_area(ofRectangle rect) { draw_area_ = rect; };
  void set_draw(void (InterpolationCanvas::*draw)(ofLight&)) { draw_ = draw; };
  void Draw(ofLight &light) { if (isVisible()) (this->*draw_)(light); };
 private:
  void (InterpolationCanvas::*set_edit_[7])();
  void (InterpolationCanvas::*draw_)(ofLight&);
  ssb_utils_interpolation::Interpolation interpolation_;
  int current_p_on_edit_;
  string type_name_[7];
  ofVec2f draw_position_; // relative position to canvas
  ofRectangle draw_area_; // absolute position to screen
  void SetEditConstant();
  void SetEditLinear();
  void SetEditBezier();
  void SetEditSlowIn();
  void SetEditSlowOut();
  void SetEditSigmoid();
  void SetEditCubicBezier();
  void DrawConstant(ofLight &light);
  void DrawLinear(ofLight &light);
  void DrawBezier(ofLight &light);
  void DrawSlowIn(ofLight &light);
  void DrawSlowOut(ofLight &light);
  void DrawSigmoid(ofLight &light);
  void DrawCubicBezier(ofLight &light);
};

} // namespace ssb_uitools_canvas

#endif
