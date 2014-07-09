#include "ssb_uitools_canvas.h"

namespace ssb_uitools_canvas {

template<typename T>
void Canvas<T>::setValid(bool valid) {
  if (!gui_->isVisible()) return;
  valid_ = valid;
  if (valid) {
    gui_->setColorBack(valid_back_color_);
    for (vector<ofxUIWidget *>::iterator it = gui_->getWidgets().begin();
         it != gui_->getWidgets().end(); ++it)
      (*it)->setColorBack(valid_widget_color_);    
  } else {
    gui_->setColorBack(not_valid_back_color_);
    for (vector<ofxUIWidget*>::iterator it = gui_->getWidgets().begin();
         it != gui_->getWidgets().end(); ++it)
      (*it)->setColorBack(not_valid_widget_color_);
  }
}

//---------------------------------------------------------------

template<typename T_outputmodel>
TentacleCanvas<T_outputmodel>::TentacleCanvas(ros::NodeHandle &nh) : model_tentacle_(nh) {
  model_tentacle_.setParams(ssb_common_enum::DEBUG);
  set_position(ofVec2f(1100, 0)); // 560 + 540
  set_color(ofxUIColor(157, 144, 192, 100), ofxUIColor(157, 144, 192, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  set_icon_grab(0, 0, 128, 128);
  canvas_name_ = "TENTACLE CONTROL";
  canvas_position_ = ofVec2f(0, 30);
  for (int i = 0; i < 7; ++i)
    widget_name_[i] = "joint " + ofToString(i);
  valid_ = true;
  setRegistered();
  setVirtuals();
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::setRegistered() {
  canvas_position_registered_ = ofVec2f(-560, 430);
  ofxXmlSettings settings;
  if (settings.loadFile("xml/registered_tentacles.xml")) {
    settings.pushTag("settings");
    settings.pushTag("registered");
    int num_of_tentacle_tags = settings.getNumTags("tentacle");
    vecs_registered_.resize(num_of_tentacle_tags);
    imgnames_registered_.resize(num_of_tentacle_tags);
    for( int i=0; i<num_of_tentacle_tags; i++ ){
      ssb_utils_xml::Xml<ssb_common_vec::VecTentacle, string> xml;
      xml.getXmlSettings(settings, i, vecs_registered_.at(i),
                         imgnames_registered_.at(i) , "img");
    }
    settings.popTag();
    settings.popTag();
  }
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::setVirtuals() {
  virtual_rotation_point_[0] = ofVec3f(0, 0, 0);
  virtual_link_length_[0] = 22.5;
  virtual_rotation_point_[1] = ofVec3f(0, 0, 22.5);
  virtual_link_length_[1] = 21.5;
  virtual_rotation_point_[2] = ofVec3f(0, 0, 44);
  virtual_link_length_[2] = 14;
  virtual_rotation_point_[3] = ofVec3f(0, 0, 58);
  virtual_link_length_[3] = 23;
  virtual_rotation_point_[4] = ofVec3f(0, 0, 81);
  virtual_link_length_[4] = 14;
  virtual_rotation_point_[5] = ofVec3f(0, 0, 95);
  virtual_link_length_[5] = 14;
  virtual_rotation_point_[6] = ofVec3f(0, 0, 109);
  virtual_link_length_[6] = 14;
  virtual_rotation_point_[7] = ofVec3f(0, 0, 123);
  total_link_length_ = 123;
  model_tentacle_.set_input(model_tentacle_.getHomeAsInput());
  UpdateVirtualRotationPoint();
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::guiSet() {
  gui_ = new ofxUISuperCanvas(canvas_name_, position_.x, position_.y,
                              300, 720);
  gui_->addSpacer();
  gui_->setFontSize(OFX_UI_FONT_LARGE, OFX_UI_FONT_LARGE_SIZE * 3);
  gui_->setFontSize(OFX_UI_FONT_SMALL, OFX_UI_FONT_LARGE_SIZE); 
  gui_->setWidgetFontSize(OFX_UI_FONT_SMALL);
  gui_->setColorBack(valid_back_color_);
  for (int i = 0; i < 7; ++i) {
    gui_->addRotarySlider(widget_name_[i], -90.0, 90.0,
                          static_cast<float>(model_tentacle_.getHomeAsInput().joint[i]),
                          140, 0, 0);                          
    gui_->getWidget(widget_name_[i])
        ->getRect()->setX(canvas_position_.x+150*(i/4));
    gui_->getWidget(widget_name_[i])
        ->getRect()->setY(canvas_position_.y+165*(i%4));
    gui_->getWidget(widget_name_[i])->setColorBack(valid_widget_color_);
  }
  gui_->addSpacer();
  gui_->addLabelButton("save", false);
  gui_->getWidget("save")->getRect()->setX(canvas_position_.x+5);
  gui_->getWidget("save")->getRect()->setY(canvas_position_.y+165*4);
  gui_->getWidget("save")->setColorBack(valid_widget_color_);
  guiSetRegistered();
  gui_->setPosition(position_.x, position_.y);
  ofAddListener(gui_->newGUIEvent, this, &TentacleCanvas<T_outputmodel>::guiEvent);
}

// The following function has a Bug report: Allocation to non-valid memory.
// The sixth button is created to a non-valid memory such as 0x2000000000000000.
// Because of this, we have a few problems.
// First, the setValid function cannot access to the sixth widget
// Second, the button creation event in the guiEventRegistered function
// fails to create a new seventh button, possibly because it cannot link to the
// prior buttons via sixth button.
// Further survey is needed to solve this memory allocation problem.
template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::guiSetRegistered() {
  gui_->setGlobalButtonDimension(128);
  for (int i = 0; i < vecs_registered_.size(); ++i) {
    gui_->addImageButton("tentacle"+ofToString(i),
                         imgnames_registered_.at(i),
                         false);
    gui_->getWidget("tentacle"+ofToString(i))
        ->getRect()->setX(canvas_position_registered_.x+138*(i%4));
    gui_->getWidget("tentacle"+ofToString(i))
        ->getRect()->setY(canvas_position_registered_.y+138*(i/4));
    gui_->getWidget("tentacle"+ofToString(i))->setColorBack(valid_widget_color_);
  }
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::guiEvent(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  for (int i = 0; i < 7; ++i)
    if (name == widget_name_[i]) {
      ofxUIRotarySlider *slider = (ofxUIRotarySlider*) e.widget;
      ssb_common_vec::VecTentacle input = model_tentacle_.get_input();
      input.joint[i] = slider->getValue();
      model_tentacle_.set_input(input);
      UpdateVirtualRotationPoint();
      model_tentacle_.Input2Output();
      model_tentacle_.send();
      return;
    }
  guiEventRegistered(e);
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::guiEventRegistered(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  if (name.substr(0, 8) == "tentacle") {
    ofxUILabelButton *button = (ofxUILabelButton*) e.widget;
    if (button->getValue()) {
      for (int i = 0; i < vecs_registered_.size(); ++i)
	if (name == ("tentacle" + ofToString(i))) {
          ssb_common_vec::VecTentacle input = vecs_registered_.at(i);
          for (int i = 0; i < 7; ++i)
            ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[i]))->setValue(input.joint[i]);
          model_tentacle_.set_input(input);
          UpdateVirtualRotationPoint();
          model_tentacle_.Input2Output();
          model_tentacle_.send();
	  return;
	}
    } // if (button->getValue())
  } // if (name.substr(0, 8) == "tentacle")
  if (name == "save") {
    ofxUILabelButton *button = (ofxUILabelButton*) e.widget; 
    if (button->getValue()) {
      int num_of_tentacle_tags = vecs_registered_.size();
      ofxXmlSettings settings;
      settings.clear();
      settings.addTag("settings");
      settings.pushTag("settings");
      settings.addTag("registered");
      settings.pushTag("registered");
      ssb_utils_xml::Xml<ssb_common_vec::VecTentacle, string> xml;
      for (int i = 0; i < num_of_tentacle_tags; ++i)
        xml.setXmlSettings(settings, i, vecs_registered_.at(i),
                           imgnames_registered_.at(i), "img");
      ssb_common_vec::VecTentacle new_register;
      for (int i = 0; i < 7; ++i)
        new_register.joint[i] =
            ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[i]))->getValue();
      string new_register_imgname(
          "GUI/images/tentacle"+ofToString(num_of_tentacle_tags)+".png");
      xml.setXmlSettings(settings, num_of_tentacle_tags, new_register,
                         new_register_imgname, "img");
      settings.popTag();
      settings.popTag();
      settings.saveFile("xml/registered_tentacles.xml");
      vecs_registered_.push_back(new_register);
      imgnames_registered_.push_back(new_register_imgname);
      SaveIcon("GUI/images/tentacle"+ofToString(num_of_tentacle_tags)+".png");
      gui_->addImageButton("tentacle"+ofToString(num_of_tentacle_tags),
                           imgnames_registered_.at(num_of_tentacle_tags),
                           false);
      gui_->getWidget("tentacle"+ofToString(num_of_tentacle_tags))
          ->getRect()->setX(
              canvas_position_registered_.x+138*(num_of_tentacle_tags%4));
      gui_->getWidget("tentacle"+ofToString(num_of_tentacle_tags))
          ->getRect()->setY(
              canvas_position_registered_.y+138*(num_of_tentacle_tags/4));
      gui_->getWidget("tentacle"+ofToString(num_of_tentacle_tags))
          ->setColorBack(valid_widget_color_);
      return;
    } // if (button->getValue())
  } // if (name == "save")
} // void TentacleCanvas<T_outputmodel>::guiEventRegistered(ofxUIEventsArgs &e)

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::guiReset() {
  for (int i = 0; i < 7; ++i)
    ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[i]))->setValue(0);
  model_tentacle_.set_input(ssb_common_vec::VecTentacle(0, 0, 0, 0, 0, 0, 0));
  UpdateVirtualRotationPoint();
  model_tentacle_.Input2Output();
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::setUIValue(ssb_common_vec::VecTentacle value) {
  for (int i = 0; i < 7; ++i)
    ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[i]))->setValue(value.joint[i]);
  model_tentacle_.set_input(value);
  UpdateVirtualRotationPoint();
  model_tentacle_.Input2Output();
}

template<typename T_outputmodel>
ssb_common_vec::VecTentacle TentacleCanvas<T_outputmodel>::getUIValue() {
  ssb_common_vec::VecTentacle ui_value;
  for (int i = 0; i < 7; ++i)
    ui_value.joint[i] = ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[i]))->getValue();
  return ui_value;
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::sendUIValue() { model_tentacle_.send(); }

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::CreateIcon(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  for (int i = 0; i < 7; ++i) {
    ofPolyline line;
    ofVec3f scaled_start_point = virtual_rotation_point_[i] * icon_grab_h_ / total_link_length_;
    ofVec3f scaled_end_point = virtual_rotation_point_[i+1] * icon_grab_h_ / total_link_length_;
    float icon_root_x = icon_grab_x_ + icon_grab_w_ / 2;
    float icon_root_y = icon_grab_y_ + icon_grab_h_;
    line.addVertex(ofVec2f(icon_root_x - scaled_start_point.x,
                           icon_root_y - scaled_start_point.z));
    line.addVertex(ofVec2f(icon_root_x - scaled_end_point.x,
                           icon_root_y - scaled_end_point.z));
    line.close();
    line.draw();
  }
}

template<typename T_outputmodel>
void TentacleCanvas<T_outputmodel>::UpdateVirtualRotationPoint() {
  ofVec3f pos_j = virtual_rotation_point_[0];
  ssb_common_vec::VecTentacle ui_value = model_tentacle_.get_input();  
  float base_angle_radian = ui_value.joint[0] * M_PI/180;
  ofVec4f R_j = ofVec4f(cos(base_angle_radian), -sin(base_angle_radian),
                        sin(base_angle_radian), cos(base_angle_radian));
  for (int i = 1; i < 7; ++i) {
    int j = i - 1;
    float i_angle_radian = ui_value.joint[i] * M_PI/180;
    virtual_rotation_point_[i] = ofVec3f(virtual_link_length_[j]*R_j.y, 0, virtual_link_length_[j]*R_j.w) + pos_j; 
    pos_j = virtual_rotation_point_[i];
    ofVec4f R_i = ofVec4f(R_j.x * cos(i_angle_radian) + R_j.y * sin(i_angle_radian),
                          -R_j.x * sin(i_angle_radian) + R_j.y * cos(i_angle_radian),
                          R_j.z * cos(i_angle_radian) + R_j.w * sin(i_angle_radian),
                          -R_j.z * sin(i_angle_radian) + R_j.w * cos(i_angle_radian));
    R_j = R_i;
  }
  virtual_rotation_point_[7] = ofVec3f(virtual_link_length_[6]*R_j.y, 0, virtual_link_length_[6]*R_j.w) + pos_j;
}

//---------------------------------------------------------------

template<typename T_outputmodel>
EyeCanvas<T_outputmodel>::EyeCanvas(ros::NodeHandle &nh) : model_eye(nh) {
  model_eye.setParams(ssb_common_enum::DEBUG);
  set_position(ofVec2f(20, 140));
  set_color(ofxUIColor(157, 144, 192, 100), ofxUIColor(157, 144, 192, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  set_icon_grab(138, 0, 128, 128);
  canvas_name_ = "EYE CONTROL";
  widget_name_ = "eye";
  valid_ = true;
}

template<typename T_outputmodel>
void EyeCanvas<T_outputmodel>::guiSet() {
  gui_ = new ofxUISuperCanvas(canvas_name_);
  gui_->addSpacer();
  gui_->setFontSize(OFX_UI_FONT_LARGE, OFX_UI_FONT_LARGE_SIZE * 3);
  gui_->setFontSize(OFX_UI_FONT_SMALL, OFX_UI_FONT_LARGE_SIZE * 2);  
  gui_->setWidgetFontSize(OFX_UI_FONT_SMALL);
  gui_->add2DPad(widget_name_, ofPoint(-1.0, 1.0), ofPoint(1.0, -1.0), ofPoint(0, 0), 500, 500);
  gui_->autoSizeToFitWidgets();
  gui_->setPosition(position_.x, position_.y);
  gui_->setColorBack(valid_back_color_);
  gui_->getWidget(widget_name_)->setColorBack(valid_widget_color_);
  ofAddListener(gui_->newGUIEvent, this, &EyeCanvas<T_outputmodel>::guiEvent);
}

template<typename T_outputmodel>
void EyeCanvas<T_outputmodel>::guiEvent(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  if (name == widget_name_) {
    ofxUI2DPad *pad = (ofxUI2DPad *) e.widget;
    model_eye.set_input(ssb_common_vec::VecEye(pad->getScaledValue().x,
                                               pad->getScaledValue().y));
    model_eye.Input2Output();
    model_eye.send();
  }
}

template<typename T_outputmodel>
void EyeCanvas<T_outputmodel>::guiReset() {
  ((ofxUI2DPad*)gui_->getWidget(widget_name_))->setValue(ofxUIVec3f(0, 0, 0));
  model_eye.set_input(ssb_common_vec::VecEye(0, 0));
  model_eye.Input2Output();
}

template<typename T_outputmodel>
void EyeCanvas<T_outputmodel>::setUIValue(ssb_common_vec::VecEye value) {
  ((ofxUI2DPad*)gui_->getWidget(widget_name_))->setValue(ofxUIVec3f(value.horizontal,
                                                                   value.vertical,0));
  model_eye.set_input(value);
  model_eye.Input2Output();
}

template<typename T_outputmodel>
ssb_common_vec::VecEye EyeCanvas<T_outputmodel>::getUIValue() {
  ofxUIVec3f ui_value;
  ui_value = ((ofxUI2DPad*)gui_->getWidget(widget_name_))->getScaledValue();
  return ssb_common_vec::VecEye(ui_value.x, ui_value.y);
}

template<typename T_outputmodel>
void EyeCanvas<T_outputmodel>::sendUIValue() {
  model_eye.send();
}

template<typename T_outputmodel>
void EyeCanvas<T_outputmodel>::CreateIcon(ofLight &light) {
  int icon_center_x = icon_grab_x_ + icon_grab_w_ / 2;
  int icon_center_y = icon_grab_y_ + icon_grab_h_ / 2;
  ofPolyline linex, liney;
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  linex.addVertex(ofVec2f(icon_grab_x_, icon_center_y));
  linex.addVertex(ofVec2f(icon_grab_x_ + icon_grab_w_, icon_center_y));
  linex.close();
  linex.draw();
  liney.addVertex(ofVec2f(icon_center_x, icon_grab_y_));
  liney.addVertex(ofVec2f(icon_center_x, icon_grab_y_ + icon_grab_h_));
  liney.close();
  liney.draw();
  ssb_common_vec::VecEye ui_value = getUIValue();
  light.setAmbientColor(ofColor(255, 0, 0));
  light.setDiffuseColor(ofColor(255, 0, 0));
  ofCircle(icon_center_x + ui_value.horizontal*icon_grab_w_/2,
           icon_center_y - ui_value.vertical*icon_grab_h_/2,
           4);
}

//---------------------------------------------------------------

template<typename T_outputmodel>
NeckCanvas<T_outputmodel>::NeckCanvas(ros::NodeHandle &nh) : model_neck_(nh) {
  model_neck_.setParams(ssb_common_enum::DEBUG);
  set_position(ofVec2f(560, 0));
  set_color(ofxUIColor(157, 144, 192, 100), ofxUIColor(157, 144, 192, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  set_icon_grab(276, 0, 128, 128);
  canvas_name_ = "NECK CONTROL";
  widget_name_[0] = "neck r";
  widget_name_[1] = "neck p:";
  widget_name_[2] = "neck y";
  valid_ = true;
}

template<typename T_outputmodel>
void NeckCanvas<T_outputmodel>::guiSet() {
  gui_ = new ofxUISuperCanvas(canvas_name_);
  gui_->addSpacer();
  gui_->setFontSize(OFX_UI_FONT_LARGE, OFX_UI_FONT_LARGE_SIZE * 3);
  gui_->setFontSize(OFX_UI_FONT_SMALL, OFX_UI_FONT_LARGE_SIZE);  
  gui_->setWidgetFontSize(OFX_UI_FONT_SMALL);
  gui_->addRotarySlider(widget_name_[0], -2.0, 2.0, static_cast<float>(0.0), 300, 0, 0);
  gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
  gui_->addSlider(widget_name_[1], -1.0, 1.0, 0.0, 20, 300);
  gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
  gui_->addSlider(widget_name_[2], -1.0, 1.0, 0.0, 300, 20);
  gui_->autoSizeToFitWidgets();
  gui_->setPosition(position_.x, position_.y);
  gui_->setColorBack(valid_back_color_);
  gui_->getWidget(widget_name_[0])->setColorBack(valid_widget_color_);
  gui_->getWidget(widget_name_[1])->setColorBack(valid_widget_color_);
  gui_->getWidget(widget_name_[2])->setColorBack(valid_widget_color_);
  ofAddListener(gui_->newGUIEvent, this, &NeckCanvas<T_outputmodel>::guiEvent);
}

template<typename T_outputmodel>
void NeckCanvas<T_outputmodel>::guiEvent(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  if (name == widget_name_[0]) {
    ofxUIRotarySlider *slider = (ofxUIRotarySlider*) e.widget;
    ssb_common_vec::VecNeck input = model_neck_.get_input();
    input.roll = slider->getValue();
    model_neck_.set_input(input);
    model_neck_.Input2Output();
    model_neck_.send();
  } else if (name == widget_name_[1]) {
    ofxUISlider *slider = (ofxUISlider*) e.widget;
    ssb_common_vec::VecNeck input = model_neck_.get_input();
    input.pitch = slider->getValue();
    model_neck_.set_input(input);
    model_neck_.Input2Output();
    model_neck_.send();
  } else if (name == widget_name_[2]) {
    ofxUISlider *slider = (ofxUISlider*) e.widget;
    ssb_common_vec::VecNeck input = model_neck_.get_input();
    input.yaw = slider->getValue();
    model_neck_.set_input(input);
    model_neck_.Input2Output();
    model_neck_.send();
  }
}

template<typename T_outputmodel>
void NeckCanvas<T_outputmodel>::guiReset() {
  ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[0]))->setValue(0.0);
  ((ofxUISlider*)gui_->getWidget(widget_name_[1]))->setValue(0.0);
  ((ofxUISlider*)gui_->getWidget(widget_name_[2]))->setValue(0.0);
  model_neck_.set_input(ssb_common_vec::VecNeck(0, 0, 0));
  model_neck_.Input2Output();
}

template<typename T_outputmodel>
void NeckCanvas<T_outputmodel>::setUIValue(ssb_common_vec::VecNeck value) {
  ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[0]))->setValue(value.roll);
  ((ofxUISlider*)gui_->getWidget(widget_name_[1]))->setValue(value.pitch);
  ((ofxUISlider*)gui_->getWidget(widget_name_[2]))->setValue(value.yaw);
  model_neck_.set_input(value);
  model_neck_.Input2Output();
}

template<typename T_outputmodel>
ssb_common_vec::VecNeck NeckCanvas<T_outputmodel>::getUIValue() {
  ssb_common_vec::VecNeck ui_value;
  ui_value.roll = ((ofxUIRotarySlider*)gui_->getWidget(widget_name_[0]))->getValue();
  ui_value.pitch = ((ofxUISlider*)gui_->getWidget(widget_name_[1]))->getValue();
  ui_value.yaw = ((ofxUISlider*)gui_->getWidget(widget_name_[2]))->getValue();
  return ui_value;
}

template<typename T_outputmodel>
void NeckCanvas<T_outputmodel>::sendUIValue() { model_neck_.send(); };

template<typename T_outputmodel>
void NeckCanvas<T_outputmodel>::CreateIcon(ofLight &light) {
  ofVec2f rect0_center, rect1_center;
  ssb_common_vec::VecNeck ui_value = getUIValue();
  rect0_center.x = icon_grab_x_ + icon_grab_w_ / 2;
  rect0_center.y = icon_grab_y_ + icon_grab_h_ / 2;
  rect1_center.x = rect0_center.x + (icon_grab_w_ / 2) * ui_value.yaw;
  rect1_center.y = rect0_center.y - (icon_grab_h_ / 2) * ui_value.pitch;
  ofPushMatrix();{
    ofTranslate(rect0_center.x, rect0_center.y, 0);
    ofRotate(ui_value.roll * 45, 0, 0, 1);
    light.setAmbientColor(ofColor(0, 0, 0));
    light.setDiffuseColor(ofColor(0, 0, 0));
    ofRect(0, 0, icon_grab_w_/2, icon_grab_h_/2);
  }ofPopMatrix();
  ofPushMatrix();{
    ofTranslate(rect1_center.x, rect1_center.y, 0);
    ofRotate(ui_value.roll * 45, 0, 0, 1);
    light.setAmbientColor(ofColor(128, 0, 128));
    light.setDiffuseColor(ofColor(128, 0, 128));
    ofRect(0, 0, icon_grab_w_/2, icon_grab_h_/2);
  }ofPopMatrix();
}

} // namespace ssb_uitools_canvas
