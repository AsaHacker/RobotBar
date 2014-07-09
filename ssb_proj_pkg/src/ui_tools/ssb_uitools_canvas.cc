#include "ssb_uitools_canvas.h"
#include "ssb_uitools_canvas-inl.h"

namespace ssb_uitools_canvas {

TimeCanvas::TimeCanvas(float &_time_span, int &_timeline_width)
    : time_span(_time_span), timeline_width(_timeline_width) {
  set_position(ofVec2f(0, 730));
  set_color(ofxUIColor(157, 144, 192, 100), ofxUIColor(157, 144, 192, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  canvas_name_ = "time control";
  start_widget_name_ = "start: ";
  time_widget_name_ = "time: ";
  canvas_close_trigger_ = false;
  valid_ = true;
  gui_ = NULL;
}

void TimeCanvas::guiSet() {
  if (gui_)
    return;
  gui_ = new ofxUISuperCanvas(canvas_name_);
  gui_->addSpacer();
  gui_->setFontSize(OFX_UI_FONT_LARGE, OFX_UI_FONT_LARGE_SIZE * 3);
  gui_->setFontSize(OFX_UI_FONT_SMALL, OFX_UI_FONT_LARGE_SIZE);  
  gui_->setWidgetFontSize(OFX_UI_FONT_SMALL);
  gui_->addSlider(start_widget_name_, 0, time_span, 0.0, timeline_width, 20, 0, 0);
  gui_->addSlider(time_widget_name_, 0, time_span, 0.0, timeline_width, 20, 0, 30);
  gui_->addLabelButton("ok", false);
  gui_->autoSizeToFitWidgets();
  gui_->setPosition(position_.x, position_.y);
  gui_->setColorBack(valid_back_color_);
  gui_->getWidget(start_widget_name_)->setColorBack(valid_widget_color_);
  gui_->getWidget(time_widget_name_)->setColorBack(valid_widget_color_);
  gui_->getWidget("ok")->setColorBack(valid_widget_color_);
  ofAddListener(gui_->newGUIEvent, this, &TimeCanvas::guiEvent);
}

void TimeCanvas::guiEvent(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  if (name == start_widget_name_) {
    ofxUISlider *slider = (ofxUISlider*) e.widget;
    time_vec_.start_time = slider->getValue();
    float keep4tmp_play_time_sec = ((ofxUISlider*)gui_->getWidget(time_widget_name_))->getValue();
    int start_time_px = time_vec_.start_time * timeline_width / time_span;
    gui_->getWidget(time_widget_name_)->getRect()->setX(start_time_px);
    gui_->getWidget(time_widget_name_)->getRect()->setWidth(timeline_width - start_time_px);
    ((ofxUISlider*)gui_->getWidget(time_widget_name_))->setMax(time_span - time_vec_.start_time,
                                                             false);
    ((ofxUISlider*)gui_->getWidget(time_widget_name_))->setValue(keep4tmp_play_time_sec);
    return;
  }
  if (name == time_widget_name_) {
    ofxUISlider *slider = (ofxUISlider*) e.widget;
    time_vec_.play_time = slider->getValue();
    return;
  }
  if (name == "ok") {
    ofxUILabelButton *button = (ofxUILabelButton*) e.widget; 
    if (button->getValue()) {
      canvas_close_trigger_ = true;
      return;
    }
  }
}

void TimeCanvas::guiReset() {
  ((ofxUISlider*)gui_->getWidget(start_widget_name_))->setValue(0.0);
  ((ofxUISlider*)gui_->getWidget(time_widget_name_))->setValue(0.0);
  time_vec_ = ssb_common_vec::VecTime(0.0, 0.0);
}

void TimeCanvas::setUIValue(ssb_common_vec::VecTime value) {
  ((ofxUISlider*)gui_->getWidget(start_widget_name_))->setValue(value.start_time);
  int start_time_px = value.start_time * timeline_width / time_span;
  gui_->getWidget(time_widget_name_)->getRect()->setX(start_time_px);
  gui_->getWidget(time_widget_name_)->getRect()->setWidth(timeline_width - start_time_px);
  ((ofxUISlider*)gui_->getWidget(time_widget_name_))->setMax(time_span - value.start_time, false);
  ((ofxUISlider*)gui_->getWidget(time_widget_name_))->setValue(value.play_time);
  time_vec_ = value;
}

ssb_common_vec::VecTime TimeCanvas::getUIValue() {
  return ssb_common_vec::VecTime(((ofxUISlider*)gui_->getWidget(start_widget_name_))->getValue(),
                                 ((ofxUISlider*)gui_->getWidget(time_widget_name_))->getValue());
}

// This is a unique funtion used for KeyFrameEditor.
// By using this function, when the editor slides, the timeline in this canvas also slides.
// Instead of creating another gui for non-scrolling buttons
// (which will need some overriding parent functions: this is not beautiful)
// the function just transfers the timeline widgets according to the editor.
// Therefore some tricky steps are needed for maintaining the play_time_widget and its value.
void TimeCanvas::ScrollTimeline(int to_x) {
  ((ofxUISlider*)gui_->getWidget(start_widget_name_))->getRect()->setX(to_x);
  float keep4tmp_play_time_sec = ((ofxUISlider*)gui_->getWidget(time_widget_name_))->getValue();
  int start_time_px = time_vec_.start_time * timeline_width / time_span;
  gui_->getWidget(time_widget_name_)->getRect()->setX(start_time_px + to_x);
  gui_->getWidget(time_widget_name_)->getRect()->setWidth(timeline_width - start_time_px);
  ((ofxUISlider*)gui_->getWidget(time_widget_name_))->setMax(time_span - time_vec_.start_time,
                                                             false);
  ((ofxUISlider*)gui_->getWidget(time_widget_name_))->setValue(keep4tmp_play_time_sec);
}

//---------------------------------------------------------------

VoiceCanvas::VoiceCanvas(ros::NodeHandle &nh) : model_voice_(nh) {
  set_position(ofVec2f(1500, 0));
  set_color(ofxUIColor(157, 144, 192, 100), ofxUIColor(157, 144, 192, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  canvas_name_ = "VOICE CONTROL";
  ssb_common_files::Files file_handler(nh);
  widget_names_ = file_handler.getList("sound/voice", ".wav");
  std::vector<float> length_list_of_wav = file_handler.getWavPlayTime(widget_names_);
  voice_length_.insert(map<string, float>::value_type("quit", 0.0));
  for (int i = 0; i < widget_names_.size(); ++i)
    voice_length_.insert(map<string, float>::value_type(widget_names_.at(i),
                                                        length_list_of_wav.at(i)));
  valid_ = true;
  model_voice_.set_input(ssb_common_vec::VecVoice());
  model_voice_.Input2Output();
}

void VoiceCanvas::guiSet() {
  gui_ = new ofxUISuperCanvas(canvas_name_);
  gui_->addSpacer();
  gui_->setFontSize(OFX_UI_FONT_LARGE, OFX_UI_FONT_LARGE_SIZE * 3);
  gui_->setFontSize(OFX_UI_FONT_SMALL, OFX_UI_FONT_LARGE_SIZE * 2); 
  gui_->setWidgetFontSize(OFX_UI_FONT_SMALL);
  gui_->setGlobalCanvasWidth(400);
  gui_->setColorBack(valid_back_color_);
  gui_->addLabelButton("", false);
  gui_->getWidget("")->setColorBack(valid_widget_color_);
  gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
  for (int i = 0; i < widget_names_.size(); ++i) {
    gui_->addLabelButton(widget_names_.at(i), false);
    gui_->getWidget(widget_names_.at(i))->setColorBack(valid_widget_color_);
    (i%3 == 1) ? gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN)
        : gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
  }
  gui_->autoSizeToFitWidgets();
  gui_->setPosition(position_.x, position_.y);
  ofAddListener(gui_->newGUIEvent, this, &VoiceCanvas::guiEvent);
}

void VoiceCanvas::guiEvent(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  if (name == "") { // if "quit"
    ofxUILabelButton *button = (ofxUILabelButton*) e.widget;
    if (button->getValue()) {
      model_voice_.set_input(ssb_common_vec::VecVoice());
      model_voice_.Input2Output();
      model_voice_.send();
    }
  }
  for (int i = 0; i < widget_names_.size(); ++i) {
    if (name == widget_names_.at(i)) {
      ofxUILabelButton *button = (ofxUILabelButton*) e.widget;
      if (button->getValue()) {
        model_voice_.set_input(ssb_common_vec::VecVoice()); // "quit"
        model_voice_.Input2Output();
        model_voice_.send();
        model_voice_.set_input(ssb_common_vec::VecVoice(widget_names_.at(i),
                                                        0.0,
                                                        voice_length_[widget_names_.at(i)],
                                                        "1.0")); // speak *.wav
        model_voice_.Input2Output();
        model_voice_.send();
      }
    } // if (name == widget_names_.at(i))
  } // for (int i = 0; i < widget_names_.size(); ++i)
}

void VoiceCanvas::guiReset() {
  model_voice_.set_input(ssb_common_vec::VecVoice());
  model_voice_.Input2Output();
}

void VoiceCanvas::setUIValue(ssb_common_vec::VecVoice value) {
  model_voice_.set_input(value);
  model_voice_.Input2Output();
}

ssb_common_vec::VecVoice VoiceCanvas::getUIValue() {
  return model_voice_.get_input();
}

void VoiceCanvas::sendUIValue() {
  model_voice_.send();
}

// For some reason iterator for buttons have a bug and cannot use the setValid
// defined by the base class.
void VoiceCanvas::setValid(bool valid) {
  if (!gui_->isVisible()) return;
  valid_ = valid;
  if (valid) {
    gui_->setColorBack(valid_widget_color_);
    gui_->getWidget("")->setColorBack(valid_widget_color_);
    for (int i = 0; i < widget_names_.size(); ++i)
      gui_->getWidget(widget_names_.at(i))->setColorBack(valid_widget_color_);
  } else {
    gui_->setColorBack(not_valid_widget_color_);
    gui_->getWidget("")->setColorBack(not_valid_widget_color_);
    for (int i = 0; i < widget_names_.size(); ++i)
      gui_->getWidget(widget_names_.at(i))->setColorBack(not_valid_widget_color_);
  }
}

// Used in KeyFrameEditor to set the play time length of voice key.
float VoiceCanvas::get_voice_length(string voice) const {
  return voice_length_.at(voice);
}

//---------------------------------------------------------------

InterpolationCanvas::InterpolationCanvas() : interpolation_(),
                                             draw_(&InterpolationCanvas::DrawConstant) {
  set_edit_[0] = &InterpolationCanvas::SetEditConstant;
  set_edit_[1] = &InterpolationCanvas::SetEditLinear;
  set_edit_[2] = &InterpolationCanvas::SetEditBezier;
  set_edit_[3] = &InterpolationCanvas::SetEditSlowOut;
  set_edit_[4] = &InterpolationCanvas::SetEditSlowIn;
  set_edit_[5] = &InterpolationCanvas::SetEditSigmoid;
  set_edit_[6] = &InterpolationCanvas::SetEditCubicBezier;
  set_position(ofVec2f(1500, 50));
  set_color(ofxUIColor(157, 144, 192, 100), ofxUIColor(157, 144, 192, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  canvas_name_ = "interpolation control";
  type_name_[0] = "constant";
  type_name_[1] = "linear";
  type_name_[2] = "bezier";
  type_name_[3] = "slow-out";
  type_name_[4] = "slow-in";
  type_name_[5] = "sigmoid";
  type_name_[6] = "cubic-bezier";
  draw_position_ = ofVec2f(10, 10);
  draw_area_ = ofRectangle(position_.x + draw_position_.x,
                           position_.y + draw_position_.y,
                           500,
                           500);
  valid_ = true;
  gui_ = NULL;
  interpolation_.set_type(1);
  interpolation_.Init();
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_linear_p);
  set_draw(&InterpolationCanvas::DrawLinear);
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::guiSet() {
  if (gui_)
    return;
  gui_ = new ofxUISuperCanvas(canvas_name_);
  gui_->addSpacer();
  gui_->setFontSize(OFX_UI_FONT_LARGE, OFX_UI_FONT_LARGE_SIZE * 3);
  gui_->setFontSize(OFX_UI_FONT_SMALL, OFX_UI_FONT_LARGE_SIZE);  
  gui_->setWidgetFontSize(OFX_UI_FONT_SMALL);
  ofVec2f slider_size(20, 500);
  ofVec2f spacing(20, 20);
  gui_->addSlider("y", 0.0, 1.0, 0.0, slider_size.x, slider_size.y);
  gui_->getWidget("y")
      ->getRect()->setX(draw_position_.x + draw_area_.width + spacing.x);
  gui_->getWidget("y")
      ->getRect()->setY(draw_position_.y);
  gui_->getWidget("y")->setColorBack(valid_widget_color_);
  gui_->addSlider("x", 0.0, 1.0, 0.0, slider_size.y, slider_size.x);
  gui_->getWidget("x")
      ->getRect()->setX(draw_position_.x);
  gui_->getWidget("x")
      ->getRect()->setY(draw_position_.y + draw_area_.height + spacing.y);
  gui_->getWidget("x")->setColorBack(valid_widget_color_);
  ofVec2f button_size(200, 20);
  for (int i = 1; i < 7; ++i) {
    gui_->addLabelButton(type_name_[i], false, button_size.x, button_size.y);
    gui_->getWidget(type_name_[i])
        ->getRect()->setX(draw_position_.x + draw_area_.width + slider_size.x + spacing.x*2);
    gui_->getWidget(type_name_[i])
        ->getRect()->setY(draw_position_.y+(button_size.y+spacing.y)*i);
    gui_->getWidget(type_name_[i])->setColorBack(valid_widget_color_);
  }
  for (int i = 0; i < 3; ++i) {
    gui_->addLabelButton("p"+ofToString(i+1), false);
    gui_->getWidget("p"+ofToString(i+1))
        ->getRect()->setX(draw_position_.x + draw_area_.width + slider_size.x
                          + button_size.x + spacing.x*3);
    gui_->getWidget("p"+ofToString(i+1))
        ->getRect()->setY(draw_position_.y+(button_size.y+spacing.y)*i);
    gui_->getWidget("p"+ofToString(i+1))->setColorBack(valid_widget_color_);
    gui_->getWidget("p"+ofToString(i+1))->setVisible(false);
  }
  gui_->autoSizeToFitWidgets();
  gui_->setPosition(position_.x, position_.y);
  gui_->setColorBack(valid_back_color_);
  ofAddListener(gui_->newGUIEvent, this, &InterpolationCanvas::guiEvent);
}

void InterpolationCanvas::guiEvent(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  for (int i = 0; i < 7; ++i)
    if (name == type_name_[i]) {
      ofxUILabelButton *button = (ofxUILabelButton*) e.widget; 
      if (button->getValue()) {
        interpolation_.set_type(i);
        interpolation_.Init();
        (this->*set_edit_[i])();
        return;
      }
    }
  if ((name == "x") || (name == "y")) {
    if (current_p_on_edit_ == -1)
      return;
    interpolation_.set_p(ofVec2f(((ofxUISlider*)gui_->getWidget("x"))->getValue(),
                                 ((ofxUISlider*)gui_->getWidget("y"))->getValue()),
                         current_p_on_edit_);
    return;
  }
  for (int i = 1; i < 4; ++i)
    if (name == ("p"+ofToString(i))) {
      current_p_on_edit_ = i;
      ((ofxUISlider*)gui_->getWidget("x"))->setValue(interpolation_.get_p(i).x);
      ((ofxUISlider*)gui_->getWidget("y"))->setValue(interpolation_.get_p(i).y);    
      return;
    }
}

void InterpolationCanvas::SetEditConstant() {
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_constant_p);
  set_draw(&InterpolationCanvas::DrawConstant);
  gui_->getWidget("p1")->setVisible(false);
  gui_->getWidget("p2")->setVisible(false);
  gui_->getWidget("p3")->setVisible(false);
  gui_->autoSizeToFitWidgets();
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::SetEditLinear() {
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_linear_p);
  set_draw(&InterpolationCanvas::DrawLinear);
  gui_->getWidget("p1")->setVisible(false);
  gui_->getWidget("p2")->setVisible(false);
  gui_->getWidget("p3")->setVisible(false);
  gui_->autoSizeToFitWidgets();
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::SetEditBezier() {
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_bezier_p);
  set_draw(&InterpolationCanvas::DrawBezier);
  gui_->getWidget("p1")->setVisible(true);
  gui_->getWidget("p2")->setVisible(false);
  gui_->getWidget("p3")->setVisible(false);
  gui_->autoSizeToFitWidgets();
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::SetEditSlowOut() {
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_slowout_p);
  set_draw(&InterpolationCanvas::DrawSlowOut);
  gui_->getWidget("p1")->setVisible(true);
  gui_->getWidget("p2")->setVisible(false);
  gui_->getWidget("p3")->setVisible(false);
  gui_->autoSizeToFitWidgets();
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::SetEditSlowIn() {
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_slowin_p);
  set_draw(&InterpolationCanvas::DrawSlowIn);
  gui_->getWidget("p1")->setVisible(false);
  gui_->getWidget("p2")->setVisible(true);
  gui_->getWidget("p3")->setVisible(false);
  gui_->autoSizeToFitWidgets();
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::SetEditSigmoid() {
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_sigmoid_p);
  set_draw(&InterpolationCanvas::DrawSigmoid);
  gui_->getWidget("p1")->setVisible(false);
  gui_->getWidget("p2")->setVisible(true);
  gui_->getWidget("p3")->setVisible(true);
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::SetEditCubicBezier() {
  interpolation_.set_set_p(&ssb_utils_interpolation::Interpolation::set_cubicbezier_p);
  set_draw(&InterpolationCanvas::DrawCubicBezier);
  gui_->getWidget("p1")->setVisible(true);
  gui_->getWidget("p2")->setVisible(true);
  gui_->getWidget("p3")->setVisible(false);
  gui_->autoSizeToFitWidgets();
  current_p_on_edit_ = -1;
}

void InterpolationCanvas::guiReset() {
  interpolation_.set_type(1);
  interpolation_.Init();
  (this->*set_edit_[1])();
  ((ofxUISlider*)gui_->getWidget("x"))->setValue(0.0);
  ((ofxUISlider*)gui_->getWidget("y"))->setValue(0.0);
}

void InterpolationCanvas::setUIValue(ssb_common_vec::VecInterpolation value) {
  interpolation_.set_interpolation(value);
  (this->*set_edit_[value.type])();
  ((ofxUISlider*)gui_->getWidget("x"))->setValue(0.0);
  ((ofxUISlider*)gui_->getWidget("y"))->setValue(0.0);
}

ssb_common_vec::VecInterpolation InterpolationCanvas::getUIValue() {
  return interpolation_.get_interpolation();
}

void InterpolationCanvas::DrawConstant(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  ofVec2f p0(draw_area_.x, draw_area_.y+draw_area_.height);
  ofVec2f p1(draw_area_.x+draw_area_.width, draw_area_.y+draw_area_.height);
  ofPolyline line;
  line.addVertex(p0);
  line.addVertex(p1);
  line.close();
  line.draw();
}

void InterpolationCanvas::DrawLinear(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  ofVec2f p0(draw_area_.x, draw_area_.y+draw_area_.height);
  ofVec2f p1(draw_area_.x+draw_area_.width, draw_area_.y);
  ofPolyline line;
  line.addVertex(p0);
  line.addVertex(p1);
  line.close();
  line.draw();
}

void InterpolationCanvas::DrawBezier(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  ofVec2f p0(draw_area_.x, draw_area_.y+draw_area_.height);
  ofVec2f p1(draw_area_.x+interpolation_.get_p(1).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(1).y)*draw_area_.height);
  ofVec2f p2(draw_area_.x+draw_area_.width, draw_area_.y);
  ofPolyline line;
  line.quadBezierTo(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y);
  line.draw();
}

void InterpolationCanvas::DrawSlowOut(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  ofVec2f p0(draw_area_.x, draw_area_.y+draw_area_.height);
  ofVec2f p1(draw_area_.x+interpolation_.get_p(1).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(1).y)*draw_area_.height);
  ofVec2f p2(draw_area_.x+interpolation_.get_p(2).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(2).y)*draw_area_.height);
  ofVec2f p3(draw_area_.x+draw_area_.width, draw_area_.y);
  ofPolyline line1, line2;  
  line1.addVertex(p0);
  line1.addVertex(p1);
  line1.close();
  line1.draw();
  line2.quadBezierTo(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
  line2.draw();
}

void InterpolationCanvas::DrawSlowIn(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  ofVec2f p0(draw_area_.x, draw_area_.y+draw_area_.height);
  ofVec2f p1(draw_area_.x+interpolation_.get_p(1).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(1).y)*draw_area_.height);
  ofVec2f p2(draw_area_.x+interpolation_.get_p(2).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(2).y)*draw_area_.height);
  ofVec2f p3(draw_area_.x+draw_area_.width, draw_area_.y);
  ofPolyline line1, line2;
  line1.quadBezierTo(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y);
  line1.draw();
  line2.addVertex(p2);
  line2.addVertex(p3);
  line2.close();
  line2.draw();
}

void InterpolationCanvas::DrawSigmoid(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  ofVec2f p0(draw_area_.x, draw_area_.y+draw_area_.height);
  ofVec2f p1(draw_area_.x+interpolation_.get_p(1).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(1).y)*draw_area_.height);
  ofVec2f p2(draw_area_.x+interpolation_.get_p(2).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(2).y)*draw_area_.height);
  ofVec2f p3(draw_area_.x+interpolation_.get_p(3).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(3).y)*draw_area_.height);
  ofVec2f p4(draw_area_.x+interpolation_.get_p(4).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(4).y)*draw_area_.height);
  ofVec2f p5(draw_area_.x+draw_area_.width, draw_area_.y);
  ofPolyline line1, line2, line3;
  line1.quadBezierTo(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y);
  line1.draw();
  line2.addVertex(p2);
  line2.addVertex(p3);
  line2.close();
  line2.draw();
  line3.quadBezierTo(p3.x, p3.y, p4.x, p4.y, p5.x, p5.y);
  line3.draw();
}

void InterpolationCanvas::DrawCubicBezier(ofLight &light) {
  light.setAmbientColor(ofColor(0, 0, 0));
  light.setDiffuseColor(ofColor(0, 0, 0));
  ofVec2f p0(draw_area_.x, draw_area_.y+draw_area_.height);
  ofVec2f p1(draw_area_.x+interpolation_.get_p(1).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(1).y) *draw_area_.height);
  ofVec2f p2(draw_area_.x+interpolation_.get_p(2).x*draw_area_.width,
             draw_area_.y+(1-interpolation_.get_p(2).y) *draw_area_.height);
  ofVec2f p3(draw_area_.x+draw_area_.width, draw_area_.y);
  ofPolyline line;
  line.addVertex(p0);
  line.bezierTo(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
  line.draw();
}

} // namespace ssb_uitools_canvas
