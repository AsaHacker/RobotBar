#ifndef SSB_UITOOLS_KEYFRAME_INL_H_
#define SSB_UITOOLS_KEYFRAME_INL_H_

#include "ssb_uitools_keyframe.h"

namespace ssb_uitools_keyframe {

template<typename T>
void KeyFrame<T>::setXmlSettings(ofxXmlSettings &settings, int which_id) {
  settings.addTag("frame");
  settings.pushTag("frame", which_id);
  ssb_utils_xml::Xml<T, string> xmlp;
  ssb_utils_xml::Xml<ssb_common_vec::VecTime, ssb_common_vec::None> xmlt;
  ssb_utils_xml::Xml<ssb_common_vec::VecInterpolation, ssb_common_vec::None> xmli;
  ssb_common_vec::None none;
  xmlp.setXmlSettings(settings, 0, part_data_, imgname_, "img");
  xmlt.setXmlSettings(settings, 0, time_data_, none, "\0");
  xmli.setXmlSettings(settings, 0, interpolation_data_, none, "\0");
  settings.popTag();
}

template<typename T>
void KeyFrame<T>::getXmlSettings(ofxXmlSettings &settings, int which_id) {
  settings.pushTag("frame", which_id);
  ssb_utils_xml::Xml<T, string> xmlp;
  ssb_utils_xml::Xml<ssb_common_vec::VecTime, ssb_common_vec::None> xmlt;
  ssb_utils_xml::Xml<ssb_common_vec::VecInterpolation, ssb_common_vec::None> xmli;
  ssb_common_vec::None none;
  xmlp.getXmlSettings(settings, 0, part_data_, imgname_, "img");
  xmlt.getXmlSettings(settings, 0, time_data_, none, "\0");
  xmli.getXmlSettings(settings, 0, interpolation_data_, none, "\0");
  settings.popTag();
}

//---------------------------------------------------------------

template<typename T>
KeyFrameCanvas<T>::KeyFrameCanvas(ssb_uitools_canvas::Canvas<T> &p,
                                  ssb_uitools_canvas::TimeCanvas &t,
                                  ssb_uitools_canvas::InterpolationCanvas &i)
    : part_canvas(p), time_canvas(t), interpolation_canvas(i) {
  _constructor((T*)0);
}

template<typename T>
void KeyFrameCanvas<T>::_constructor(void*) {
  part_canvas.guiSet();
  time_canvas.guiSet();
  interpolation_canvas.guiSet();
  part_canvas.setValid(false);
  time_canvas.setVisible(false);
  interpolation_canvas.setVisible(false);
}

template<typename T>
void KeyFrameCanvas<T>::_guiOpenNew(void*) {
  part_canvas.guiReset();
  time_canvas.guiReset();
  interpolation_canvas.guiReset();
  part_canvas.setValid(true);
  time_canvas.setVisible(true);
  interpolation_canvas.setVisible(true);
}

template<typename T>
void KeyFrameCanvas<T>::_guiOpenLoad(void*, KeyFrame<T> load_key) {
  part_canvas.guiLoad(load_key.get_part_data());
  time_canvas.guiLoad(load_key.get_time_data());
  interpolation_canvas.guiLoad(load_key.get_interpolation_data());
  part_canvas.setValid(true);
  time_canvas.setVisible(true);
  interpolation_canvas.setVisible(true);
}

template<typename T>
void KeyFrameCanvas<T>::_guiClose(void*) {
  part_canvas.setValid(false);
  time_canvas.setVisible(false);
  interpolation_canvas.setVisible(false);
}

//---------------------------------------------------------------

template<typename T>
KeyFramePlayer<T>::KeyFramePlayer(string tname, std::vector<KeyFrame<T> > &kf, KeyFrameCanvas<T> &kfc, int &fps) : key_list(kf), play_canvas(kfc), frames_per_sec(fps) {
  play_ = false;
}

template<typename T>
void KeyFramePlayer<T>::Setup() {
  if (key_list.size() == 0)
    return;
  sequence_.clear();
  sequence_.resize(key_list.size()*2-1);
  T diff_0; // The default initializer initializes any typename T with zero.
  sequence_.at(0) = ssb_common_vec::VecSequence<T>(
      key_list.at(0).get_time_data().play_time*frames_per_sec, // number_of_frames
      false, // is_interpolation
      0, // interpolation_type
      0, // key_ref
      diff_0);
  for (int i = 1; i < key_list.size(); ++i) {
    T diff = key_list.at(i).get_part_data();
    diff -= key_list.at(i-1).get_part_data();
    sequence_.at(2*i-1) = ssb_common_vec::VecSequence<T>(
        (key_list.at(i).get_time_data().start_time
         - key_list.at(i-1).get_time_data().start_time
         - key_list.at(i-1).get_time_data().play_time)*frames_per_sec,
        true,
        key_list.at(i-1).get_interpolation_data().type,
        i-1,
        diff);
    sequence_.at(2*i) = ssb_common_vec::VecSequence<T>(
        key_list.at(i).get_time_data().play_time*frames_per_sec,
        false,
        0,
        i,
        diff_0);
  }
  send_value_ = key_list.at(0).get_part_data(); // Set to initial state.
  interpolation_function_.set_interpolation(key_list.at(0).get_interpolation_data());
  id_of_current_sequence_ = 0;
  frame_count_from_start_of_current_sequence_ = 0;
}

template<typename T>
void KeyFramePlayer<T>::StartPlay() {
  if (key_list.size() == 0)
    return;
  send_value_ = key_list.at(0).get_part_data(); // Set to initial state.
  play_canvas.part_canvas.setUIValue(send_value_);
  play_canvas.part_canvas.sendUIValue();
  interpolation_function_.set_interpolation(key_list.at(0).get_interpolation_data());
  id_of_current_sequence_ = 0;
  frame_count_from_start_of_current_sequence_ = 0;
  play_ = true;
}

template<typename T>
void KeyFramePlayer<T>::OnPlay() {
  if (!play_)
    return;
  frame_count_from_start_of_current_sequence_++;
  if (frame_count_from_start_of_current_sequence_
      > sequence_.at(id_of_current_sequence_).number_of_frames) {
    id_of_current_sequence_++;
    frame_count_from_start_of_current_sequence_ = 0;
    if (id_of_current_sequence_ > (sequence_.size() - 1)) {
      EndPlay();
      return;
    } else if (sequence_.at(id_of_current_sequence_).is_interpolation == false) {
      send_value_ =
          key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data();
      play_canvas.part_canvas.setUIValue(send_value_);
      play_canvas.part_canvas.sendUIValue();
      // Set the interpolation function for next sequence.
      interpolation_function_.set_interpolation(
          key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_interpolation_data());
      return;
    }
  } else if (sequence_.at(id_of_current_sequence_).interpolation_type > 0) {
    send_value_ = key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data();
    T add_amount = sequence_.at(id_of_current_sequence_).diff;
    add_amount *= interpolation_function_.F(
        static_cast<float>(frame_count_from_start_of_current_sequence_)
        /sequence_.at(id_of_current_sequence_).number_of_frames);
    send_value_ += add_amount;
    play_canvas.part_canvas.setUIValue(send_value_);
    play_canvas.part_canvas.sendUIValue();
  }
}

template<typename T>
void KeyFramePlayer<T>::_Play2Pause(void*) {
  if (key_list.size() == 0)
    return;
  play_ = false;
}

template<typename T>
void KeyFramePlayer<T>::_Pause2Play(void*) {
  if (key_list.size() == 0)
    return;
  play_ = true;
}

template<typename T>
void KeyFramePlayer<T>::EndPlay() {
  if (key_list.size() == 0)
    return;
  send_value_ = key_list.at(0).get_part_data(); // Set to initial state.
  interpolation_function_.set_interpolation(ssb_common_vec::VecInterpolation());
  id_of_current_sequence_ = 0;
  frame_count_from_start_of_current_sequence_ = 0;
  play_ = false;
}

template<typename T>
void KeyFramePlayer<T>::_PlayFrame(void*, int frame_number) {
  if (key_list.size() == 0)
    return;
  int frame_count_to_find_current_sequence = 0;
  for (int i = 0; i < (sequence_.size()-1); ++i) {
    frame_count_to_find_current_sequence += sequence_.at(i).number_of_frames;
    if (frame_number <= frame_count_to_find_current_sequence) {
      id_of_current_sequence_ = i;
      send_value_ = key_list.at(sequence_.at(id_of_current_sequence_).key_ref)
                    .get_part_data();
      frame_count_from_start_of_current_sequence_ =
          frame_number
          - (frame_count_to_find_current_sequence - sequence_.at(i).number_of_frames);
      interpolation_function_.set_interpolation(
          key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_interpolation_data());
      T add_amount = sequence_.at(id_of_current_sequence_).diff;
      add_amount *= interpolation_function_.F(
          static_cast<float>(frame_count_from_start_of_current_sequence_)
          /sequence_.at(id_of_current_sequence_).number_of_frames);
      send_value_ += add_amount;
      play_canvas.part_canvas.setUIValue(send_value_);
      play_canvas.part_canvas.sendUIValue();
      return;
    }
  }
  // If frame_number is out of play range.
  send_value_ = key_list.at(key_list.size()-1).get_part_data();
  play_canvas.part_canvas.setUIValue(send_value_);
  play_canvas.part_canvas.sendUIValue();
}

//---------------------------------------------------------------

template<typename T>
KeyFrameEditor<T>::KeyFrameEditor(string tname, std::vector<KeyFrame<T> > &kf, KeyFrameCanvas<T> &kfc, float &_timespan, int &_timeline_width) : key_list(kf), edit_canvas(kfc), time_span(_timespan), timeline_width(_timeline_width) {
  set_position(ofVec2f(0, 1000));
  set_color(ofxUIColor(139, 172, 210, 100), ofxUIColor(139, 172, 210, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  //time_span_ = 30.0;
  //timeline_width_ = 2500;
  part_name_ = tname;
}

template<typename T>
void KeyFrameEditor<T>::guiSet(int pos_y) {
  gui_ = new ofxUISuperCanvas(part_name_+" editor", position_.x, position_.y + pos_y, 3000, 46);
  gui_button_ = new ofxUIScrollableCanvas(position_.x, position_.y + pos_y + 46, 8000, 80);
  gui_timeline_ = new ofxUIScrollableCanvas(position_.x, position_.y + pos_y + 126, 8000, 30);
  gui_button_->setScrollArea(position_.x, position_.y + pos_y, 3000, 135);
  gui_timeline_->setScrollArea(position_.x, position_.y + pos_y, 3000, 135);
  gui_button_->setScrollableDirections(true, false);
  gui_timeline_->setScrollableDirections(true, false);
  gui_button_->addSpacer();
  gui_timeline_->addSpacer();
  gui_->setFontSize(OFX_UI_FONT_LARGE, OFX_UI_FONT_LARGE_SIZE * 3);
  gui_->setFontSize(OFX_UI_FONT_SMALL, OFX_UI_FONT_LARGE_SIZE);  
  gui_->setWidgetFontSize(OFX_UI_FONT_SMALL);
  gui_->addLabelButton("add " + part_name_ + " frame", static_cast<bool>(false), 200, 0);
  gui_button_->setGlobalButtonDimension(64);
  for (int i = 0; i < key_list.size(); ++i) {
    target_key_ = i; // This is needed in CreateTimeline.
    CreateTimelineButton(key_list.at(target_key_));
  };
  target_key_ = 0;
  gui_button_->setColorBack(valid_back_color_);
  gui_->setColorBack(valid_back_color_);
  gui_->getWidget("add " + part_name_ + " frame")->setColorBack(valid_widget_color_);
  gui_timeline_->setColorBack(valid_back_color_);
  ofAddListener(gui_->newGUIEvent, this, &KeyFrameEditor::OpenEditCanvas);
  ofAddListener(gui_button_->newGUIEvent, this, &KeyFrameEditor::OpenEditCanvas);
  ofAddListener(gui_timeline_->newGUIEvent, this, &KeyFrameEditor::OpenEditCanvas);
}

template<typename T>
void KeyFrameEditor<T>::OpenEditCanvas(ofxUIEventArgs &e) {
  if (e.getName() == ("add " + part_name_ + " frame")) {
    ofxUILabelButton *button = (ofxUILabelButton*) e.widget;
    if (button->getValue()) {
      edit_canvas.guiOpenNew();
      target_key_ = key_list.size();
      return;
    }
  }
  for (int i = 0; i < key_list.size(); ++i) {
    if (e.getName() == key_list.at(i).get_imgname()) {
      ofxUILabelButton *button = (ofxUILabelButton*) e.widget;
      if (button->getValue()) {
        edit_canvas.guiOpenLoad(key_list.at(i));
        target_key_ = i;
        return;
      }
    } // if
  } // for
}

template<typename T>
void KeyFrameEditor<T>::CloseEditCanvas() {
  edit_canvas.time_canvas.ResetTrigger();
  edit_canvas.guiClose();
  if (target_key_ >= key_list.size()) {
    KeyFrame<T> new_key;
    new_key.set_imgname("GUI/images/" + part_name_ + "Frame" + ofToString(target_key_) + ".png");
    edit_canvas.part_canvas.SaveIcon(new_key.get_imgname());
    T part_value = edit_canvas.part_canvas.getUIValue();
    new_key.set_part_data(part_value);
    ssb_common_vec::VecTime time_value = edit_canvas.time_canvas.getUIValue();
    new_key.set_time_data(time_value);
    ssb_common_vec::VecInterpolation interpolation_value
        = edit_canvas.interpolation_canvas.getUIValue();
    new_key.set_interpolation_data(interpolation_value);
    // Button must be created first! Some types revise their values while creating buttons.
    CreateTimelineButton(new_key); 
    key_list.push_back(new_key);
  } else {
    edit_canvas.part_canvas.SaveIcon(key_list.at(target_key_).get_imgname());
    T part_value = edit_canvas.part_canvas.getUIValue();
    key_list.at(target_key_).set_part_data(part_value);
    ssb_common_vec::VecTime time_value = edit_canvas.time_canvas.getUIValue();
    key_list.at(target_key_).set_time_data(time_value);
    ssb_common_vec::VecInterpolation interpolation_value
        = edit_canvas.interpolation_canvas.getUIValue();
    key_list.at(target_key_).set_interpolation_data(interpolation_value);
    EditTimelineButton(key_list.at(target_key_));
  }
  if (RearrangeKeyList()) // sort list and revise duplicates
    for (int i = 0; i < key_list.size(); ++i)
      EditTimelineButton(key_list.at(i));
}

template<typename T>
int KeyFrameEditor<T>::UpdateEditor() {
  //int play_px = pcc.get_value() * timeline_width_ / time_span_;
  //gui_button_->getRect()->setX(play_px);
  if (edit_canvas.time_canvas.isVisible())
    edit_canvas.time_canvas.ScrollTimeline(gui_button_->getRect()->getX());
  if (edit_canvas.time_canvas.CheckTrigger() && edit_canvas.part_canvas.isValid()) {
    CloseEditCanvas();
    return 1;
  }
  return 0;
}
  
template<typename T>
void KeyFrameEditor<T>::LoadXml(ofxXmlSettings &settings) {
  key_list.clear();
  if (settings.getNumTags(part_name_ + "Frames") > 0) {
    settings.pushTag(part_name_ + "Frames");
    for (int i = 0; i < settings.getNumTags("frame"); ++i) {
      KeyFrame<T> load;
      load.getXmlSettings(settings, i);
      key_list.push_back(load);
    }
    settings.popTag();
  }
}

template<typename T>
void KeyFrameEditor<T>::SaveXml(ofxXmlSettings &settings) {
  settings.addTag(part_name_ + "Frames");
  settings.pushTag(part_name_ + "Frames", 0);
  for (int i = 0; i < key_list.size(); ++i)
    key_list.at(i).setXmlSettings(settings, i);
  settings.popTag();
}

template<typename T>
bool KeyFrameEditor<T>::RearrangeKeyList() {
  std::sort(key_list.begin(), key_list.end());
  bool time_revised = false;
  /*
  for (int i = 1; i < key_list.size(); ++i) {
    float end_t = key_list.at(i-1).get_time_data().start_time
        + key_list.at(i-1).get_time_data().play_time;
    if (end_t > key_list.at(i).get_time_data().start_time) {
      key_list.at(i).set_time_data(
          ssb_common_vec::VecTime(end_t,
                                  key_list.at(i).get_time_data().play_time));
      time_revised = true;
    }
  }
  */
  return time_revised;
}

template<typename T>
void KeyFrameEditor<T>::CreateTimelineButton(KeyFrame<T> &key_list_i) {
  float k_sec2px = timeline_width / time_span;
  gui_button_->addImageButton(key_list_i.get_imgname(),
                              key_list_i.get_imgname(),
                              true);
  gui_button_->getWidget(key_list_i.get_imgname())
             ->setColorBack(valid_widget_color_);
  gui_button_->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
  gui_button_->getWidget(key_list_i.get_imgname())
             ->getRect()
             ->setX(position_.x + key_list_i.get_time_data().start_time * k_sec2px);
  gui_timeline_->addSlider(part_name_ + " frame" + ofToString(target_key_), 0,
                           key_list_i.get_time_data().play_time * k_sec2px, 0.0,
                           key_list_i.get_time_data().play_time * k_sec2px, 10);
  gui_timeline_->getWidget(part_name_ + " frame" + ofToString(target_key_))
               ->getRect()
               ->setX(position_.x + key_list_i.get_time_data().start_time * k_sec2px);
  gui_timeline_->getWidget(part_name_ + " frame" + ofToString(target_key_))
               ->getRect()
               ->setY(10);
  gui_timeline_->getWidget(part_name_ + " frame" + ofToString(target_key_))
               ->setColorBack(valid_widget_color_);
  gui_button_->autoSizeToFitWidgets();
  gui_timeline_->autoSizeToFitWidgets();
}

template<typename T>
void KeyFrameEditor<T>::EditTimelineButton(KeyFrame<T> &key_list_i) {
  float k_sec2px = timeline_width / time_span;
  ((ofxUIImageButton*)gui_button_->getWidget(key_list_i.get_imgname()))
             ->setImage(key_list_i.get_imgname());
  gui_button_->getWidget(key_list_i.get_imgname())
             ->getRect()
             ->setX(position_.x + key_list_i.get_time_data().start_time * k_sec2px);
  gui_timeline_->getWidget(part_name_ + " frame" + ofToString(target_key_))
               ->getRect()
               ->setWidth(key_list_i.get_time_data().play_time * k_sec2px);
  gui_timeline_->getWidget(part_name_ + " frame" + ofToString(target_key_))
               ->getRect()
               ->setX(position_.x + key_list_i.get_time_data().start_time * k_sec2px);
  gui_timeline_->getWidget(part_name_ + " frame" + ofToString(target_key_))
               ->getRect()
               ->setY(10);
  gui_button_->autoSizeToFitWidgets();
  gui_timeline_->autoSizeToFitWidgets();
}

//---------------------------------------------------------------

template<typename T_p, typename T_e>
void PlayControllerCanvas::Update(KeyFramePlayer<T_p> &player, KeyFrameEditor<T_e> &editor) {
  switch (play_state_) {
    case ssb_common_enum::REPLAY: {}
    case ssb_common_enum::STARTPLAY: {
      player.StartPlay();
      break;
    }
    case ssb_common_enum::ONPLAY: {
      player.OnPlay();
      break;
    }
    case ssb_common_enum::PLAY2PAUSE: {
      player.Play2Pause();
      break;
    }
    case ssb_common_enum::ONPAUSE: {
      break;
    }
    case ssb_common_enum::PAUSE2PLAY: {
      player.Pause2Play();
      break;
    }
    case ssb_common_enum::STOP: {
      player.EndPlay();
      break;
    }
    case ssb_common_enum::EDIT: {
      player.PlayFrame(((ofxUISlider*)gui_->getWidget("play timeline"))->getValue()
                       * frames_per_sec);
      break;
    }
    default: {
    }
  } // switch
} // Update

//---------------------------------------------------------------

// Events don't have interpolation settings.
// Therefore the interpolation canvas is closed.
// Also event canvases are not visible at default,
// therefore the constructor also differs.
template<typename T>
void KeyFrameCanvas<T>::_constructor(ssb_common_vec::VecEvent*) {
  part_canvas.guiSet();
  time_canvas.guiSet();
  interpolation_canvas.guiSet();
  part_canvas.setValid(false);
  part_canvas.setVisible(false);
  time_canvas.setVisible(false);
}

template<typename T>
void KeyFrameCanvas<T>::_guiOpenNew(ssb_common_vec::VecEvent*) {
  part_canvas.guiReset();
  time_canvas.guiReset();
  interpolation_canvas.guiLoad(ssb_common_vec::VecInterpolation());
  part_canvas.setVisible(true);
  part_canvas.setValid(true);
  time_canvas.setVisible(true);
  interpolation_canvas.setVisible(false);
}

template<typename T>
void KeyFrameCanvas<T>::_guiOpenLoad(ssb_common_vec::VecEvent*, KeyFrame<T> load_key) {
  part_canvas.guiLoad(load_key.get_part_data());
  time_canvas.guiLoad(load_key.get_time_data());
  interpolation_canvas.guiLoad(ssb_common_vec::VecInterpolation());
  part_canvas.setVisible(true);
  part_canvas.setValid(true);
  time_canvas.setVisible(true);
  interpolation_canvas.setVisible(false);
}

template<typename T>
void KeyFrameCanvas<T>::_guiClose(ssb_common_vec::VecEvent*) {
  part_canvas.setValid(false);
  part_canvas.setVisible(false);
  time_canvas.setVisible(false);
  interpolation_canvas.setVisible(false);
}

//---------------------------------------------------------------

// Specialized functions for event.
// Events are not dynamic motions but rather a on/off trigger.
// Therefore the pause function works differently.
template<typename T>
void KeyFramePlayer<T>::_Play2Pause(ssb_common_vec::VecEvent*) {
  if (key_list.size() == 0)
    return;
  play_ = false;
  send_value_ = ssb_common_vec::VecVoice();
  play_canvas.part_canvas.setUIValue(send_value_);
  play_canvas.part_canvas.sendUIValue();
}

template<typename T>
void KeyFramePlayer<T>::_Pause2Play(ssb_common_vec::VecEvent*) {
  if (key_list.size() == 0)
    return;
  play_ = true;
  if (key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data().msg == "quit")
    return;
  send_value_ = ssb_common_vec::VecVoice("continue", 0, -1, "1.0");
  play_canvas.part_canvas.setUIValue(send_value_);
  play_canvas.part_canvas.sendUIValue();
}

// Events will play nothing during the default interpolation.
// The following method allows non-interpolated events to be played at frame level,
// or to be more accurate, as if played at frame level.
template<typename T>
void KeyFramePlayer<T>::_PlayFrame(ssb_common_vec::VecEvent*, int frame_number) {
  if (key_list.size() == 0)
    return;
  int frame_count_to_find_current_sequence = 0;
  for (int i = 0; i < sequence_.size(); ++i) {
    frame_count_to_find_current_sequence += sequence_.at(i).number_of_frames;
    if (frame_number <= frame_count_to_find_current_sequence) {
      id_of_current_sequence_ = i;
      frame_count_from_start_of_current_sequence_ =
          frame_number
          - (frame_count_to_find_current_sequence - sequence_.at(i).number_of_frames);
      if (sequence_.at(id_of_current_sequence_).is_interpolation)
        return;
      send_value_ = key_list.at(sequence_.at(id_of_current_sequence_).key_ref).get_part_data();
      if (send_value_.msg == "quit")
        return;
      ssb_common_vec::VecVoice add_amount = send_value_;
      add_amount /= sequence_.at(i).number_of_frames;
      add_amount += ssb_common_vec::VecTime(add_amount.end_time, 0);      
      // expand play time
      add_amount *= frame_count_from_start_of_current_sequence_;
      // shift start time to current frame but keep end time the same
      add_amount += ssb_common_vec::VecTime(add_amount.end_time-2*add_amount.start_time,
                                            -add_amount.end_time+2*add_amount.start_time);
      // adjust play length of chunk
      add_amount *= ssb_common_vec::VecTime(1.0, 10.0);
      send_value_ = add_amount;
      play_canvas.part_canvas.setUIValue(send_value_);
      play_canvas.part_canvas.sendUIValue();
      return;
    }
  }
  // If frame_number is out of play range or is a non-speech("quit") frame.
  send_value_ = ssb_common_vec::VecVoice();
  play_canvas.part_canvas.setUIValue(send_value_);
  play_canvas.part_canvas.sendUIValue();
}

//---------------------------------------------------------------

// Specialized functions for voice.
// Voices don't have images and the play time
// has to be automatically fixed to the voices' play time.
template<>
void KeyFrameEditor<ssb_common_vec::VecVoice>
::CreateTimelineButton(KeyFrame<ssb_common_vec::VecVoice> &key_list_i) {
  float k_sec2px = timeline_width / time_span;
  if (key_list_i.get_part_data().msg != "quit") {
    const ssb_uitools_canvas::VoiceCanvas &tmp
        = static_cast<const ssb_uitools_canvas::VoiceCanvas&>(edit_canvas.part_canvas);
    key_list_i.set_time_data(
        ssb_common_vec::VecTime(key_list_i.get_time_data().start_time,
                                tmp.get_voice_length(key_list_i.get_part_data().msg)));
  }
  gui_button_->addLabelButton(key_list_i.get_imgname(),
                              static_cast<bool>(true),
                              key_list_i.get_time_data().play_time * k_sec2px);
  gui_button_->getWidget(key_list_i.get_imgname())
             ->setColorBack(valid_widget_color_);
  gui_button_->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
  gui_button_->getWidget(key_list_i.get_imgname())
             ->getRect()
             ->setX(position_.x + key_list_i.get_time_data().start_time * k_sec2px);
  ((ofxUILabelButton*)gui_button_->getWidget(key_list_i.get_imgname()))
      ->setLabelText(key_list_i.get_part_data().msg);
  gui_button_->autoSizeToFitWidgets();
}

template<>
void KeyFrameEditor<ssb_common_vec::VecVoice>
::EditTimelineButton(KeyFrame<ssb_common_vec::VecVoice> &key_list_i) {
  float k_sec2px = timeline_width / time_span;
  if (key_list_i.get_part_data().msg != "quit") {
    const ssb_uitools_canvas::VoiceCanvas &tmp
        = static_cast<const ssb_uitools_canvas::VoiceCanvas&>(edit_canvas.part_canvas);
    key_list_i.set_time_data(
        ssb_common_vec::VecTime(key_list_i.get_time_data().start_time,
                                tmp.get_voice_length(key_list_i.get_part_data().msg)));
  }
  gui_button_->getWidget(key_list_i.get_imgname())
             ->getRect()
             ->setX(position_.x + key_list_i.get_time_data().start_time * k_sec2px);
  gui_button_->getWidget(key_list_i.get_imgname())
             ->getRect()
             ->setWidth(key_list_i.get_time_data().play_time * k_sec2px);
  ((ofxUILabelButton*)gui_button_->getWidget(key_list_i.get_imgname()))
      ->setLabelText(key_list_i.get_part_data().msg);
  gui_button_->autoSizeToFitWidgets();
}

} // namespace ssb_uitools_keyframe

#endif
