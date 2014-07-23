#include "ssb_uitools_keyframe.h"

namespace ssb_uitools_keyframe {

PlayControllerCanvas::PlayControllerCanvas(int &fps, float &_timespan, int &_timeline_width) : frames_per_sec(fps), time_span(_timespan), timeline_width(_timeline_width) {
  set_position(ofVec2f(0, 880));
  set_color(ofxUIColor(139, 172, 210, 100), ofxUIColor(139, 172, 210, 100),
            ofxUIColor(0, 0, 0, 100), ofxUIColor(0, 0, 0, 100));
  set_playfilename("default");
  play_state_ = ssb_common_enum::NONE;
}

void PlayControllerCanvas::guiSet() {
  gui_ = new ofxUISuperCanvas("play");
  gui_->addImageButton("play", "GUI/images/start_button.png", true, 256, 64);
  gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
  gui_->addImageButton("pause", "GUI/images/pause_button.png", true, 256, 64);
  gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
  gui_->addImageButton("stop", "GUI/images/stop_button.png", true, 256, 64);
  gui_->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
  gui_->addSlider("play timeline", 0, time_span, 0.0, timeline_width, 10);
  gui_->autoSizeToFitWidgets();
  gui_->setPosition(position_.x, position_.y);
  gui_->setColorBack(valid_back_color_);
  gui_->getWidget("play timeline")->setColorBack(valid_widget_color_);
  ofAddListener(gui_->newGUIEvent, this, &PlayControllerCanvas::guiEvent);
}

void PlayControllerCanvas::guiEvent(ofxUIEventArgs &e) {
  string name = e.getName();
  int kind = e.getKind();
  if (name == "play") {
    ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
    if (button->getValue())
      play_state_ = ssb_common_enum::STARTPLAY;
    return;
  }
  if (name == "pause") {
    ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
    if (button->getValue()) {
      if (play_state_ == ssb_common_enum::ONPAUSE
          || play_state_ == ssb_common_enum::NONE)
        play_state_ = ssb_common_enum::PAUSE2PLAY;
      else if (play_state_ == ssb_common_enum::ONPLAY)
        play_state_ = ssb_common_enum::PLAY2PAUSE;
    } // if (button->getValue())
    return;
  } // if (name == "pause")
  if (name == "stop") {
    ofxUILabelButton *button = (ofxUILabelButton *) e.widget;
    if (button->getValue())
      play_state_ = ssb_common_enum::STOP;
    return;
  }
  if (name == "play timeline") {
    play_state_ = ssb_common_enum::EDIT;
    return;
  }
}

ssb_common_enum::PlayState PlayControllerCanvas::PushUpdate() {
  switch (play_state_) {
    case ssb_common_enum::REPLAY: {}
    case ssb_common_enum::STARTPLAY: {
      played_frames_from_start_ = 0;
      ((ofxUISlider*)gui_->getWidget("play timeline"))->setValue(0);
      break;
    }
    case ssb_common_enum::ONPLAY: {
      played_frames_from_start_++;
      break;
    }
    case ssb_common_enum::STOP: {
      ((ofxUISlider*)gui_->getWidget("play timeline"))->setValue(0);
      break;
    }
    default: {
    }
  }
  return play_state_;
}

ssb_common_enum::PlayState PlayControllerCanvas::PopUpdate() {
  switch (play_state_) {
    case ssb_common_enum::REPLAY: {}
    case ssb_common_enum::STARTPLAY:{
      play_state_ = ssb_common_enum::ONPLAY;
      break;
    }
    case ssb_common_enum::ONPLAY: {
      play_state_ = ssb_common_enum::ONPLAY;
      float play_time = static_cast<float>(played_frames_from_start_) / frames_per_sec;
      ((ofxUISlider*)gui_->getWidget("play timeline"))->setValue(play_time);
      if (((ofxUISlider*)gui_->getWidget("play timeline"))->getValue() == time_span)
        play_state_ = ssb_common_enum::REPLAY;
      break;
    }
    case ssb_common_enum::PLAY2PAUSE: {
      play_state_ = ssb_common_enum::ONPAUSE;
      break;
    }
    case ssb_common_enum::ONPAUSE: {
      play_state_ = ssb_common_enum::ONPAUSE;
      break;
    }
    case ssb_common_enum::PAUSE2PLAY: {
      play_state_ = ssb_common_enum::ONPLAY;
      break;
    }
    case ssb_common_enum::STOP: {
      play_state_ = ssb_common_enum::NONE;
      played_frames_from_start_ = 0;
      break;
    }
    case ssb_common_enum::EDIT: {
      play_state_ = ssb_common_enum::NONE;
      played_frames_from_start_ =
          ((ofxUISlider*)gui_->getWidget("play timeline"))->getValue() * frames_per_sec;
      break;
    }
    default: {
    }
  }
  return play_state_;
}

} // namespace ssb_uitools_keyframe
