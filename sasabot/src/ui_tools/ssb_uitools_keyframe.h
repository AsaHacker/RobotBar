#ifndef SSB_UITOOLS_KEYFRAME_H_
#define SSB_UITOOLS_KEYFRAME_H_

#include "ssb_common_common.h"
#include "ssb_utils_xml.h"
#include "ssb_utils_interpolation.h"
#include "ssb_uitools_canvas.h"

// Code description on keyframe:
// KeyFrameCanvas/KeyFramePlayer behave differently with VecEvent types.
// Therefore, these two classes use a specialization trick to
// specify vector classes derived from VecEvent.
// In this way, there should be no need for creating new specializations
// when adding a new class derived from VecEvent.
// On the other hand, the KeyFrameEditor usually behaves differently
// among events.
// For example, a voice's play time or video should be automatically
// edited with its given play time from the wav file.
// But, a projection's play time has no given time nor has an end.
// Therefore, the template is specified among classes.

// What's typical with VecEvent derived classes?
//  1. VecEvent classes do not interpolate. It is an on and off switch.
//  2. VecEvent classes pause by command, not by stopping calculation.

// Usage warning on keyframe:
// In general, KeyFrame types should be specified with the type of event,
// if the template type is a derived class of VecEvent.
//   OK:  KeyFrame<VecVoice> voice_key_frame;
//   BAD: KeyFrame<VecEvent> voice_key_frame;
// This is because file loading and saving classes (e.g. xml classes) depend on
// specific types. Also, defualt constructors(zero amount) can differ between events.


namespace ssb_uitools_keyframe {

template<typename T> class KeyFrame {
 public:
  KeyFrame() {};
  KeyFrame(T p, ssb_common_vec::VecTime t, ssb_common_vec::VecInterpolation i) {
    part_data_ = p;
    time_data_ = t;
    interpolation_data_ = i;
  };
  ~KeyFrame() {};
  bool operator<(const KeyFrame &kf) const {
    return (time_data_.start_time < kf.time_data_.start_time);
  }
  void set_imgname(string imgname) { imgname_ = imgname; };
  void set_part_data(T part_data) { part_data_ = part_data; };
  void set_time_data(ssb_common_vec::VecTime time_data) { time_data_ = time_data; };
  void set_interpolation_data(ssb_common_vec::VecInterpolation interpolation_data) {
    interpolation_data_ = interpolation_data;
  };
  string get_imgname() { return imgname_; };
  T get_part_data() { return part_data_; };
  ssb_common_vec::VecTime get_time_data() { return time_data_; };
  ssb_common_vec::VecInterpolation get_interpolation_data() { return interpolation_data_; };
  void setXmlSettings(ofxXmlSettings &settings, int which_id);
  void getXmlSettings(ofxXmlSettings &settings, int which_id);
 private:
  string imgname_;
  T part_data_;
  ssb_common_vec::VecTime time_data_;
  ssb_common_vec::VecInterpolation interpolation_data_;
};

template<typename T> class KeyFrameCanvas {
 public:
  KeyFrameCanvas(ssb_uitools_canvas::Canvas<T> &p,
                 ssb_uitools_canvas::TimeCanvas &t,
                 ssb_uitools_canvas::InterpolationCanvas &i);
  ~KeyFrameCanvas() {};
  void guiOpenNew() { _guiOpenNew((T*)0); };
  void guiOpenLoad(KeyFrame<T> load_key) { _guiOpenLoad((T*)0, load_key); };
  void guiClose() { _guiClose((T*)0); };
  ssb_uitools_canvas::Canvas<T> &part_canvas;
  ssb_uitools_canvas::TimeCanvas &time_canvas;
  ssb_uitools_canvas::InterpolationCanvas &interpolation_canvas;
 private:
  void _constructor(void*);
  void _constructor(ssb_common_vec::VecEvent*);
  void _guiOpenNew(void*);
  void _guiOpenNew(ssb_common_vec::VecEvent*);
  void _guiOpenLoad(void*, KeyFrame<T> load_key);
  void _guiOpenLoad(ssb_common_vec::VecEvent*, KeyFrame<T> load_key);
  void _guiClose(void*);
  void _guiClose(ssb_common_vec::VecEvent*);
};

template<typename T> class KeyFramePlayer {
 public:
  KeyFramePlayer(string tname, std::vector<KeyFrame<T> > &kf, KeyFrameCanvas<T> &kfc,
                 int &fps);
  ~KeyFramePlayer() {};
  void Setup();
  void StartPlay();
  void OnPlay();
  inline void Play2Pause() { _Play2Pause((T*)0); };
  inline void Pause2Play() { _Pause2Play((T*)0); };
  void EndPlay();
  void PlayFrame(int frame_number) { _PlayFrame((T*)0, frame_number); };
 private:
  std::vector<ssb_common_vec::VecSequence<T> > sequence_;
  int frame_count_from_start_of_current_sequence_;
  int id_of_current_sequence_;
  ssb_utils_interpolation::Interpolation interpolation_function_;
  T send_value_;
  bool play_;
  int &frames_per_sec;
  // Save target addresses in order to keep structures similar to KeyFrameEditor.
  std::vector<KeyFrame<T> > &key_list;
  KeyFrameCanvas<T> &play_canvas;
  void _Play2Pause(void*);
  void _Play2Pause(ssb_common_vec::VecEvent*);
  void _Pause2Play(void*);
  void _Pause2Play(ssb_common_vec::VecEvent*);
  void _PlayFrame(void*, int frame_number);
  void _PlayFrame(ssb_common_vec::VecEvent*, int frame_number);
};

template<typename T> class KeyFrameEditor {
 public:
  KeyFrameEditor(string tname, std::vector<KeyFrame<T> > &kf, KeyFrameCanvas<T> &kfc,
                 float &_timespan, int &_timeline_width);
  ~KeyFrameEditor() {};
  void guiSet(int pos_y);
  void guiEnd() {
    delete gui_;
    delete gui_button_;
    delete gui_timeline_;
  };
  inline void set_position(ofVec2f pos) { position_ = pos; };
  inline void set_color(ofxUIColor valid_back_color, ofxUIColor valid_widget_color,
                       ofxUIColor not_valid_back_color, ofxUIColor not_valid_widget_color) {
    valid_back_color_ = valid_back_color;
    valid_widget_color_ = valid_widget_color;
    not_valid_back_color_ = not_valid_back_color;
    not_valid_widget_color_ = not_valid_widget_color;
  };
  void OpenEditCanvas(ofxUIEventArgs &e);
  void CloseEditCanvas();
  int UpdateEditor();
  void LoadXml(ofxXmlSettings &settings);
  void SaveXml(ofxXmlSettings &settings);
  void SetScrollAreaHeight(int top, int bottom) {
    gui_button_->setScrollArea(position_.x, 1000+top, 3000, 1000+bottom);
    gui_timeline_->setScrollArea(position_.x, 1000+top, 3000, 1000+bottom);
  };
 protected:
  ofVec2f position_;
  ofxUIColor valid_back_color_, valid_widget_color_;
  ofxUIColor not_valid_back_color_, not_valid_widget_color_;
  ofxUISuperCanvas *gui_;
  ofxUIScrollableCanvas *gui_button_,*gui_timeline_;
  int target_key_;
  string part_name_;
  // Save target addresses so that it is usable in OpenEditCanvas (= guiEvent).
  // Note that guiEvent can only take 1 argument ofxUIEventArgs &e.
  std::vector<KeyFrame<T> > &key_list;
  KeyFrameCanvas<T> &edit_canvas;
  float &time_span;
  int &timeline_width;
  bool RearrangeKeyList();
  void CreateTimelineButton(KeyFrame<T> &set_data_i);
  void EditTimelineButton(KeyFrame<T> &set_data_i);
};

class PlayControllerCanvas {
 public:
  PlayControllerCanvas(int &fps, float &_timespan, int &_timelinewidth);
  ~PlayControllerCanvas() {};
  void guiSet();
  void guiEvent(ofxUIEventArgs &e);
  void guiEnd() { delete gui_; };
  inline void set_position(ofVec2f pos) { position_ = pos; };
  inline void set_color(ofxUIColor valid_back_color, ofxUIColor valid_widget_color,
                       ofxUIColor not_valid_back_color, ofxUIColor not_valid_widget_color) {
    valid_back_color_ = valid_back_color;
    valid_widget_color_ = valid_widget_color;
    not_valid_back_color_ = not_valid_back_color;
    not_valid_widget_color_ = not_valid_widget_color;
  };
  ssb_common_enum::PlayState PushUpdate();
  template<typename T_p, typename T_e>
  void Update(KeyFramePlayer<T_p> &player, KeyFrameEditor<T_e> &editor);
  ssb_common_enum::PlayState PopUpdate();
  //inline void set_time_span(float sec) { time_span_ = sec; };
  //inline void set_timeline_width(int px) { timeline_width_ = px; };
  inline void set_playfilename(string filename) { playfilename_ = filename; };
  inline string get_playfilename(string filename) { return playfilename_; };
 private:
  ofVec2f position_;
  ofxUIColor valid_back_color_, valid_widget_color_;
  ofxUIColor not_valid_back_color_, not_valid_widget_color_;
  ofxUISuperCanvas *gui_;
  string playfilename_;
  ssb_common_enum::PlayState play_state_;
  int &frames_per_sec;
  float &time_span;
  int &timeline_width;
  int played_frames_from_start_;
};

} // namespace ssb_uitools_keyframe

#endif
