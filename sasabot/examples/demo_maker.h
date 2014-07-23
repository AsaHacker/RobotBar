#ifndef SSB_EXAMPLES_DEMOMAKER_H_
#define SSB_EXAMPLES_DEMOMAKER_H_

#include "ssb_common_common.h"
#include "ssb_utils_xml-inl.h"
#include "ssb_uitools_canvas-inl.h"
#include "ssb_uitools_keyframe-inl.h"

namespace demo_maker {

class DemoMaker : public ofBaseApp {
 public:
  void setup();
  void update();
  void draw();
  void exit();
  void keyPressed(int key);
  void keyReleased(int key);
  void mousePressed(int x, int y, int button);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);
  ros::NodeHandle nh;
  ofLight light;
  int _fps_;
  float _time_span_;
  int _timeline_width_;
  int count;
};

namespace vec = ssb_common_vec;
namespace cvs = ssb_uitools_canvas;
namespace xml = ssb_utils_xml;
namespace mod = ssb_utils_model;
namespace key = ssb_uitools_keyframe;

vector<key::KeyFrame<vec::VecNeck> > neck_key;
vector<key::KeyFrame<vec::VecVoice> > voice_key;
vector<key::KeyFrame<vec::VecEye> > eye_key;
vector<key::KeyFrame<vec::VecTentacle> > tentacle_key;
boost::shared_ptr<cvs::NeckCanvas<mod::NeckModel> > neck_canvas;
boost::shared_ptr<cvs::EyeCanvas<mod::QuadOEyeModel> > eye_canvas;
boost::shared_ptr<cvs::TentacleCanvas<mod::TentacleModel> > tentacle_canvas;
boost::shared_ptr<cvs::VoiceCanvas> voice_canvas;
boost::shared_ptr<cvs::TimeCanvas> time_canvas;
boost::shared_ptr<cvs::InterpolationCanvas> interpolation_canvas;
boost::shared_ptr<key::KeyFrameCanvas<vec::VecNeck> > neck_keycanvas;
boost::shared_ptr<key::KeyFrameCanvas<vec::VecEye> > eye_keycanvas;
boost::shared_ptr<key::KeyFrameCanvas<vec::VecTentacle> > tentacle_keycanvas;
boost::shared_ptr<key::KeyFrameCanvas<vec::VecVoice> > voice_keycanvas;
boost::shared_ptr<key::KeyFrameEditor<vec::VecVoice> > voice_editor;
boost::shared_ptr<key::KeyFrameEditor<vec::VecNeck> > neck_editor;
boost::shared_ptr<key::KeyFrameEditor<vec::VecEye> > eye_editor;
boost::shared_ptr<key::KeyFrameEditor<vec::VecTentacle> > tentacle_editor;
boost::shared_ptr<key::KeyFramePlayer<vec::VecNeck> > neck_player;
boost::shared_ptr<key::KeyFramePlayer<vec::VecVoice> > voice_player;
boost::shared_ptr<key::KeyFramePlayer<vec::VecEye> > eye_player;
boost::shared_ptr<key::KeyFramePlayer<vec::VecTentacle> > tentacle_player;
boost::shared_ptr<key::PlayControllerCanvas> play_controller;

} // namespace demo_maker

#endif
