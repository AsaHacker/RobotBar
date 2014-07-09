#include "demo_maker.h"

namespace demo_maker {

void DemoMaker::setup() {
  ofBackground(255, 255, 255);
  ofSetFrameRate(30);
  light.enable();
  ofSetCircleResolution(120);
  _fps_ = 30;
  _time_span_ = 30.0;
  _timeline_width_ = 2500;
  ros::NodeHandle n("~");
  nh = n;
  count = 0;
  // Initialization.
  neck_canvas = boost::shared_ptr<cvs::NeckCanvas<mod::NeckModel> >(
      new cvs::NeckCanvas<mod::NeckModel>(nh));
  eye_canvas = boost::shared_ptr<cvs::EyeCanvas<mod::QuadOEyeModel> >(
      new cvs::EyeCanvas<mod::QuadOEyeModel>(nh));
  tentacle_canvas = boost::shared_ptr<cvs::TentacleCanvas<mod::TentacleModel> >(
      new cvs::TentacleCanvas<mod::TentacleModel>(nh));
  voice_canvas = boost::shared_ptr<cvs::VoiceCanvas>(new cvs::VoiceCanvas(nh));
  time_canvas = boost::shared_ptr<cvs::TimeCanvas>(
      new cvs::TimeCanvas(_time_span_, _timeline_width_));
  interpolation_canvas = boost::shared_ptr<cvs::InterpolationCanvas>(
      new cvs::InterpolationCanvas());

  neck_keycanvas = boost::shared_ptr<key::KeyFrameCanvas<vec::VecNeck> >(
      new key::KeyFrameCanvas<vec::VecNeck>(*neck_canvas, *time_canvas,
                                            *interpolation_canvas));
  eye_keycanvas = boost::shared_ptr<key::KeyFrameCanvas<vec::VecEye> >(
      new key::KeyFrameCanvas<vec::VecEye>(*eye_canvas, *time_canvas,
                                           *interpolation_canvas));
  tentacle_keycanvas = boost::shared_ptr<key::KeyFrameCanvas<vec::VecTentacle> >(
      new key::KeyFrameCanvas<vec::VecTentacle>(*tentacle_canvas, *time_canvas,
                                                *interpolation_canvas));
  voice_keycanvas = boost::shared_ptr<key::KeyFrameCanvas<vec::VecVoice> >(
      new key::KeyFrameCanvas<vec::VecVoice>(*voice_canvas, *time_canvas,
                                             *interpolation_canvas));

  voice_editor = boost::shared_ptr<key::KeyFrameEditor<vec::VecVoice> >(
      new key::KeyFrameEditor<vec::VecVoice>("voice", voice_key, *voice_keycanvas,
                                             _time_span_, _timeline_width_));
  neck_editor = boost::shared_ptr<key::KeyFrameEditor<vec::VecNeck> >(
      new key::KeyFrameEditor<vec::VecNeck>("neck", neck_key, *neck_keycanvas,
                                            _time_span_, _timeline_width_));
  eye_editor = boost::shared_ptr<key::KeyFrameEditor<vec::VecEye> >(
      new key::KeyFrameEditor<vec::VecEye>("eye", eye_key, *eye_keycanvas,
                                           _time_span_, _timeline_width_));
  tentacle_editor = boost::shared_ptr<key::KeyFrameEditor<vec::VecTentacle> >(
      new key::KeyFrameEditor<vec::VecTentacle>("tentacle", tentacle_key, *tentacle_keycanvas,
                                                _time_span_, _timeline_width_));
  
  voice_player = boost::shared_ptr<key::KeyFramePlayer<vec::VecVoice> >(
      new key::KeyFramePlayer<vec::VecVoice>("voice", voice_key, *voice_keycanvas, _fps_));
  neck_player = boost::shared_ptr<key::KeyFramePlayer<vec::VecNeck> >(
      new key::KeyFramePlayer<vec::VecNeck>("neck", neck_key, *neck_keycanvas, _fps_));
  eye_player = boost::shared_ptr<key::KeyFramePlayer<vec::VecEye> >(
      new key::KeyFramePlayer<vec::VecEye>("eye", eye_key, *eye_keycanvas, _fps_));
  tentacle_player = boost::shared_ptr<key::KeyFramePlayer<vec::VecTentacle> >(
      new key::KeyFramePlayer<vec::VecTentacle>("tentacle", tentacle_key,
                                                *tentacle_keycanvas, _fps_));
  
  play_controller = boost::shared_ptr<key::PlayControllerCanvas>(
      new key::PlayControllerCanvas(_fps_, _time_span_, _timeline_width_));
  // Load XML file.
  ofxXmlSettings settings;
  if (settings.loadFile("xml/new.xml")) {
    settings.pushTag("scenario");
    voice_editor->LoadXml(settings);
    eye_editor->LoadXml(settings);
    neck_editor->LoadXml(settings);
    tentacle_editor->LoadXml(settings);
    settings.popTag();
  }
  // Set gui and position of editor GUIs.
  voice_editor->guiSet(0);
  eye_editor->guiSet(160);
  neck_editor->guiSet(320);
  tentacle_editor->guiSet(480);
  // Reset gui scroll area
  voice_editor->SetScrollAreaHeight(0, 640);
  eye_editor->SetScrollAreaHeight(0, 640);
  neck_editor->SetScrollAreaHeight(0, 640);
  tentacle_editor->SetScrollAreaHeight(0, 640);
  // Set play data for player.
  voice_player->Setup();
  eye_player->Setup();
  neck_player->Setup();
  tentacle_player->Setup();
  // Set play_controller GUI.
  play_controller->guiSet();
}

void DemoMaker::update() {
  // Check play controller state.
  if (play_controller->PushUpdate() == ssb_common_enum::STARTPLAY) {
    count = 0;
  }
  play_controller->Update(*neck_player, *neck_editor);
  play_controller->Update(*voice_player, *voice_editor);
  play_controller->Update(*eye_player, *eye_editor);
  play_controller->Update(*tentacle_player, *tentacle_editor);
  if (play_controller->PopUpdate() == ssb_common_enum::REPLAY) {
    count++;
    cout << count << endl;
  }
  // Save play data if changes occurred.
  int save = 0;
  save += neck_editor->UpdateEditor();
  save += voice_editor->UpdateEditor();
  save += eye_editor->UpdateEditor();
  save += tentacle_editor->UpdateEditor();
  if (save > 0) {
    ofxXmlSettings settings;
    settings.addTag("scenario");
    settings.pushTag("scenario",0);
    settings.addValue("name", "hoge");
    voice_editor->SaveXml(settings);
    eye_editor->SaveXml(settings);
    neck_editor->SaveXml(settings);
    tentacle_editor->SaveXml(settings);
    settings.popTag();
    settings.save("xml/new.xml");
    if (settings.loadFile("xml/new.xml")) {
      settings.pushTag("scenario");
      voice_editor->LoadXml(settings);
      eye_editor->LoadXml(settings);
      neck_editor->LoadXml(settings);
      tentacle_editor->LoadXml(settings);
      settings.popTag();
    }
    voice_player->Setup();
    eye_player->Setup();
    neck_player->Setup();
    tentacle_player->Setup();
  }
  // Ros spin.
  ros::spinOnce();
}

void DemoMaker::draw() {
  // Draw settings.
  ofPushStyle();
  ofEnableBlendMode(OF_BLENDMODE_ALPHA);
  ofPopStyle();
  ofSetRectMode(OF_RECTMODE_CENTER);
  // Draw capture icons.
  neck_keycanvas->part_canvas.CreateIcon(light);
  eye_keycanvas->part_canvas.CreateIcon(light);
  tentacle_keycanvas->part_canvas.CreateIcon(light);
  // Draw Interpolation
  interpolation_canvas->Draw(light);
}

void DemoMaker::exit() {
  // Delete and free canvases.
  neck_canvas->guiEnd();
  eye_canvas->guiEnd();
  tentacle_canvas->guiEnd();
  time_canvas->guiEnd();
  interpolation_canvas->guiEnd();
  voice_canvas->guiEnd();
  neck_editor->guiEnd();
  play_controller->guiEnd();
}

void DemoMaker::keyPressed(int key) {}
void DemoMaker::keyReleased(int key) {};
void DemoMaker::mousePressed(int x, int y, int button) {};
void DemoMaker::mouseMoved(int x, int y) {};
void DemoMaker::mouseDragged(int x, int y, int button) {};
void DemoMaker::mouseReleased(int x, int y, int button) {};
void DemoMaker::windowResized(int w, int h) {};
void DemoMaker::dragEvent(ofDragInfo dragInfo) {};
void DemoMaker::gotMessage(ofMessage msg) {};

} // namespace demo_maker

int main(int argc, char **argv) {
  ros::init(argc,argv,"ssb");
  ofSetupOpenGL(8480,7680,OF_WINDOW);
  ofRunApp(new demo_maker::DemoMaker());
}
