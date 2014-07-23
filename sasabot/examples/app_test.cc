#include "app_test.h"

void ssb_sensors_sound::Sound::Loop() {
  if (memory_.texts.size() > 0) {
    app_test::voice->publish_voice("MSG_hello.wav");
    app_test::neck->publish(ssb_common_vec::VecNeck(0.5, 0.0, 0.0));
    memory_.texts.clear();
  }
}

namespace app_test {

void AppTest::setup() {
  ros::NodeHandle n("~");
  nh = n;
  sound = boost::shared_ptr<ssb_sensors_sound::Sound>(new ssb_sensors_sound::Sound(nh));
  voice = boost::shared_ptr<ssb_media_event::Voice>(new ssb_media_event::Voice(nh));
  neck = boost::shared_ptr<ssb_actuators_dynamixel::NeckDynamixel>(new ssb_actuators_dynamixel::NeckDynamixel(nh));
}

void AppTest::update() {
  sound->Loop();
  ros::spinOnce();
}

void AppTest::draw() {}
void AppTest::keyPressed(int key) {}
void AppTest::keyReleased(int key) {};
void AppTest::mouseMoved(int x, int y) {};
void AppTest::mouseDragged(int x, int y, int button) {};
void AppTest::mouseReleased(int x, int y, int button) {};
void AppTest::windowResized(int w, int h) {};
void AppTest::dragEvent(ofDragInfo dragInfo) {};
void AppTest::gotMessage(ofMessage msg) {};

} // namespace app_test

int main(int argc, char **argv) {
  ros::init(argc,argv,"ssb");
  ofSetupOpenGL(1024,768,OF_WINDOW);
  ofRunApp(new app_test::AppTest());
}

