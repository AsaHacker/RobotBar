#ifndef SSB_EXAMPLES_APPTEST_H_
#define SSB_EXAMPLES_APPTEST_H_

#include "ssb_common_common.h"
#include "ssb_sensors_sound.h"
#include "ssb_sensors_voice.h"
#include "ssb_actuators_dynamixel.h"

namespace app_test {

class AppTest : public ofBaseApp {
 public:
  void setup();
  void update();
  void draw();
  void keyPressed(int key);
  void keyReleased(int key);
  void mouseMoved(int x, int y);
  void mouseDragged(int x, int y, int button);
  void mouseReleased(int x, int y, int button);
  void windowResized(int w, int h);
  void dragEvent(ofDragInfo dragInfo);
  void gotMessage(ofMessage msg);
  ros::NodeHandle nh;
};

boost::shared_ptr<ssb_sensors_sound::Sound> sound;
boost::shared_ptr<ssb_media_event::Voice> voice;
boost::shared_ptr<ssb_actuators_dynamixel::NeckDynamixel> neck;

}

#endif
