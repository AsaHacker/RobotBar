#ifndef SSB_EXAMPLES_FREESKETCH_H_
#define SSB_EXAMPLES_FREESKETCH_H_

#include "ssb_common_common.h"
#include "ssb_sensors_map.h"
#include "ofxCv.h"

namespace free_sketch {

class FreeSketch : public ofBaseApp {
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

ofImage img;
boost::shared_ptr<ssb_sensors_map::PointCloud> pointcloud;
boost::shared_ptr<ssb_sensors_map::Image> image;
boost::shared_ptr<ssb_sensors_map::Fabmap> fabmap;

}

#endif
