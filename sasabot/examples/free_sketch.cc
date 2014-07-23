#include "free_sketch.h"

void ssb_sensors_map::PointCloud::Loop() {
}

void ssb_sensors_map::Image::Loop() {
  if (memory_.data.size() > 0) {
    ofxCv::toOf(img_->image, free_sketch::img);
    free_sketch::img.update();
  }
}

namespace free_sketch {

void FreeSketch::setup() {
  ros::NodeHandle n("~");
  nh = n;
  pointcloud = boost::shared_ptr<ssb_sensors_map::PointCloud>(new ssb_sensors_map::PointCloud(nh));
  image = boost::shared_ptr<ssb_sensors_map::Image>(new ssb_sensors_map::Image(nh));
  fabmap = boost::shared_ptr<ssb_sensors_map::Fabmap>(new ssb_sensors_map::Fabmap(nh));
  fabmap->test();
}

void FreeSketch::update() {
  pointcloud->Loop();
  image->Loop();
  ros::spinOnce();
}

void FreeSketch::draw() {
  //img.draw(0, 0);
}

void FreeSketch::keyPressed(int key) {}
void FreeSketch::keyReleased(int key) {};
void FreeSketch::mouseMoved(int x, int y) {};
void FreeSketch::mouseDragged(int x, int y, int button) {};
void FreeSketch::mouseReleased(int x, int y, int button) {};
void FreeSketch::windowResized(int w, int h) {};
void FreeSketch::dragEvent(ofDragInfo dragInfo) {};
void FreeSketch::gotMessage(ofMessage msg) {};

} // namespace free_sketch

int main(int argc, char **argv) {
  ros::init(argc,argv,"ssb");
  ofSetupOpenGL(1024,768,OF_WINDOW);
  ofRunApp(new free_sketch::FreeSketch());
}

