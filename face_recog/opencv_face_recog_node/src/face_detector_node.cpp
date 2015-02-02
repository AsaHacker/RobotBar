#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <compressed_image_transport/compressed_subscriber.h>

#include "cv.h"
#include "highgui.h"

static const std::string OPENCV_WINDOW = "Image window";

class FaceDetectorNode
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber compressed_image_sub_ ;
  image_transport::Subscriber image_sub_;
  ros::Publisher  rect_pub, normalized_rect_pub,
    max_rect_pub, normalized_max_rect_pub ;

  std::vector<cv::Rect> faces;
  cv::Rect largest_face;
  std::vector<cv::Rect> largest_faces;
  int largest_faces_cnt;
  cv::CascadeClassifier cascade;
  int window_flag;

public:
  FaceDetectorNode(int wf) : it_(nh_)
  {
    // this->nh_ = nh_;
    this->image_sub_ = it_.subscribe("/face_detector/image/raw", 1,
				     &FaceDetectorNode::imageCb, this);
    this->compressed_image_sub_
      = it_.subscribe("/face_detector/image", 1,
		      &FaceDetectorNode::imageCb, this,
		      image_transport::TransportHints("compressed"));
    this->rect_pub =
      this->nh_.advertise<std_msgs::Float32MultiArray>("/face_detector/rect", 1);
    this->normalized_rect_pub =
      this->nh_.advertise<std_msgs::Float32MultiArray>("/face_detector/normalized_rect", 1);

    this->max_rect_pub =
      this->nh_.advertise<std_msgs::Float32MultiArray>("/face_detector/max_rect", 1);
    this->normalized_max_rect_pub =
      this->nh_.advertise<std_msgs::Float32MultiArray>("/face_detector/normalized_max_rect", 1);

    if(!cascade.load("/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml")) std::cout << "xml invalid" << std::endl ;
    this->window_flag = wf;

    this->largest_face.x = -1;

    this->largest_faces_cnt = 0;
    this->largest_faces.resize(25);

    if ( this->window_flag ) cv::namedWindow(OPENCV_WINDOW);
  }

  ~FaceDetectorNode()
  {
    if ( this->window_flag ) cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
      {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
      }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    // tarImg = cv_ptr->image ;
    // face = cvHaarDetectObjects(tarImg, cvHCC, cvMStr);
    this->cascade.detectMultiScale(cv_ptr->image, this->faces,
				   1.3,2,CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));
    cv::Mat image_mat = cv_ptr->image ;
    std::cout << this->faces.size() << " face detected" << std::endl ;
    //
    // publish all face topic and get max area face
    cv::Rect lface;
    lface.x = -1;
    int max_area = -1;
    for (int i = 0; i < this->faces.size() ; i++) {
      cv::Rect faceRect = this->faces.at(i) ;
      cv::Point left_top_p, right_bot_p ;
      left_top_p.x = faceRect.x ;
      left_top_p.y = faceRect.y ;
      right_bot_p.x = faceRect.x + faceRect.width ;
      right_bot_p.y = faceRect.y + faceRect.height ;
      if ( this->window_flag )
	cv::rectangle(cv_ptr->image, left_top_p, right_bot_p,
		      CV_RGB(0, 255 ,0), 3, CV_AA);
      std_msgs::Float32MultiArray pub_msg ;
      std::vector<float> rect(4) ;
      rect[0] = faceRect.x; rect[1] = faceRect.y;
      rect[2] = faceRect.width; rect[3] = faceRect.height;
      pub_msg.data = rect ;
      this->rect_pub.publish(pub_msg) ;

      if ( max_area < faceRect.width * faceRect.height ){
	max_area = faceRect.width * faceRect.height;
	lface = faceRect;
      }

      rect[0] /= image_mat.cols ;
      rect[2] /= image_mat.cols ;
      rect[1] /= image_mat.rows ;
      rect[3] /= image_mat.rows ;
      pub_msg.data = rect ;
      this->normalized_rect_pub.publish(pub_msg) ;
    }

    if ( this->largest_face.x < 0 ){
      this->largest_face = lface;
    }

    //
    // publish max area face tracker
    //if ( this->largest_faces_cnt > this->largest_faces.size() - 2 ){
    this->largest_faces.at(this->largest_faces_cnt) = lface;
    this->largest_faces_cnt = (this->largest_faces_cnt+1) % this->largest_faces.size();
    //
    cv::Rect rect_buf;
    int cnt_buf=0;
    rect_buf.x=0; rect_buf.y=0; rect_buf.width=0; rect_buf.height=0;
    for ( int i=0 ; i<this->largest_faces.size() ; i++ ){
      cv::Rect rect = this->largest_faces.at(i);
      if ( rect.x < 0 || abs(rect.width * rect.height - this->largest_face.width * this->largest_face.height) > 100 || abs(rect.x - this->largest_face.x) > 100 || abs(rect.y - this->largest_face.y) > 100 ){
      } else {
	rect_buf.x += rect.x; rect_buf.width += rect.width;
	rect_buf.y += rect.y; rect_buf.height += rect.height;
	cnt_buf++;
      }
    }
    //
    double blend_rate = 0.5;
    if ( cnt_buf == 0 ){
      std::cout << this->faces.size() << " face missing" << std::endl ;
      this->largest_face.x = -1;
    } else {
      this->largest_face.x =
	blend_rate * this->largest_face.x + blend_rate/cnt_buf *rect_buf.x;
      this->largest_face.y =
	blend_rate * this->largest_face.y + blend_rate/cnt_buf *rect_buf.y;
      this->largest_face.width =
	blend_rate * this->largest_face.width + blend_rate/cnt_buf *rect_buf.width;
      this->largest_face.height =
	blend_rate * this->largest_face.height + blend_rate/cnt_buf *rect_buf.height;
    }
    //

    if ( this->largest_face.x >= 0 ){
      cv::Rect faceRect = this->largest_face;
      cv::Point left_top_p, right_bot_p ;
      left_top_p.x = faceRect.x ;
      left_top_p.y = faceRect.y ;
      right_bot_p.x = faceRect.x + faceRect.width ;
      right_bot_p.y = faceRect.y + faceRect.height ;
      if ( this->window_flag )
	cv::rectangle(cv_ptr->image, left_top_p, right_bot_p,
		      CV_RGB(255, 0 ,0), 3, CV_AA);
      std_msgs::Float32MultiArray pub_msg ;
      std::vector<float> rect(4) ;
      rect[0] = faceRect.x; rect[1] = faceRect.y;
      rect[2] = faceRect.width; rect[3] = faceRect.height;
      pub_msg.data = rect ;
      this->max_rect_pub.publish(pub_msg) ;

      rect[0] /= image_mat.cols ;
      rect[2] /= image_mat.cols ;
      rect[1] /= image_mat.rows ;
      rect[3] /= image_mat.rows ;
      pub_msg.data = rect ;
      this->normalized_max_rect_pub.publish(pub_msg) ;
    }

    // Update GUI Window
    if ( this->window_flag ) {
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      cv::waitKey(3);
    }

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  int wf = 1;
  if ( const char* wfs = std::getenv("FACE_DETECT_NODE_WINDOW_FLAG") ){
    wf = atoi(wfs);
  }
  FaceDetectorNode ic(wf);
  ros::spin();
  return 0;
}
