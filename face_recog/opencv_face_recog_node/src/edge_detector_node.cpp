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

class EdgeDetectorNode
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber compressed_image_sub_ ;
  image_transport::Subscriber image_sub_;
  ros::Publisher  rect_pub, normalized_rect_pub ;

  std::vector<cv::Rect> faces;
  // cv::CascadeClassifier cascade;

public:
  EdgeDetectorNode() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/edge_detector/image/raw", 1,
			       &EdgeDetectorNode::imageCb, this);
    compressed_image_sub_ = it_.subscribe("/edge_detector/image", 1,
					  &EdgeDetectorNode::imageCb, this,
					  image_transport::TransportHints("compressed")) ;
    //rect_pub = nh_.advertise<std_msgs::Float32MultiArray>("/face_detector/rect", 1);
    //normalized_rect_pub = nh_.advertise<std_msgs::Float32MultiArray>("/face_detector/normalized_rect", 1);

    // if(!cascade.load("/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml")) std::cout << "xml invalid" << std::endl ;

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~EdgeDetectorNode()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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
    //cascade.detectMultiScale(cv_ptr->image, faces,
    //1.3,2,CV_HAAR_SCALE_IMAGE, cv::Size(50, 50));
    cv::Mat image_mat = cv_ptr->image ;
    //
    cv::Mat grayImage;
    cv::cvtColor(image_mat, grayImage, CV_BGR2GRAY);
    //
    cv::Mat binaryImage;
    const double threshold = 100.0;
    const double maxValue = 255.0;
    cv::threshold(grayImage, binaryImage, threshold, maxValue, cv::THRESH_BINARY);
    //
    std::vector< std::vector<cv::Point> > contours;
    cv::findContours(binaryImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    //
    cv::Mat contourImage(binaryImage.size(), CV_8U, cv::Scalar(255));
    const int drawAllContours = -1;
    cv::drawContours(contourImage, contours, drawAllContours, cv::Scalar(0));
    //
    // std::cout << faces.size() << " face detected" << std::endl ;
    // for (int i = 0; i < faces.size() ; i++) {
    //   cv::Rect faceRect = faces.at(i) ;
    //   cv::Point left_top_p, right_bot_p ;
    //   left_top_p.x = faceRect.x ;
    //   left_top_p.y = faceRect.y ;
    //   right_bot_p.x = faceRect.x + faceRect.width ;
    //   right_bot_p.y = faceRect.y + faceRect.height ;
    //   cv::rectangle(cv_ptr->image, left_top_p, right_bot_p,
    // 		    CV_RGB(255, 0 ,0), 3, CV_AA);
    //   std_msgs::Float32MultiArray pub_msg ;
    //   std::vector<float> rect(4) ;
    //   rect[0] = faceRect.x; rect[1] = faceRect.y;
    //   rect[2] = faceRect.width; rect[3] = faceRect.height;
    //   pub_msg.data = rect ;
    //   rect_pub.publish(pub_msg) ;

    //   rect[0] /= image_mat.cols ;
    //   rect[2] /= image_mat.cols ;
    //   rect[1] /= image_mat.rows ;
    //   rect[3] /= image_mat.rows ;
    //   pub_msg.data = rect ;
    //   normalized_rect_pub.publish(pub_msg) ;
    // }


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, contourImage) ;//cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  EdgeDetectorNode ic;
  ros::spin();
  return 0;
}
