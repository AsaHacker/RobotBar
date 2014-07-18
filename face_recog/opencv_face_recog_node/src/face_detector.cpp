#include "cv.h"
#include "highgui.h"

int main(int argc, char* argv[]) {

  IplImage* tarImg;
  char *tarFilePath = "face.jpg";
  if ( argc > 1 ){
    tarFilePath = argv[1] ;
  }

  tarImg = cvLoadImage(tarFilePath, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);

  CvHaarClassifierCascade* cvHCC
    = (CvHaarClassifierCascade*)cvLoad("/opt/ros/hydro/share/OpenCV/haarcascades/haarcascade_frontalface_default.xml");

  CvMemStorage* cvMStr = cvCreateMemStorage(0);
  CvSeq* face;
  face = cvHaarDetectObjects(tarImg, cvHCC, cvMStr);

  for (int i = 0; i < face->total; i++) {
    CvRect* faceRect = (CvRect*)cvGetSeqElem(face, i);

    cvRectangle(tarImg,
		cvPoint(faceRect->x, faceRect->y),
		cvPoint(faceRect->x + faceRect->width, faceRect->y + faceRect->height),
		CV_RGB(255, 0 ,0),
		3, CV_AA);
  }

  cvNamedWindow("face_detect");
  cvShowImage("face_detect", tarImg);

  cvWaitKey(0);
  cvDestroyWindow("face_detect");
  cvReleaseMemStorage(&cvMStr);
  cvReleaseHaarClassifierCascade(&cvHCC);
  cvReleaseImage(&tarImg);

  return 0;
}
