#include <string>
#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv_modules.hpp"
#include <opencv2/features2d/features2d.hpp>

using namespace cv;
using namespace std;

Mat preProcess(Mat const frame) {
  Mat operateOn;
  cvtColor(frame,operateOn,CV_BGR2GRAY);
  GaussianBlur(operateOn,operateOn,Size(7,7),0,0);
  Canny(operateOn,operateOn,10,20);
  return operateOn;
}



Mat getContours(Mat& frame) {
   preProcess(frame);
   std::vector<std::vector<Point> > contours;
   findContours(frame,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
   drawContours(frame,contours,-1,Scalar(1000),20);
   bitwise_not(frame,frame);
   return frame;
}

Mat removeNonContours(Mat& frame) {
  Mat rtn;
  Mat colors[3];
  split(frame,colors);
  Mat mask = getContours(frame);
  for(int i = 0; i < 3; i++) {
    frame.copyTo(colors[i],mask);
  }
  merge(colors,3,rtn);
  return rtn;
}

std::vector<std::vector<Point> > getRectangularish(std::vector<std::vector<Point> > contours) {
  std::vector<std::vector<Point> > rtn;
  for(int i = 0; i < contours.size(); i++) {
    if(contours[i].size() >= 20) {
      rtn.push_back(contours[i]);
    }
  }
  return rtn;
}


void showImage(std::string name, Mat& frame) {
  for(;;) {
    char(c) = waitKey(1000);
    imshow("Output",frame);
    if(c == 27) {
      break;
    } 
  }
  imwrite("Test.bmp",frame);
}


std::vector<KeyPoint> getKeyPoints(Mat const img)   {
  Mat operate = preProcess(img);
  std::vector<KeyPoint> rtn;
  FAST(operate,rtn,5);
  return rtn;
}

std::vector<Mat> hsvSplit (Mat const img) {
  std::vector<Mat> rtn;
  Mat cpy;
  Mat hsvSplit[3];
  cvtColor(img,cpy,CV_BGR2HSV);
  split(cpy,hsvSplit);
  rtn.push_back(hsvSplit[0]);
  rtn.push_back(hsvSplit[1]);
  rtn.push_back(hsvSplit[2]);
  return rtn;
}



std::vector<KeyPoint> filter (Mat const img, Scalar const Color, std::vector<KeyPoint> const pts) {
  std::vector<KeyPoint> rtn;
  std::vector<Mat> hsv = hsvSplit(img);
  Mat h = hsv[0];
  Mat s = hsv[1];
  Mat v = hsv[2];

  for(int i = 0; i < pts.size(); i++) {
    Point keyPoint = pts[i].pt;
    if((h.at<unsigned char>(keyPoint) > 0 && h.at<unsigned char>(keyPoint) < 50) || v.at<unsigned char>(keyPoint) < 50) {
      //      if( v.at<unsigned char>(keyPoint) > 50) {
	rtn.push_back(pts[i]);
	//         }
    }
  }
  return rtn;
}


int main() {
 Mat img = imread("17foot_LeftBox.png");
 Mat other = preProcess(img);
 std::vector<KeyPoint>  pts;
 FAST(other,pts,30);
 // cvtColor(img,img,CV_GRAY2BGR);
 drawKeypoints(img,pts,img);
 resize(img,1000,1000);
 showImage("",img);


}



