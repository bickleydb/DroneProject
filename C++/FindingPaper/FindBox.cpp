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
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <chrono>
#include <random>
#include <algorithm>

#define LOW_RED  0
#define HIGH_RED  5
#define FOCAL_LENGTH 216.79
#define FM 667.88
#define FB 257.00
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286 

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void showImage(Mat img);

std::vector<KeyPoint> generateNeighbors(KeyPoint pt, unsigned int rows, unsigned int cols,double offset) {
  



}

std::vector<KeyPoint> findPaper (Mat frame, double avgBright) {
  Mat colors[3];
  Mat colored;
  Mat copy;
  
  frame.copyTo(colored);
  cvtColor(colored,colored,CV_BGR2HSV);
  split(frame,colors);
  copy = frame;
   
  cvtColor(frame,frame,CV_BGR2GRAY);
  GaussianBlur(frame,frame,Size(3,3),0);

   
  for(int parameter = 10; parameter > 0; parameter--) {
   
    double offset = 1;
    std:vector<KeyPoint> points;
    std::vector<KeyPoint> allPoints;
    FAST(frame,allPoints,parameter);

    do {      
      points = std::vector<KeyPoint>(allPoints);
      std::vector<int> keep(points.size());
      for (std::vector<KeyPoint>::iterator it = points.begin(); it != points.end(); it++) {
	KeyPoint pt = *it;
	Vec3b at = colored.at<Vec3b>(pt.pt);
	int numRed = 0;
	int numWhite = 0;
	 double dr[8] = { 0, -1,-1, -1, 0,  1, 1, 1};
	 double dc[8] = { 1, 1,  0, -1,-1,-1, 0, 1};
	 std::vector<int> deltaRow (dr, dr+sizeof(dr)/sizeof(int));
	 std::vector<int> deltaCol (dc, dc+sizeof(dc)/sizeof(int));
  	  
	 for(unsigned int i = 0; i < deltaRow.size(); i++) {
	   KeyPoint delta = pt;
	   int rowAdd = offset;
	   int colAdd = offset;
	  
	   delta.pt.x = pt.pt.x + (int) (deltaRow[i] * rowAdd);
	   delta.pt.y = pt.pt.y + (int) (deltaCol[i] * colAdd);
	   if(delta.pt.x < 0 || delta.pt.y < 0 || delta.pt.x >= frame.cols || delta.pt.y >= frame.rows) {
	     continue;
	   }
	   int colorCode = 0;
	  Vec3b neighbor = colored.at<Vec3b>(delta.pt);
	  int count = 0;
	  for(int i = 0; i < 3; i++) {
	    if((int)neighbor[i] == 255){count++;} 
	  }
	  
	  
	  if(count >= 2){
	    continue;
	  }
	  
	  if((neighbor[0] > LOW_RED || neighbor[0] < HIGH_RED) && neighbor[1] > 73 && neighbor[2] > 50) {
	    numRed++;
	    colorCode = 1;
	  } else if( neighbor[2] > avgBright) {
	    numWhite++;
	    colorCode = 2;
	  }
	  
	  if(colorCode == 0) {
	    colored.at<Vec3b>(delta.pt)[0]= 3;
	    colored.at<Vec3b>(delta.pt)[1] = 100;
	    colored.at<Vec3b>(delta.pt)[2] = 50;
	  } else if (colorCode == 1) {
	    colored.at<Vec3b>(delta.pt)[0]= 100;
	    colored.at<Vec3b>(delta.pt)[1] = 255;
	    colored.at<Vec3b>(delta.pt)[2] = 255;
	  } else if (colorCode == 2) {
	    colored.at<Vec3b>(delta.pt)[0]= 10;
	    colored.at<Vec3b>(delta.pt)[1] = 255;
	    colored.at<Vec3b>(delta.pt)[2] = 255;
	    
	  }
	}  
	if(numRed != 5 ||( numWhite != 3 )) {
	  keep[it - points.begin()] = -1;
	}
      }
      
      int size = points.size();
      int trueIndex = 0;
      int loopIndex = 0;
      
      while(trueIndex < size) {
	if(keep[trueIndex] == -1) {
	  points.erase(points.begin()+loopIndex);
	  
	  trueIndex++;
	} else {
	  loopIndex++;
	  trueIndex++;
	}
      }
      offset++;
    } while(points.size() % 4 != 0 && offset < 20);
    return points;
  }
  
}

double getPerim(std::vector<KeyPoint> paper) {

  double dist1 = std::sqrt(std::pow(paper[0].pt.x - paper[1].pt.x,2) + std::pow(paper[0].pt.y - paper[1].pt.y,2));
  double dist2 = std::sqrt(std::pow(paper[0].pt.x - paper[2].pt.x,2) + std::pow(paper[0].pt.y - paper[2].pt.y,2));
  double dist3 = std::sqrt(std::pow(paper[0].pt.x - paper[3].pt.x,2) + std::pow(paper[0].pt.y - paper[3].pt.y,2));
  double width = std::min(dist1, std::min(dist2,dist3));
  double height = dist1 + dist2 + dist3 - width - std::max(dist1,std::max(dist2,dist3));
  //double widthRatio = width/8.5;
  //double heightRatio = height/11;
  return width;
}

double calculateDistance(double width, double inchWidth) {
  double distance = 0;
  distance = FOCAL_LENGTH;
  distance = distance * inchWidth;
  distance = distance/width;
  return distance;
}


std::vector<KeyPoint> findPaper(Mat& frame) {
  Mat hsv[3];
  cvtColor(frame,frame,CV_BGR2HSV);
    split(frame,hsv);
    cvtColor(frame,frame,CV_HSV2BGR);

    double avgBright = 0;
    double stdDevBright = 0;
    double avgSat = 0;
    double stdDevSat = 0;
    Scalar avg;
    Scalar stdDev;
    meanStdDev(hsv[2],avg,stdDev);
    avgBright = avg[0];
    stdDevBright = stdDev[0];
    meanStdDev(hsv[1],avg,stdDev);
    avgSat = avg[0];
    stdDevSat = stdDev[0];
    std::vector<KeyPoint> paper = findPaper(frame,avgBright);
    return paper;
}


Point getLaserLocation(Mat& frame) {
  // cvtColor(frame,frame,CV_GRAY2BGR);
  //showImage(frame);
  for(int i = 0; i < frame.rows; i++) {
    for(int t = 0; t < frame.cols; t++) {
        if(frame.at<unsigned char>(i,t) != 0) {	 
	  showImage(frame);
          std:cout << i << "," << t << std::endl;
	  //ellipse(frame, Point(i,t),Size(2,2),0,0,360,Scalar(100),20);
	//showImage(frame);
	  return Point(i,t);
	} else {
	  frame.at<Vec3b>(i,t)[0] = 255;
	  frame.at<Vec3b>(i,t)[1] = 255;
	  frame.at<Vec3b>(i,t)[2] = 0;
      }
    }
  }
} 


void showImage(Mat img) {
  for(;;) {
    imshow("Stuff",img);
    char c = waitKey(1);
    if(c == 27) {break;}
  }
  imwrite("stuff.png",img);
}

Mat getDiff (Mat first, Mat second) {
  cvtColor(first,first,CV_BGR2GRAY);
    cvtColor(second,second,CV_BGR2GRAY);
    GaussianBlur(first,first,Size(3,3),0);
    GaussianBlur(second,second,Size(3,3),0);

  //Rect roi = Rect(first.rows/2-10,0,20,first.rows-1);
  //first = first(roi);
 
  Mat rtn = abs(first-second);
  double maxVal = 0;
  double minVal = 0;
  // minMaxLoc(rtn,&minVal,&maxVal);
  //threshold(rtn,rtn,maxVal-10,255,0);

  //Rect roi = Rect(first.rows/2+300,0,100,first.rows-1);
  //rtn = rtn(roi);
  return rtn;
}


double calcDistance(double yCoor) {
 
  double z = FB;
  double denom = yCoor - FM;
  z = z / denom;

  return z;

 
}

Point2f determineLaserPoint(std::vector<std::vector<Point> > pts) {
  Point2f bestPt;
  double bestRad = 1000;
  for(int i = 0; i < pts.size(); i++) {
    std::vector<Point> cur = pts[i];
    Point2f curPoint;
    float curRad;
    minEnclosingCircle(cur,curPoint,curRad);
    if(curRad < bestRad) {
      bestPt = curPoint;
      bestRad = curRad;
    }
  }
  return bestPt;
}

std::vector<std::vector<Point> > findCircleContours (Mat& image) {
  std::vector<std::vector<Point> > contours;
  Canny(image,image,50,150);
  std::vector<std::vector<Point> > approxConts;
  std::vector<std::vector<Point> > circles;
  findContours(image,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> cont = contours[i];
    double perimeter = arcLength(cont,true);
    Point2f center;
    float rad = 0;
    std::vector<Point> approx;
    approxPolyDP(cont,approx,1,true);
    minEnclosingCircle(approx,center,rad);
    double contArea = contourArea(approx,false);

    if(PI*rad*rad <= 1.5*contArea) {
      circles.push_back(approx);
    }
  }
  return circles;
}





int main() {
    Mat laser = imread("15Feet.png");
    Mat noLaser = imread("15FeetLaser.png");
    Mat diff = getDiff(laser,noLaser);
    //std::cout << determineLaserPoint(findCircleContours(diff)) << std::endl;
    Point pt = determineLaserPoint(findCircleContours(diff));
    std::cout << "DIST " << calcDistance(pt.y) << std::endl;
    //    std::vector<KeyPoint> paper = findPaper(laser);
    //double widthPixels =  getPerim(paper);
    //double distance = calculateDistance(widthPixels,8.6);
    //drawKeypoints(laser,paper,laser);*/

  return 0;
}
