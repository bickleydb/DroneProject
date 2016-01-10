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
#define PAPER_AR 8.5/11.0
#define BOX_AR 22.5/44.5

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


void showImage(Mat img) {
  for(;;) {
    imshow("Stuff",img);
    char c = waitKey(1);
    if(c == 27) {break;}
  }
  imwrite("stuff.png",img);
}

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
    } while(points.size() != 4 && offset < 20);
    if(points.size() == 4){return points;}
  }
  
}

double getPerim(std::vector<KeyPoint> paper) {
  assert(paper.size() == 4);
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

std::vector<std::vector<Point> > getPaperShapedContours(std::vector<std::vector<Point> > contours, double ar) {
  std::vector<std::vector<Point> > rtn;
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> cur =  contours[i];
    approxPolyDP(cur,cur,10,true);
    if(cur.size() >= 4) {
      Rect rectSize = boundingRect(cur);
      double aspectRatio = ((double)rectSize.width)/rectSize.height;
      if(abs(abs(aspectRatio)-abs(ar)) < 0.01) {
	rtn.push_back(cur);
      }
    }
  }
  
  return rtn;
}

std::vector<Point> pickBiggestPaper(std::vector<std::vector<Point> > possibles) {
  double bestArea = -1;
  std::vector<Point> bestVect;
  for(int i = 0; i < possibles.size(); i++) {
    double area = contourArea(possibles[i]);
    if(area > bestArea) {
      bestArea = area;
      bestVect = possibles[i];
    }
  }
  return bestVect;
}

double getWidth(std::vector<Point> paper) {
  if(paper.size() != 4) {
    return -1;
  }
   double dist1 = std::sqrt(std::pow(paper[0].x - paper[1].x,2) + std::pow(paper[0].y - paper[1].y,2));
  double dist2 = std::sqrt(std::pow(paper[0].x - paper[2].x,2) + std::pow(paper[0].y - paper[2].y,2));
  double dist3 = std::sqrt(std::pow(paper[0].x - paper[3].x,2) + std::pow(paper[0].y - paper[3].y,2));
  double width = std::min(dist1, std::min(dist2,dist3));
  return width;
}

std::vector<Point> getPaperContour(Mat& img) {
  GaussianBlur(img,img,Size(7,7),0,0);
  cvtColor(img,img,CV_BGR2GRAY);
  Canny(img,img,50,150);
  std::vector<std::vector<Point> > contours;
  findContours(img,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  contours = getPaperShapedContours(contours,PAPER_AR);
  std::vector<Point> paper = pickBiggestPaper(contours);
  return paper;
}

std::pair<double,double> getRotatedRectVals(RotatedRect r) {
  Point2f paper[4];
  r.points(paper);
  double dist1 = std::sqrt(std::pow(paper[0].x - paper[1].x,2) + std::pow(paper[0].y - paper[1].y,2));
  double dist2 = std::sqrt(std::pow(paper[0].x - paper[2].x,2) + std::pow(paper[0].y - paper[2].y,2));
  double dist3 = std::sqrt(std::pow(paper[0].x - paper[3].x,2) + std::pow(paper[0].y - paper[3].y,2));
  double width = std::min(dist1, std::min(dist2,dist3));
  double height = dist1+dist2+dist3-width-std::max(std::max(dist1,dist2),dist3);
  return std::pair<double,double> (width,height);


}

std::vector<std::vector<Point> > getBoxyContours(std::vector<std::vector<Point> > conts) {
  std::vector<std::vector<Point> > rtn;
  for(int i = 0; i < conts.size(); i++) {
    std::vector<Point> approx;
    approxPolyDP(conts[i],approx,10,true);
    if(approx.size() >= 10 && approx.size() <= 22) {
      RotatedRect rect = minAreaRect(approx);
      std::pair<double,double> rectInfo = getRotatedRectVals(rect);
      double ar = rectInfo.first/rectInfo.second;
      double ar2 = rectInfo.second/rectInfo.first;
      if(abs(abs(BOX_AR)-abs(ar)) < 1 || abs(abs(BOX_AR)-abs(ar2)) < 1   ) {
	rtn.push_back(approx);
      }
    }
  }
  return rtn;
}





void getBoxContour(Mat& img) {
  GaussianBlur(img,img,Size(7,7),0,0);
  cvtColor(img,img,CV_BGR2GRAY);
  Canny(img,img,2,15);
  std::vector<std::vector<Point> > contours;
  findContours(img,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  contours = getBoxyContours(contours);
  drawContours(img,contours,-1,Scalar(255),30);
  showImage(img);




}

int main() {
    Mat frame = imread("17foot_RightBox.png");
    getBoxContour(frame);
    /*std::vector<Point> paper = getPaperContour(frame);
    double widthPixels =  getWidth(paper);
    double distance = calculateDistance(widthPixels,8.5);
    std::cout << distance << std::endl;*/

}
