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
#include <chrono>
#include <random>
#include <algorithm>
#include <thread>

#define LOW_RED  0
#define HIGH_RED  5
#define FOCAL_LENGTH 216.79
#define FM 667.88
#define FB 257.00
#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286 
#define BOX_AR 44.5/22.5

using namespace std;
using namespace cv;


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

void showMovie(Mat& img) {
  imshow("Movie",img);
}

void showImage(Mat img) {
  for(;;) {
    imshow("Stuff",img);
    char c = waitKey(1);
      if(c == 27) {break;}
  }
   imwrite("AlgoOutput.png",img);
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
  std::cout << z << std::endl;
  double denom = (yCoor - FM);
  std::cout << denom << std::endl;
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

Mat valThresholding(Mat val) {
  Mat cpy;
  threshold(val,cpy,90,255,3);
  //showImage(cpy);
  threshold(cpy,cpy,150,255,4);
  //showImage(cpy);
  threshold(cpy,cpy,1,255,0);
  //showImage(cpy);
  return cpy;
}

Mat hueThresholding(Mat hue) {
  Mat cpy;
  threshold(hue,cpy,100,255,4);
  showImage(cpy);
  threshold(cpy,cpy,1,255,0);
  showImage(cpy);

  return cpy;
}

Mat satThresholding(Mat sat) {
  Mat cpy;
  threshold(sat,cpy,55,255,3);
  threshold(cpy,cpy,150,255,4);
  threshold(cpy,cpy,1,255,0);
  return cpy;
}

Mat filterOutNonBox(Mat box) {
  GaussianBlur(box,box,Size(7,7),0);
  Mat hsvImg;
  cvtColor(box,hsvImg,CV_BGR2HSV);
  Mat hsv[3];
  split(hsvImg,hsv);
  Mat maskH = hueThresholding(hsv[0]);
  Mat maskV = valThresholding(hsv[2]);
  Mat maskS = satThresholding(hsv[1]);
  Mat mask;
  bitwise_and(maskH,maskV,mask);
  bitwise_and(mask,maskS,mask);
  //showImage(mask);
  Mat rtn;
  hsv[2].copyTo(rtn,mask);
  // showImage(rtn);
  return rtn;
}

std::vector<std::vector<Point> > filterBasedOnArea(std::vector<std::vector<Point> > contours) {
  std::vector<std::vector<Point> > rtn;
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> approx;
    approxPolyDP(contours[i],approx,5,true);
    Rect test = boundingRect(approx);
    if(test.area() > 10000) {
      rtn.push_back(approx);
    }
  }
  return rtn;
}

std::vector<Rect> determineBoxes(std::vector<std::vector<Point>> contours) {
  std::vector<Rect> rtn;
  for(int i = 0; i < contours.size(); i++) {
    Rect possible = boundingRect(contours[i]);
    int width = possible.width;
    int height = possible.height;
    double larger = (double) std::max(width,height);
    double smaller = (double) std::min(width,height);
    if(larger/smaller < BOX_AR + 0.5 && larger/smaller > BOX_AR - 0.5 && possible.area() > 10000) {
      rtn.push_back(possible);
    }
  }
  return rtn;
}

void removeRedNotHighest(Mat& img) {
  for(int i = 0; i < img.rows; i++) {
    for(int t = 0; t < img.cols; t++) {
      Vec3b pix = img.at<Vec3b>(i,t);
      Scalar mean,dev;
      meanStdDev(pix,mean,dev);
      if(pix[2] <= mean[0] + dev[0] ) {
        img.at<Vec3b>(i,t) = Vec3b(0,0,0);
      }

    }

  }

}



void removeKeyPoints(Mat& img, std::vector<KeyPoint>& pts) {
  std::vector<KeyPoint> replace;
  for(int i = 0; i < pts.size(); i++) {
    int rowPt = pts[i].pt.x;
    int colPt = pts[i].pt.y;
    Vec3b val =  img.at<Vec3b>(rowPt,colPt);
    if((val[0] == 0 && val[1] == 0 && val[2] == 0) ) {
      replace.push_back(pts[i]);
    }
  }
  pts = replace;
} 

void removeHSV(Mat& other, std::vector<KeyPoint>& pts) {
  Mat img;
  Mat test[3];
  other.copyTo(img);
  showImage(img);
  cvtColor(img,img,CV_BGR2HSV);
  split(img,test);
  equalizeHist(test[0],test[0]);
  equalizeHist(test[1],test[1]);
  equalizeHist(test[2],test[2]);
  merge(test,3,img);
  std::vector<KeyPoint> keep;
  for(int i = 0; i < pts.size(); i++) {
     int rowPt = pts[i].pt.x;
     int colPt = pts[i].pt.y;
     Vec3b val =  img.at<Vec3b>(rowPt,colPt);
     if((val[0] > 10) || val[2] != 0) {
     } else {
       std::cout << val << std::endl;
       keep.push_back(pts[i]);
     }
  }
  std::cout << keep.size() << std::endl;
  pts = keep;
}


/*std::vector<KeyPoint> findBox (Mat& img, std::vector<KeyPoint> pts) {
  for (std::vector<KeyPoint>::iterator it = points.begin(); it != points.end(); it++) {
    KeyPoint pt = *it;
    Vec3b at = colored.at<Vec3b>(pt.pt);
    int numBlack; = 0;
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
      Vec3b neighbor = img.at<Vec3b>(delta.pt);
      
      if((neighbor[0] <= 10 && neighbor[1] <= 10 && neighbor[2] <= 10) {
	  numBlack++;
	colorCode = 1;
      } else if( neighbor[2] > avgBright) {
	numWhite++;
	colorCode = 2;
      }
  
	if(numBlack > 3 ) {
	  keep[it - points.begin()] = -1;
    }
  }
  }*/


void threadDoEveryOther(std::vector<Mat> imgVec) {
  for(int q = 0; q < imgVec.size(); q++) {
    Mat img = imgVec[q];
      for(int t = 0; t < img.cols; t++) {
	Vec3b at = img.at<Vec3b>(0,t);
	if(std::abs(at[2]-at[1]) < 20) {
	  Vec3b newVec(0,0,0);
	  img.at<Vec3b>(0,t) = newVec;
	} else if(at[0] > at[1] && at[0] > at[2]) {
	  Vec3b newVec(0,0,0);
	  img.at<Vec3b>(0,t) = newVec;
	} else if(at[1] > at[0] && at[1] > at[2]) {
	  Vec3b newVec(0,0,0);
	  img.at<Vec3b>(0,t) = newVec;
	} else if(at[2] > at[0] && at[2] > at[1]) {
	  Vec3b newVec(0,0,255);
	  img.at<Vec3b>(0,t) = newVec;
	} else {
	  Vec3b newVec(0,0,0);
	  img.at<Vec3b>(0,t) = newVec;
	}
    } 
  }
}

Mat changeToMax (Mat& in) {
  Mat img;
  GaussianBlur(in,img,Size(7,7),0);
  std::vector<Mat> evens;
  std::vector<Mat> odds;
  for(int i = 0; i < img.rows; i++) {
    if(i&2 == 0) {evens.push_back(img.row(i));
    } else {odds.push_back(img.row(i));}
  }
  std::thread first(threadDoEveryOther,evens);
  std::thread second(threadDoEveryOther,odds);
  first.join();
  second.join();
  return img;
}

Mat makeMask (Mat& in) {
  Mat bgr[3];
  split(in,bgr);
  return bgr[2];
}

Mat createHorizontalDelta(Mat& in) {
  Mat output;
  in.copyTo(output);
  for(int i = 0; i < in.rows-1; i++) {
    for(int t = 0; t < in.cols; t++) {
      Vec3b cur = in.at<Vec3b>(i,t);
      Vec3b next = in.at<Vec3b>(i+1,t);
      Vec3b delta(std::abs(cur[0]-next[0]),std::abs(cur[1]-next[1]),std::abs(cur[2] - next[2]));
      output.at<Vec3b>(i,t) = delta;
    }
  }
  return output;
}

Mat createHorizontalDelta1(Mat& in) {
  Mat output;
  in.copyTo(output);
  for(int i = 0; i < in.rows-1; i++) {
    for(int t = 0; t < in.cols; t++) {
      unsigned char cur = in.at<unsigned char>(i,t);
      unsigned char next = in.at<unsigned char>(i+1,t);
      unsigned char delta(std::abs(cur-next));
      output.at<unsigned char>(i,t) = delta;
    }
  }
  return output;
}


std::vector<Rect> findBoxes(Mat& in) {
  Mat preservedColor;
  Mat redMask, mask,maskedBox,gray;
  Mat bgr[3];
  std::vector<std::vector<Point> > contours;
  in.copyTo(preservedColor);
  redMask = changeToMax(in);
  showImage(redMask);
  mask = makeMask(redMask);
  in.copyTo(maskedBox,mask);
  showImage(maskedBox);
  split(maskedBox,bgr);
  equalizeHist(bgr[0],bgr[0]);
  equalizeHist(bgr[1],bgr[1]);
  equalizeHist(bgr[2],bgr[2]);
  merge(bgr,3,maskedBox);
  showImage(maskedBox);
  cvtColor(maskedBox,gray,CV_BGR2GRAY);
  GaussianBlur(gray,gray,Size(7,7),0);
  Canny(gray,gray,20,50);
  //std::vector<std::vector<Point> > contours;
  findContours(gray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  drawContours(gray,contours,-1,Scalar(255,0,255),1);
  showImage(gray);
  drawContours(gray,contours,-1,Scalar(255),3);
  findContours(gray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  std::vector<Rect > possibleBoxes = determineBoxes(contours); 
  std::vector<Rect> rtn;
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> appr;
    approxPolyDP(contours[i],appr,1,true);
    possibleBoxes.push_back(boundingRect(appr));
  }
  for(int i = 0; i < possibleBoxes.size(); i++) {
    if(possibleBoxes[i].area() > 10000 && std::abs(((double)possibleBoxes[i].height/possibleBoxes[i].width) - BOX_AR) < 0.6) {
      rtn.push_back(possibleBoxes[i]);
    }
  }
  return rtn;
}


void threadDoEveryOtherLaser(std::vector<Mat> imgVec) {
  for(int q = 0; q < imgVec.size(); q++) {
    Mat img = imgVec[q];
      for(int t = 0; t < img.cols; t++) {
	Vec3b at = img.at<Vec3b>(0,t);
	if(at[0] < 200 || at[1] < 200 || at[2] < 200) {
	  Vec3b newVal(0,0,0);
	  img.at<Vec3b>(0,t)=newVal;
	  
	}
    } 
  }
}

Mat changeToMaxLaser (Mat& in) {
  Mat img;
  GaussianBlur(in,img,Size(7,7),0);
  std::vector<Mat> evens;
  std::vector<Mat> odds;
  for(int i = 0; i < img.rows; i++) {
    if(i&2 == 0) {evens.push_back(img.row(i));
    } else {odds.push_back(img.row(i));}
  }
  std::thread first(threadDoEveryOtherLaser,evens);
  std::thread second(threadDoEveryOtherLaser,odds);
  first.join();
  second.join();
  return img;
}


int main() {
  std::string imgName = "17foot_LeftBoxwithLaser.png";
  Mat laser = imread(imgName);
  Rect ROI(laser.cols/2 -25,0,200,laser.rows);
  Mat other = laser(ROI);
  showImage(other);
  other = changeToMaxLaser(other);
  cvtColor(other,other,CV_BGR2GRAY);
  Canny(other,other,50,150);
  std::vector<std::vector<Point> > contours;
  std::vector<std::vector<Point> > keep;
  findContours(other,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> approx;
    approxPolyDP(contours[i],approx,1,true);
    Point2f pt;
    float val = 0;
    minEnclosingCircle(approx,pt,val);
    if(std::abs(PI*val*val -  (float)contourArea(approx)) < 100) {
      keep.push_back(approx);
    }
  }
  drawContours(other,keep,-1,Scalar(100),10);
  showImage(other);
  /* other = createHorizontalDelta(other);
  cvtColor(other,other,CV_BGR2GRAY);
   threshold(other,other,20,255,0);
  showImage(other);
  equalizeHist(other,other);
  showImage(other);
  Canny(other,other,10,15);
  std::vector<std::vector<Point> > contours;
  findContours(other,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  drawContours(other,contours,-1,Scalar(100),1);
  showImage(other);
  // VideoCapture cap(0);

  //////////////////////////////////////////////////////////////
  //WORKING CODE DONT TOUCH
  // std::string imgName = "17foot_BothBoxes.png";
  /*        std::string imgName = "17foot_LeftBox.png";
  Mat box = imread(imgName);
  //cap >> box;
  resize(box,box,Size(640,480));
  std::vector<Rect> boxes = findBoxes(box);
  for(int i = 0; i < boxes.size(); i++) {
    rectangle(box,boxes[i],Scalar(255,0,255),3);
  }
  showImage(box);/*
  //////////////////////////////////////////////////////////////

  
  Mat cpy;
  resize(box,box,Size(640,480));
  Mat bgr[3];
  split(box,bgr);
  equalizeHist(bgr[0],bgr[0]);
  equalizeHist(bgr[1],bgr[1]);
  equalizeHist(bgr[2],bgr[2]);
  GaussianBlur(bgr[0],bgr[0],Size(7,7),0);
  GaussianBlur(bgr[1],bgr[1],Size(7,7),0);
  GaussianBlur(bgr[2],bgr[2],Size(7,7),0);
  // showImage(bgr[1]);
  Scalar stdDev,mean;
  meanStdDev(bgr[1],mean,stdDev);
  std::cout << mean << std::endl;
  threshold(bgr[1],bgr[1],mean[0],255,4);
  //resize(bgr[1],bgr[1],Size(1000,1000));
 // showImage(bgr[1]);


 // showImage(bgr[0]);


 meanStdDev(bgr[0],mean,stdDev);
 std::cout << mean << std::endl;
 threshold(bgr[0],bgr[0],mean[0],255,4);

 Mat mask;
 bitwise_or(bgr[0],bgr[1],mask);
 threshold(mask,mask,1,255,0);
 showImage(mask);
 
 // resize(bgr[0],bgr[0],Size(1000,1000));
 //showImage(bgr[0]);
 Mat out;
 Mat outBgr[3];
 bgr[0].copyTo(outBgr[0],mask);
bgr[1].copyTo(outBgr[1],mask);
bgr[2].copyTo(outBgr[2],mask);
 merge(outBgr,3,out);
 removeRedNotHighest(out);
 out.copyTo(cpy);

 cvtColor(out,out,CV_BGR2GRAY);
 equalizeHist(out,out);
 //  Canny(out,out,100,150);
 std::vector<KeyPoint> keyPts;
 FAST(out,keyPts,10);
 //std::cout << keyPts.size() << std::endl;
 removeKeyPoints(out,keyPts);
 //std::cout << keyPts.size() << std::endl;
  removeHSV(cpy,keyPts);
 //std::cout << keyPts.size() << std::endl;
 //findBox(cpy,keyPts);
 // showImage(cpy);
 drawKeypoints(cpy,keyPts,cpy);
 // showImage(out);
 imwrite("out.png",cpy);
 showImage(cpy);


  /* std::vector<std::vector<Point> > contours;
 findContours(out,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
 contours = filterBasedOnArea(contours);
 drawContours(out,contours,-1,Scalar(100),2);
 showImage(out);*/
 
 /* showImage(bgr[1]);
 showImage(bgr[2]);
 Mat out;
 merge(bgr,3,out);
 showImage(out);*/



  /*
  Mat boxCpy = imread(imgName);
  RNG rng(12345);
  box = filterOutNonBox(box);
  Canny(box,box,50,100);
  // showImage(box);
  std::vector<std::vector<Point> > contours;
  findContours(box,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  drawContours(boxCpy,contours,-1,Scalar(255,0,0),2);
  // showImage(box);
  contours = filterBasedOnArea(contours);
  
  std::vector<Rect> boxes = determineBoxes(contours);
  for(int i = 0; i < boxes.size(); i++) {
    rectangle(boxCpy,boxes[i],Scalar(0,255,0),5);
  }
  Mat show;
  resize(boxCpy,show,Size(1000,1000));
  // GaussianBlur(show,show,Size(7,7),0);
  showImage(show);
   imwrite("out.png",show);
  /*Mat laser = imread("15Feet.png");
  Mat noLaser = imread("15FeetLaser.png");
  Mat diff = getDiff(laser,noLaser);
  Point laserDot =  determineLaserPoint(findCircleContours(diff));
  Point pt(std::abs((laser.cols/2) - laserDot.x), std::abs((laser.rows/2)-laserDot.y));
  std::cout << pt << std::endl;
  std::cout << calcDistance(laserDot.y) << std::endl;
  
    /*Point pt = determineLaserPoint(findCircleContours(noLaser));
    std::cout << pt << std::endl;
    std::vector<KeyPoint> paper = findPaper(laser);
    double widthPixels =  getPerim(paper);
    double distance = calculateDistance(widthPixels,8.6);
    drawKeypoints(laser,paper,laser);*/

  return 0;
}
