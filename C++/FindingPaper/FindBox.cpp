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
#include <queue>
#include <unordered_set>
#include <tuple>

#define LOW_RED  0
#define HIGH_RED  5
#define FOCAL_LENGTH 216.79
#define FM 667.486
#define FB 249.991
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
  //cvtColor(frame,frame,CV_GRAY2BGR);
   // equalizeHist(frame,frame);
   // showImage(frame);
   //threshold(frame,frame,1,255,0);
   //showImage(frame);
  //showImage(frame);
  // Canny(frame,frame,50,150);
  // showImage(frame);


  double min = 0;
  double max = 0;
  Point minLoc;
  Point maxLoc;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc(frame, &min, &max, &min_loc, &max_loc);
  threshold(frame,frame,max-5,0,3);
  showImage(frame);
  return max_loc;
  //  minMaxLoc(frame,min,max,minLoc,maxLoc);
  /*for(int i = 0; i < frame.rows; i++) {
    for(int t = 0; t < frame.cols; t++) {
      if(frame.at<unsigned char>(i,t) == max) {	 
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
    }*/
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
  std::cout << "Y COOR " << yCoor << std::endl;
  double z = FB;
  std::cout << "FB" << z << std::endl;
  double denom = (yCoor - FM);
  std::cout << "yc -fm" << denom << std::endl;
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

void threadDoEveryOtherMin(std::vector<Mat> imgVec) {
  for(int q = 0; q < imgVec.size(); q++) {
    Mat img = imgVec[q];
      for(int t = 0; t < img.cols; t++) {
	Vec3b at = img.at<Vec3b>(0,t);
	if(std::abs(at[0] - at[1]) < 20 && std::abs(at[0]-at[2]) < 20 && std::abs(at[1]-at[2]) < 20) {
	  Vec3b newVal(0,0,0);
	  img.at<Vec3b>(0,t) = newVal;
	}
    } 
  }
}

Mat changeToMin (Mat& in) {
  Mat img;
  GaussianBlur(in,img,Size(7,7),0);
  std::vector<Mat> evens;
  std::vector<Mat> odds;
  for(int i = 0; i < img.rows; i++) {
    if(i&2 == 0) {evens.push_back(img.row(i));
    } else {odds.push_back(img.row(i));}
  }
  std::thread first(threadDoEveryOtherMin,evens);
  std::thread second(threadDoEveryOtherMin,odds);
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

Mat createVerticalDelta1(Mat& in) {
  Mat output;
  in.copyTo(output);
  for(int i = 0; i < in.rows; i++) {
    for(int t = 0; t < in.cols-1; t++) {
      unsigned char cur = in.at<unsigned char>(i,t);
      unsigned char next = in.at<unsigned char>(i,t+1);
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
  mask = makeMask(redMask);
  in.copyTo(maskedBox,mask);
  split(maskedBox,bgr);
  equalizeHist(bgr[0],bgr[0]);
  equalizeHist(bgr[1],bgr[1]);
  equalizeHist(bgr[2],bgr[2]);
  merge(bgr,3,maskedBox);
  
  // showImage(maskedBox);
  cvtColor(maskedBox,gray,CV_BGR2GRAY);
  GaussianBlur(gray,gray,Size(7,7),0);
  Canny(gray,gray,20,50);
  // showImage(gray);
  //std::vector<std::vector<Point> > contours;
  findContours(gray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  drawContours(gray,contours,-1,Scalar(255),3);
  findContours(gray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  std::vector<Rect > possibleBoxes = determineBoxes(contours); 
  std::vector<Rect> rectApprox;
  std::vector<Rect> rtn;
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> appr;
    approxPolyDP(contours[i],appr,1,true);
    rectApprox.push_back(boundingRect(appr));
  }
  possibleBoxes= rectApprox;
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
      for(int t = 0; t < img.cols-1; t++) {
        unsigned char at = img.at<unsigned char>(0,t);
        unsigned char next = img.at<unsigned char>(0,t+1);
	unsigned char diff(std::abs(at-next));
	img.at<unsigned char>(0,t)=diff;	
    } 
  }
}

Mat changeToMaxLaser (Mat& in) {
  Mat img;
  GaussianBlur(in,img,Size(7,7),0);
  cvtColor(img,img,CV_BGR2GRAY);
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


void changeToRed (Mat& in) {
  for(int i = 0; i < in.rows; i++) {
    for(int t = 0; t < in.cols; t++) {
      	Vec3b black = Vec3b(0,0,0);
	Vec3b cur = in.at<Vec3b>(i,t) ;
	if(cur[2] < cur[0] && cur[2] < cur[1]) {
	  in.at<Vec3b>(i,t) = black;
	}
    }
  }
}

bool isUseful(unsigned char i) {
  return i > 1;
}

Point findLeftCorner(Mat const in, int curX, int curY, Mat& drawOn) {
  int startX = curX;
  int startY = curY;

  int changeX[] = {2,1,0,-1,-2};
  int changeY[] = {-1,-1,-1,-1,-1};
  std::vector<int> dx (changeX,changeX+sizeof(changeX)/ sizeof(changeX[0]));
  std::vector<int> dy (changeY, changeY+sizeof(changeY)/ sizeof(changeY[0]));
	     
  bool cont = true;
  int numDiag  = 0;
  int direction = 0;
    Vec3b lookedAt(255,255,0);
  while(true) {
    std::vector<unsigned char> children;
    for(int i = 0; i < dx.size(); i++) {
      if(curX+dx[i] >= 0 && curX+dx[i] < in.rows && curY+dy[i] >= 0 && curY+dy[i] < in.cols) {
	children.push_back(in.at<unsigned char>(curX+dx[i],curY+dy[i]));
	drawOn.at<Vec3b>(curX+dx[i],curY+dy[i]) = lookedAt;
	  } 
    }
    std::vector<unsigned char>::iterator found = std::find_if(children.begin(),children.end(),isUseful);
    int foundX = curX+dx[found-children.begin()];
    int foundY = curY+dy[found-children.begin()];
    
    if(std::abs(foundX-curX) >= 1 && std::abs(foundY-curY) >= 1) {
      int curDirection = foundY-curY;
      if(numDiag == 20  ||  (std::abs(startX-curX) > 5) && numDiag == 10) {
	break;
      } else {
	if(numDiag == 0) {
	  numDiag++;
	  direction = curDirection;
	} else {
	  if(direction == curDirection) {
	    numDiag++;
	  } else {
	    direction = curDirection;
	    numDiag = 1;
	  }
	}
      }
    } else {
      numDiag = 0;
    }
    if(found == children.end()) {
	break;
    } else {
      curX = foundX;
      curY = foundY;  
    }
  }
  // showImage(lookingAt);
  return Point(curY,curX);
}

Point findRightCorner(Mat const in, int curX, int curY, Mat& drawOn) {
   Vec3b lookedAtColor(255,255,0);
  int startX = curX;
  int startY = curY;
  int changeX[] = {2,1,0,-1,-2};
  int changeY[] = {1,1,1,1,1};
  std::vector<int> dx (changeX,changeX+sizeof(changeX)/ sizeof(changeX[0]));
  std::vector<int> dy (changeY, changeY+sizeof(changeY)/ sizeof(changeY[0]));
	     
  bool cont = true;
  int numDiag  = 0;
  int direction = 0;
 
  while(true) {
    std::vector<unsigned char> children;
    for(int i = 0; i < dx.size(); i++) {
       if(curX+dx[i] > 0 && curX+dx[i] < in.rows && curY+dy[i] > 0 && curY+dy[i] < in.cols) {
	children.push_back(in.at<unsigned char>(curX+dx[i],curY+dy[i]));
	drawOn.at<Vec3b>(curX+dx[i],curY+dy[i])=lookedAtColor;
      }
     
    }
    std::vector<unsigned char>::iterator found = std::find_if(children.begin(),children.end(),isUseful);
    int foundX = curX+dx[found-children.begin()];
    int foundY = curY+dy[found-children.begin()];
    
    if(std::abs(foundX-curX) >= 1 && std::abs(foundY-curY) >= 1) {
      int curDirection = foundY-curY;
      if(numDiag == 20 || (std::abs(startX-curX) > 5) && numDiag == 10) {
	break;
      } else {
	if(numDiag == 0) {
	  numDiag++;
	  direction = curDirection;
	} else {
	  if(direction == curDirection) {
	    numDiag++;
	  } else {
	    direction = curDirection;
	    numDiag = 1;
	  }
	}
      }
    } else {
      numDiag = 0;
    }
    if(found == children.end()) {
	break;
    } else {
      curX = foundX;
      curY = foundY;  
    }
  }
  return Point(curY,curX);
}




void removeAverageBlackRows(Mat& img) {
  for(int i = 0; i < img.rows; i++) {
    Mat row = img.row(i);
    Scalar average;
    Scalar stdDev;
    meanStdDev(row,average,stdDev);
    if(average[0] < 10) {
      img.row(i) = Mat::zeros(1,img.cols,CV_8UC3);
    }
  }

}

void showBGR(const Mat& img) {

}




void showHSV( Mat& img) {
  Mat other;
  cvtColor(img,other,CV_BGR2HSV);
  Mat hsv[3];
  split(other,hsv);
  hsv[0] = createVerticalDelta1(hsv[0]); 
  showImage(hsv[0]);
  hsv[1] = createVerticalDelta1(hsv[1]); 
  showImage(hsv[1]);
  hsv[2] = createVerticalDelta1(hsv[2]); 
  showImage(hsv[2]);
}

void removeUnsaturated(Mat& in) {
  Vec3b ignoreColor(6,100,100);
  cvtColor(in,in,CV_BGR2HSV);
  Mat hsv[3];
  split(in,hsv);
  equalizeHist(hsv[1],hsv[1]);
  merge(hsv,3,in);
  for(int i = 0; i < in.rows; i++) {
    for(int t = 0; t < in.cols; t++) {
      Vec3b cur = in.at<Vec3b>(i,t);
      if(cur[1] < 10) {
	in.at<Vec3b>(i,t) = ignoreColor;
      }
    }
  }
  cvtColor(in,in,CV_HSV2BGR);
}

void removeUnsaturatedSecondPass(Mat& in) {
  Vec3b ignoreColor(6,100,100);
  cvtColor(in,in,CV_BGR2HSV);
  for(int i = in.rows/2; i < in.rows; i++) {
    for(int t = 0; t < in.cols; t++) {
      Vec3b cur = in.at<Vec3b>(i,t);
      if(cur[1] < 20) {
	in.at<Vec3b>(i,t) = ignoreColor;
      }
    }
  }
  cvtColor(in,in,CV_HSV2BGR);
}







std::vector<Point> findBoxCorner (Mat& color) {
  Mat in;
  Mat lookingAt = Mat::zeros(Size(color.cols,color.rows),CV_8UC3);
  removeAverageBlackRows(color);
  removeUnsaturated(color);
  removeUnsaturatedSecondPass(color);
  showImage(color);
  imwrite("cur.png",color);
  GaussianBlur(color,color,Size(7,7),0);
  showImage(color);
  cvtColor(color,in,CV_BGR2GRAY);
  Canny(in,in,50,50);
  showImage(in);
  int curX = 0;
  int curY = in.cols/2;
  unsigned char cur = in.at<unsigned char>(curX,curY);
  Vec3b lookedAtColor = Vec3b(0,255,255);
  while(cur < 1) {
    curX++;
    cur = in.at<unsigned char>(curX,curY);
    lookingAt.at<Vec3b>(curX,curY)=lookedAtColor;
    if(curX > color.rows/2) {
      return std::vector<Point>();
    }
  }
  showImage(lookingAt);
  int topLeftX = curX;
  int topLeftY = curY;
  int topRightX = curX;
  int topRightY = curY;
  Point topLeft, topRight;

  topLeft = findLeftCorner(in,topLeftX,topLeftY,lookingAt);
  showImage(lookingAt);
  topRight = findRightCorner(in,topRightX,topRightY,lookingAt);
  showImage(lookingAt);
   curX = in.rows-1;
   curY = in.cols/2;
   cur = in.at<unsigned char>(curX,curY);
 
   while(cur < 5) {
    curX--;
    cur = in.at<unsigned char>(curX,curY);
    lookingAt.at<Vec3b>(curX,curY)=lookedAtColor;
    if(curX < color.rows/2) {
      return std::vector<Point>();
    }
  }
  showImage(lookingAt);
  int bottomLeftX = curX;
  int bottomLeftY = curY;
  int bottomRightX = curX;
  int bottomRightY = curY;
  Point bottomLeft = findLeftCorner(in,bottomLeftX,bottomLeftY,lookingAt);
  Point bottomRight = findRightCorner(in,bottomRightX,bottomRightY,lookingAt);
  // resize(lookingAt,lookingAt,Size(1000,1000));
  cvtColor(in,in,CV_GRAY2BGR);
  bitwise_or(lookingAt,in,lookingAt);
  showImage(lookingAt);
  std::vector<Point> corners;
  corners.push_back(topLeft);
  corners.push_back(topRight);
  corners.push_back(bottomLeft);
  corners.push_back(bottomRight);
		   
  imwrite("LookedAtHigh.png",lookingAt);
  for(int i =0; i < corners.size(); i++) {
    circle(in,corners[i],10,Scalar(255),2);  }
  return corners;
}

void mostRed ( Mat & in) {
  Vec3b red = Vec3b(255,255,255);
  Vec3b black = Vec3b(0,0,0);
  for(int i = 0; i < in.rows; i++) {
    for(int t = 0; t < in.cols; t++) {
      Vec3b cur = in.at<Vec3b>(i,t);
      if(cur[2] > cur[1] && cur[2] > cur[0]) {
	in.at<Vec3b>(i,t) = red;
      } else {
	in.at<Vec3b>(i,t) = black;
      }
    }
  }
 
}

void keepColIf( Mat & rtn) {
 Mat rowBlack = Mat::zeros(rtn.rows,1,CV_8UC3);
  for(int i = 0; i < rtn.cols; i++) {
    Mat rtnRow = rtn.col(i);
    bool hasBrightSpot = false;
    for(int t = 0; t < rtnRow.rows; t++) {
      Vec3b cur = rtnRow.at<Vec3b>(t,0);
      if(cur[0] > 200 && cur[1] > 200 && cur[2] > 200) {
	hasBrightSpot = true;
      }
    }
    if(!hasBrightSpot) {
      rowBlack.copyTo(rtnRow);
    }
  }
}





void keepRowIf ( Mat& rtn) {
  Mat rowBlack = Mat::zeros(1,rtn.cols,CV_8UC3);
  for(int i = 0; i < rtn.rows; i++) {
    Mat rtnRow = rtn.row(i);
    bool hasBrightSpot = false;
    for(int t = 0; t < rtnRow.cols; t++) {
      Vec3b cur = rtnRow.at<Vec3b>(0,t);
      if(cur[0] > 200 && cur[1] > 200 && cur[2] > 200) {
	hasBrightSpot = true;
      }
    }
    if(!hasBrightSpot) {
      rowBlack.copyTo(rtnRow);
    }
  }
}


Mat getWhiteOrRed(const Mat& in) {
  Mat rtn;
  in.copyTo(rtn);
  Vec3b white(255,255,255);
  Vec3b black(0,0,0);
    for(int i = 0; i < rtn.rows; i++) {
    for(int t = 0; t < rtn.cols; t++) {
      Vec3b cur = rtn.at<Vec3b>(i,t);
      if(std::abs(cur[0]-cur[1]) < 10 && std::abs(cur[1] - cur[2]) < 10 && std::abs(cur[0]-cur[2]) < 10) {
		rtn.at<Vec3b>(i,t) = white;
      } else if (cur[2] - cur[1] > 20 && cur[2] -cur[0] > 20) {
		rtn.at<Vec3b>(i,t) = white;
      } else {
	rtn.at<Vec3b>(i,t) = black;
      }
    }
    }
  return rtn;
}

Point getCenter(const std::vector<Point>& vec) {
  double x = 0;
  double y =0 ;
  for(int i = 0; i < vec.size(); i++) {
    x+=vec[i].x;
    y+=vec[i].y;
  }
  return Point(x/vec.size(),y/vec.size());
}

std::pair<int,Vec3b> getAverageSize(Mat& in, const Point startPoint, const std::vector<Point>& contour) {
  Mat other = Mat::zeros(Size(in.cols,in.rows),CV_8UC3);
  //std::cout << "Contour starting at " << startPoint << std::endl;
  // showImage(other);
  std::vector<Point> queue;
  std::vector<Point> lookedAt;
  Vec3b middleVal = in.at<Vec3b>(startPoint);
  unsigned int avgR = 0;
  unsigned int avgG = 0;
  unsigned int avgB = 0;
  unsigned int numLooked = 0;
  int dr[8] = { 0, 1, 1, 1, 0, -1, -1, -1};
  int dc[8] = { -1, -1, 0, 1, 1, 1, 0, -1};
  std::vector<int> deltaRow (dr, dr+sizeof(dr)/sizeof(int));
  std::vector<int> deltaCol (dc, dc+sizeof(dc)/sizeof(int));
  queue.emplace(queue.begin(),startPoint);
  Vec3b white(255,255,255);
  while(queue.size() > 0) {
    Point cur = Point(queue[queue.size()-1].x, queue[queue.size()-1].y);
    lookedAt.push_back(cur);
    queue.pop_back();
    numLooked++;
     Vec3b curVal = in.at<Vec3b>(cur);
     //   std::cout << curVal << std::endl;
     avgB+=curVal[0];
    avgG+=curVal[1];
    avgR+=curVal[2];
    //  std::cout << "START POINT" << cur << std::endl;
    for(int i = 0; i < deltaRow.size(); i++) {
      Point neighbor = Point(cur.x+deltaRow[i],cur.y+deltaCol[i]);
      Vec3b val = in.at<Vec3b>(neighbor);
      if((std::abs(middleVal[0]-val[0]) >= 10 || std::abs(middleVal[1] - val[1]) >= 10 || std::abs(middleVal[2]-val[2]) >= 10) ||( val[2] < 100))  {
	 continue;
       }
       std::vector<Point>::const_iterator found = std::find(lookedAt.begin(),lookedAt.end(),neighbor);
	if(found != lookedAt.end()) {
	  continue;
	}
	double insideContour = pointPolygonTest(contour,neighbor,false);
	if(insideContour > 0){ 
	  // in.at<Vec3b>(neighbor) = Vec3b(255,0,255);
	  //  circle(other,neighbor,5,Scalar(0,0,255));
	  queue.push_back(neighbor);
	  }
    }
  }
  //  std::cout << std::endl << std::endl;
  //howImage(in);
  return std::pair<int,Vec3b>(numLooked, Vec3b(avgB/numLooked, avgG/numLooked, avgR/numLooked));
} 


std::pair<double, double> getWidthHeight (std::vector<Point> contour) {
  double high = -1;
  double low = 10000;
  double left = 10000;
  double right = -1;
  for(int i = 0; i < contour.size(); i++) {
    if(contour[i].x < left) {
      left = contour[i].x;
    } 
    if (contour[i].x > right) {
      right = contour[i].x;
    }
    if(contour[i].y < low) {
      low = contour[i].y;
    } 
    if (contour[i].y > high) {
      high = contour[i].y;
    }

  }
  return std::pair<double,double>(std::abs(left-right),std::abs(high-low));
}

int main() {
  std::string laserStr = "/home/daniel/DroneProject/RangePictures/test.png";
  Mat laser = imread(laserStr);
  Mat beforeMask;
  Mat bgr[3];
  Rect ROI(laser.cols/2 - 25, 0, 200,laser.rows);
  beforeMask = laser(ROI);
  Mat redAndWhite = getWhiteOrRed(beforeMask);
  Mat black;
  Mat smallerLaser;
  beforeMask.copyTo(smallerLaser);
  beforeMask.copyTo(black);
  cvtColor(smallerLaser,smallerLaser,CV_BGR2GRAY);
  GaussianBlur(smallerLaser,smallerLaser,Size(7,7),0);
  Canny(smallerLaser,smallerLaser,50,50);
  Mat save;
  smallerLaser.copyTo(save);
  std::vector<std::vector<Point> > contours;
  std::vector<std::vector<Point> > keep;
  findContours(smallerLaser,contours,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
  drawContours(smallerLaser,contours,-1,Scalar(255),1);
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> approx;
    approxPolyDP(contours[i],approx,5,true);
    if(contourArea(contours[i]) > 2) {
      keep.push_back(contours[i]);
    } 
  }
  for(int i = 0; i < keep.size(); i++) {
    std::vector<Point> hull;
    convexHull(keep[i],hull);
    keep[i] = hull;
  }
  Point best;
  Vec3b bestVec;
  double bestVecVal = 0;
  std::cout << keep.size() << std::endl;
  std::vector<std::vector<Point> > lastStage;
  for(int i = 0; i < keep.size(); i++) {
    Point p = getCenter(keep[i]);
    Vec3b color = black.at<Vec3b>(p);
    if(arcLength(keep[i],true) > 200) {
      continue;
    }
    std::pair<int, Vec3b>  cur = getAverageSize(black,Point(getCenter(keep[i]).x,getCenter(keep[i]).y),keep[i]);
    double goodVal = cur.second[2];
    std::cout << goodVal << std::endl;

    if(goodVal >= 40 ) {
      lastStage.push_back(keep[i]);
    }
  }
  std::vector<std::vector<Point> > goodOnes;
  double bestRatio = 0;
  Point bestPoint = Point(-1,-1);
  for(int i = 0; i < lastStage.size(); i++) {
    Rect cur = boundingRect(lastStage[i]);
    cur.x += ROI.x;
    cur.x -= cur.width/2;
    cur.width += cur.width;
    if(cur.x + cur.width > laser.cols) {
      cur.width = laser.cols - cur.x;
    }
    cur.y -= cur.height/2;
    if(cur.y < 0) {
      cur.y = 0;
    }
    cur.height += cur.height;
    Mat contour = laser(cur);
    cvtColor(contour,contour,CV_BGR2GRAY);
    GaussianBlur(contour,contour,Size(7,7),0);
    Canny(contour,contour,50,150);
    std::vector<std::vector<Point> > stuff;
    findContours(contour,stuff,CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
    for(int i = 0; i < stuff.size(); i++) {
      float rad = 0;
      Point2f cent;
      minEnclosingCircle(stuff[i],cent,rad);
      if(contourArea(stuff[i])/(rad * rad * PI) > 0.75 && contourArea(stuff[i])/(rad * rad * PI) > bestRatio) {
	bestPoint = Point((int)cent.x + cur.x, (int)cent.y + cur.y);
	bestRatio = contourArea(stuff[i])/(rad * rad * PI);
      }
    }
  }
  std::cout << lastStage.size() << std::endl;
  if(bestPoint.x == -1 && bestPoint.y == -1) {
    for(int i = 0; i < lastStage.size(); i++) {
    Rect cur = boundingRect(lastStage[i]);
    cur.x += ROI.x;
    cur.x -= cur.width/2;
    cur.width += cur.width;
    if(cur.x + cur.width > laser.cols) {
      cur.width = laser.cols - cur.x;
    }
    cur.y -= cur.height/2;
    if(cur.y < 0) {
      cur.y = 0;
    }
    cur.height += cur.height;
    Mat contour = laser(cur);
    showImage(contour);
    int numRed = 0;
    int numSat = 0;
    for(int t = 0; t < contour.rows; t++) {
      for(int q = 0; q < contour.cols; q++) {
	Vec3b thisColor = contour.at<Vec3b>(t,q);
	if((thisColor[2] - thisColor[1] > 50 && thisColor[2] - thisColor[0] > 50) && thisColor[1] < 100) {
	  numRed++;
	}
	if((thisColor[2] > 200 && thisColor[1] > 200 && thisColor[0] > 200)) {
	  numSat++;
	}
      }
    }
    double ratioOfRedToNot = (double)(numRed) / (contour.rows * contour.cols - numSat);
    if(ratioOfRedToNot > bestRatio) {
      goodOnes.push_back(lastStage[i]);
      bestRatio = ratioOfRedToNot;
      bestPoint = getCenter(lastStage[i]);
       bestPoint.x += ROI.x;
      }
    }
  }
  std::cout << goodOnes.size() << std::endl;
  circle(laser,bestPoint,20,Scalar(0,0,255),2);
  //std::cout << lastStage.size() << std::endl;
  //drawContours(laser,lastStage,-1,Scalar(255,0,255),1,8,noArray(),INT_MAX,Point(ROI.x,0));
  resize(laser,laser,Size(1000,1000));
  showImage(laser);
  


  //showImage(black);
  /* std::string wolaser = "Danlaser10without.jpg";
  Mat laser = imread(laserStr);
  Mat noLaser = imread(wolaser);
  Mat diff = getDiff(laser,noLaser);
  showImage(diff);
  Rect ROI(laser.cols/2,0,100,3*laser.rows/4);
  Mat grayROI;
  diff(ROI).copyTo(grayROI);
  Mat diffMask;
  threshold(grayROI,diffMask,12,255,0);
  Mat other;
  laser(ROI).copyTo(other);
  showImage(other);
  mostRed(other);
  cvtColor(other,other,CV_BGR2GRAY);
  Mat totalMask;
  bitwise_and(other,diffMask,totalMask);
  Mat onlyLookAt;
  laser(ROI).copyTo(onlyLookAt,totalMask);
  cvtColor(onlyLookAt,onlyLookAt,CV_BGR2GRAY);
  std::vector<std::vector<Point> > contours;
  findContours(onlyLookAt,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  std::vector<std::vector<Point> > circularContours;
  Point2f smallestPoint;
  float largestRad = 0;
  double smallestRatio = 10;
  for(int i = 0; i < contours.size(); i++) {
    Point2f center;
    float rad = 0;
    minEnclosingCircle(contours[i],center,rad);
    double ratio = (rad * rad * PI)/contourArea(contours[i]);
    if(ratio < smallestRatio && largestRad < rad) {
      smallestPoint = center;
      smallestRatio = ratio;
      largestRad = rad;
    }
  }
  std::cout << calcDistance(smallestPoint.y+ROI.y) << std::endl;
  circle(laser,Point(smallestPoint.x+ROI.x,smallestPoint.y+ROI.y),10,Scalar(0,0,255),4);
  resize(laser,laser,Size(1000,1000));
  showImage(laser); 
  /* Point p = getLaserLocation(cur);
  circle(cpyLaser,Point(p.x+ROI.x, p.y),5,Scalar(0,0,255),10);
  std::cout << "LASER Y : " << p.y << std::endl;
  std::cout << "MID " << laser.rows/2 << std::endl;
  std::cout << p.y - laser.rows/2 << std::endl;
  std::cout << calcDistance(p.y) << std::endl;*/
  //  showImage(cpyLaser);
  /*
  /( Rect ROI(laser.cols/2 -25,0,200,laser.rows);
  Mat other = laser(ROI);
  //  showImage(other);
  other = changeToMaxLaser(other);
  threshold(other,other,1,255,0);
  showImage(other);
  //cvtColor(other,other,CV_BGR2GRAY);
  Canny(other,other,50,150);
  showImage(other);
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
  drawContours(other,keep,-1,Scalar(255,255,0),2);
  showImage(other);
  return 0;*/
  //////////////////////////////////////////////////////////////
  //WORKING CODE DONT TOUCH
  //  VideoCapture cap(0);

  /*std::string imgName = "output.png";
  Mat box = imread(imgName);
   
  Mat output;
  box.copyTo(output);
  std::vector<Rect> boxes = findBoxes(box);
  for(int i = 0; i < boxes.size(); i++) {
    Rect cur = boxes[i];
    rectangle(output,boxes[i],Scalar(0,0,255),3);
    cur.x = std::max(cur.x-=10,0);
    cur.y = std::max(cur.y-=10,0);
    cur.width+=20;
    cur.height+=20;
    if(cur.x + cur.width > box.cols) {
      cur.width = box.cols-cur.x;
      //  continue;
    }	
    if(cur.y+cur.height > box.rows) {
      //   continue;
      cur.height = box.rows-cur.y;
    }
    Mat curLookAt = box(cur);
    curLookAt = changeToMin(curLookAt);
    std::vector<Point> corners = findBoxCorner(curLookAt);
    std::cout << corners.size() << std::endl;
    for(int t = 0; t < corners.size(); t++) {
      corners[t].x+=cur.x;
      corners[t].y+=cur.y;
      circle(output,corners[t],5,Scalar(0,0,0),5);
    }
  }
  // showImage(output);  */
  return 0;
}
