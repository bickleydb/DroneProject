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

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


std::vector<KeyPoint> findPaper (Mat frame) {
  Mat colors[3];
  Mat colored;
  Mat copy;
    colored = frame;
    split(frame,colors);
    copy = frame;
   
    cvtColor(frame,frame,CV_BGR2GRAY);
    GaussianBlur(frame,frame,Size(3,3),0);

   
    for(int parameter = 10; parameter > 0; parameter--) {
    double dr[8] = { 0, -1,-1, -1, 0,  1, 1, 1};
    double dc[8] = { 1, 1,  0, -1,-1,-1, 0, 1};
    std::vector<int> deltaRow (dr, dr+sizeof(dr)/sizeof(int));
    std::vector<int> deltaCol (dc, dc+sizeof(dc)/sizeof(int));
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
	  //colored.at<Vec3b>(delta.pt)[0]= 255;
    	  //colored.at<Vec3b>(delta.pt)[1] = 255;
	  //colored.at<Vec3b>(delta.pt)[2] = 255;
	  continue;
	}
	
	if((int)neighbor[2] > 2*(int)neighbor[1] && (int)neighbor[2] > 2*(int)neighbor[0] && (int)neighbor[2] > 0) {
	  numRed++;
	  colorCode = 1;
	} else if( abs(((int)neighbor[2]-(int)neighbor[1]))< 50 && abs(((int)neighbor[2]-(int)neighbor[0])) < 50 && abs(((int)neighbor[0]-(int)neighbor[1])) < 50) {
	  numWhite++;
	  colorCode = 2;
	}

	if(colorCode == 0) {
	  //colored.at<Vec3b>(delta.pt)[0]= 255;
	  //colored.at<Vec3b>(delta.pt)[1] = 255;
	  //colored.at<Vec3b>(delta.pt)[2] = 255;
	} else if (colorCode == 1) {
	colored.at<Vec3b>(delta.pt)[0]= 255;
	colored.at<Vec3b>(delta.pt)[1] = 255;
	colored.at<Vec3b>(delta.pt)[2] = 0;
	} else if (colorCode == 2) {
	colored.at<Vec3b>(delta.pt)[0]= 0;
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

int main() {
    Mat frame = imread("15Feet.png");
    std::vector<KeyPoint> paper = findPaper(frame);
  
    double dist1 = std::sqrt(std::pow(paper[0].pt.x - paper[1].pt.x,2) + std::pow(paper[0].pt.y - paper[1].pt.y,2));
    double dist2 = std::sqrt(std::pow(paper[0].pt.x - paper[2].pt.x,2) + std::pow(paper[0].pt.y - paper[2].pt.y,2));
    double dist3 = std::sqrt(std::pow(paper[0].pt.x - paper[3].pt.x,2) + std::pow(paper[0].pt.y - paper[3].pt.y,2));
    std::cout << dist1 << std::endl;
    std::cout << dist2 << std::endl;
    std::cout << dist3 << std::endl;

    double width = std::min(dist1, std::min(dist2,dist3));
    double height = dist1 + dist2 + dist3 - width - std::max(dist1,std::max(dist2,dist3));

    std::cout << width << std::endl;
    std::cout << height << std::endl;
   

    std::cout << "Pixel to Inch Ratio Based On Width= " << width/8.6 << std::endl;
    std::cout << "Pixel to Inch Ratio Based On Height= " << height/11 << std::endl;
    

    drawKeypoints(frame,paper,frame);
   
    for(;;) {
       imshow("Stuff",frame);
      imwrite("Test.bmp",frame);
 

      char c = (char)waitKey(1);
       if(c == 27) break;

       }
  return 0;
}
