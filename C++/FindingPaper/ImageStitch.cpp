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
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include <chrono>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <random>
#include <algorithm>


using namespace cv;
using namespace cv::xfeatures2d;


const int LOOP_NUM = 10;
const int GOOD_PTS_MAX = 50;
const float GOOD_PORTION = 1;


void showImage(Mat img) {
  for(;;) {
    imshow("Stuff",img);
    char c = waitKey(1);
      if(c == 27) {break;}
  }
   imwrite("AlgoOutput.png",img);
}

int main() {
  std::string image1 = "/home/daniel/DroneProject/RangePictures/topRight.png";
  std::string image2 = "/home/daniel/DroneProject/RangePictures/btmRight.png";
  Mat img1Gray, img2Gray;  

  Mat img1 = imread(image1);
  Mat img2 = imread(image2);
  std::vector<KeyPoint> img1KeyPts;
  std::vector<KeyPoint> img2KeyPts;
  Mat img1Descript;
  Mat img2Descript;
  cvtColor(img1,img1Gray,CV_BGR2GRAY);
  cvtColor(img2,img2Gray,CV_BGR2GRAY);

  //drawKeypoints(img1,img1KeyPts,img1);
  //drawKeypoints(img2,img2KeyPts,img2);
  // showImage(img1);
  //showImage(img2);
  
  // Ptr<DescriptorExtractor> test = DescriptorExtractor::create("SURF");
  Ptr<Feature2D> test = SURF::create(800);
  test->detectAndCompute(img1Gray,img1Gray,img1KeyPts,img1Descript,false);
  test->detectAndCompute(img2Gray,img2Gray,img2KeyPts,img2Descript,false);
  
  std::vector<DMatch> matches;
  std::vector< DMatch > good_matches;
  Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
  matcher->match(img1Descript,img2Descript,matches);
  std::cout << "lol" << std::endl;

  std::sort(matches.begin(), matches.end());
  double minDist = matches.front().distance;
  double maxDist = matches.back().distance;
  const int ptsPairs = std::min(GOOD_PTS_MAX, (int)(matches.size() * GOOD_PORTION));
  for( int i = 0; i < ptsPairs; i++ ) {
    good_matches.push_back( matches[i] );
  }
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( int i = 0; i < good_matches.size(); i++ ) {
    //-- Get the keypoints from the good matches
    obj.push_back( img1KeyPts[ good_matches[i].queryIdx ].pt );
    scene.push_back( img2KeyPts[ good_matches[i].trainIdx ].pt );
  }
  
  std::cout << "Finding Homography" << std::endl;
  Mat H = findHomography(obj,scene,CV_RANSAC);
  std::cout << "Found Homography" << std::endl;
  Mat warped;
  warpPerspective(img1,warped,H,Size(img1.cols+img2.cols,std::max(img1.rows,img2.rows)));
  Mat roi(warped,Rect(0,0,img2.cols,img2.rows));
  img2.copyTo(roi);
  // Mat half(warped,Rect(0,0,img2.cols,img2.rows));
  // showImage(warped);
  // Mat final(Size(warped.cols + img1.cols, std::max(img1.rows,warped.rows)*2),CV_8UC3);
  //Mat roi1(final,Rect(0,0,img1.cols,img1.rows));
  //Mat roi2(final,Rect(img1.cols,0,warped.cols,warped.rows));
  //  img2.copyTo(half);
  showImage(warped);
  imwrite("output.jpg",warped);
	   /*
  warped.copyTo(roi1);
  showImage(roi1);
  img2.copyTo(roi2);
  showImage(final);



  /* Mat roi(warped,Rect(0,0,img1.cols,img1.rows));
  img1.copyTo(roi);
  showImage(roi);/*
   //Point a cv::Mat header at it (no allocation is done)
    Mat final(Size(img2.cols*2 + img1.cols, img2.rows*2),CV_8UC3);
   
   //velikost img1
   Mat roi1(final, Rect(0, 0,  img1.cols, img1.rows));
   Mat roi2(final, Rect(img1.cols, 0, warped.cols, warped.rows));
   warped.copyTo(roi2);
   img1.copyTo(roi1);
   showImage(final);
   imwrite("HomographyTest.jpg",final);
   //imshow("final", final);
   std::cout << "test" << std::endl;*/

    Mat img_matches;
  drawMatches( img1, img1KeyPts, img2, img2KeyPts,
	       good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
	       std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS  );



  resize(img_matches,img_matches,Size(1500,1000));
  showImage(img_matches);
  
}
