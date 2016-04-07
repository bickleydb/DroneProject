#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/persistence.hpp"
#include "opencv2/core/ocl.hpp"
#include "opencv2/core/base.hpp"
#include "opencv2/core/optim.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core/opengl.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "pycompat.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include <algorithm>
#include <numeric>
#include <tuple>

#define PI 3.141592653589793238462643383279502884197169399375105820974944592307816406286 
#define PAPER_WIDTH 8.6
#define LOW_RED 0
#define HIGH_RED 5
#define FOCAL_LENGTH 216.79
#define PAPER_AR 8.5/11.0
#define BOX_AR 44.5/22.5

using namespace cv;
using namespace std;

///////////////////////////////////////////////////////////////////////////
/*
This group of functions is what is able to be used in Python.


char* params is the bits of the image, found with tostring('c')
width and height are self explanitory



 */
void displayImage(char* img, int width, int height);
double getPaperDist(char * img, int width, int height);
void displayImage(char* img, int width, int height);
double getPaperDistByCorner(char * img, int width, int height);
double getLaserDist (char * img1, char* img2, int width, int height, double FB, double FM);
double getPaperDistContour(char * img, int width, int height);

/*
These functions return a python list, when a collection would be the most 
reasonable thing to return. Look at the function in the .cpp file for more
explanation about the format of each returned list.
 */
boost::python::list getLaserLocation (char * img1, char * img2, int width, int height);
boost::python::list getBoxes(char * img, int width, int height);
boost::python::list getPaperByCorner(char * img, int width, int height);
boost::python::list  paperContour(char* img, int width, int height);
///////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
/*

Group of C++ helper functions, not visible in Python.

 */
double getAverageBright(std::vector<Mat> const img);
void displayImageInner(Mat image);
double getWidth(std::vector<KeyPoint> paper);
double getWidth(std::vector<Point> paper);
double calculateDistance(double width, double inchWidth);
void hsvSplit (Mat const img, std::vector<Mat>& vects);
double getAverageBright(std::vector<cv::Mat> const img);
Mat getDiff (Mat& first, Mat& second);
Point2f determineLaserPoint(const Mat& frame);
Mat valThresholding(const Mat& val);
Mat hueThresholding(const Mat& hue);
Mat satThresholding(const Mat& sat);
Mat filterOutNonBox(const Mat& box);
double calcDistance(double yCoor, double FB, double FM);


std::vector<Point> pickBiggestPaper(std::vector<std::vector<Point> >& possibles);
std::vector<std::vector<Point> > getPaperShapedContours(std::vector<std::vector<Point> > contours, double ar);
std::vector<Point> getPaperContour(Mat& img);
std::vector<std::vector<Point> > filterBasedOnArea(const std::vector<std::vector<Point> >& contours);
std::vector<Rect> determineBoxes(const std::vector<std::vector<Point> >& contours);
std::vector<std::vector<Point> > findCircleContours (Mat& image);


////////////////////////////////////////////////////////////////////
