#include <Python.h>

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/ndarrayobject.h>
#include <boost/numpy.hpp>
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
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"

#define PAPER_WIDTH 8.6
#define LOW_RED 0
#define HIGH_RED 5
#define FOCAL_LENGTH 216.79
#define PAPER_AR 8.5/11.0

using namespace cv;
using namespace std;
using namespace boost::numpy;

void displayImage(char* img, int width, int height);
double getPaperDist(char * img, int width, int height);
double getAverageBright(std::vector<Mat> const img);
