#ifndef IMAGEFUNCTION_H
#define IMAGEFUNCTION_H

#include <Python.h>
#include "numpy/ndarraytypes.h"
#include "numpy/ndarrayobject.h"
#include "numpy/npy_common.h"
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv_modules.hpp"

char const* greet( );
void printStr (std::string stuff);
void testing(PyObject* stuff);
//void displayImg(cv::m img);
#endif
