#ifndef NUMPYALLOCATOR_H
#define NUMPYALLOCATOR_H

#include <Python.h>
#include <numpy/ndarrayobject.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>


class NumpyAllocator: public MatAllocator {
public:
NumpyAllocator() { stdAllocator = Mat::getStdAllocator(); };
~NumpyAllocator() {};

UMatData* allocate(PyObject* o, int dims, const int* sizes, int type, size_t* step) const;
UMatData* allocate(int dims0, const int* sizes, int type, void* data, size_t* step, int flags, UMatUsageFlags usageFlags) const;
bool allocate(UMatData* u, int accessFlags, UMatUsageFlags usageFlags) const;
 void deallocate(UMatData* u) const;
};

#endif
