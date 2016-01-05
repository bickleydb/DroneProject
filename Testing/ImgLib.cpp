#include <boost/python.hpp>
#include "ImageFunction.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

BOOST_PYTHON_MODULE(ImgProc) {
  using namespace boost::python;

  def( "greet", greet);
  def( "printStr", printStr);
  def( "testing", testing);
}
