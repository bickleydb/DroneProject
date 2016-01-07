#include "ImageFunction.h"


using namespace std;

char const* greet() {
  return "hello, world";
}

void printStr (std::string stuff) {
  std::cout << stuff << std::endl;
}

void testing(char* data) {
  std::cout << data << std::endl;
  cv::Mat test(4,4,CV_8UC3);
}
