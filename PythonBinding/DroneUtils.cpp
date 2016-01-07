#include <boost/python.hpp>

#if defined(_MSC_VER) && (_MSC_VER >= 1800)
// eliminating duplicated round() declaration
#define HAVE_ROUND 1
#endif

#include "DroneUtils.h"
using namespace cv;
using namespace std;


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
	    colored.at<Vec3b>(delta.pt)[0]= 255;
	    colored.at<Vec3b>(delta.pt)[1] = 255;
	    colored.at<Vec3b>(delta.pt)[2] = 255;
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
    } while(points.size() != 4 && offset < 20);
    if(points.size()%4 == 0){return points;}
  } 
}




void displayImage(char* img, int width, int height) {
  cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
  for(;;) {
    cv::imshow("",image);
    char c = cv::waitKey(1);
    if(c == 27) {
      break;
    }
  }
}

void displayImageInner(Mat image) {
  for(;;) {
    cv::imshow("",image);
    char c = cv::waitKey(1);
    if(c == 27) {
      break;
    }
  }
}

double getWidth(std::vector<KeyPoint> paper) {
  assert(paper.size() % 4 == 0);
   double dist1 = std::sqrt(std::pow(paper[0].pt.x - paper[1].pt.x,2) + std::pow(paper[0].pt.y - paper[1].pt.y,2));
  double dist2 = std::sqrt(std::pow(paper[0].pt.x - paper[2].pt.x,2) + std::pow(paper[0].pt.y - paper[2].pt.y,2));
  double dist3 = std::sqrt(std::pow(paper[0].pt.x - paper[3].pt.x,2) + std::pow(paper[0].pt.y - paper[3].pt.y,2));
  double width = std::min(dist1, std::min(dist2,dist3));
  return width;


}

double calculateDistance(double width, double inchWidth) {
  double distance = 0;
  distance = FOCAL_LENGTH;
  distance = distance * inchWidth;
  distance = distance / width;
  return distance;


}

void hsvSplit (Mat const img, std::vector<Mat>& vects) {
  Mat hsv[3];
  Mat hsvVersion;
  cvtColor(img,hsvVersion,CV_BGR2HSV);
  split(img,hsv);
  vects.push_back(hsv[0]);
  vects.push_back(hsv[1]);
  vects.push_back(hsv[2]);
}

double getPaperDist(char * img, int width, int height) {
   cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
   std::vector<Mat> hsv;
   hsvSplit(image,hsv);
   double avgBright = getAverageBright(hsv);
   std::vector<KeyPoint> paper = findPaper(image,avgBright);
   double widthInPix = getWidth(paper);
   double distance = calculateDistance(widthInPix,PAPER_WIDTH);
   return distance;


}

double getAverageBright(std::vector<cv::Mat> const img) {
  Scalar averageBright (0);
  Scalar stdDevBright (0);
  meanStdDev(img[2],averageBright,stdDevBright);
  return averageBright[0];
}




BOOST_PYTHON_MODULE(DroneUtils) {
  using namespace boost::python;
  def("displayImage",displayImage);
  def("getPaperDist",getPaperDist);
}
