
#include <boost/python.hpp>
#if defined(_MSC_VER) && (_MSC_VER >= 1800)
// eliminating duplicated round() declaration
#define HAVE_ROUND 1
#endif

#include "DroneUtils.h"
using namespace cv;
using namespace std;
std::vector<KeyPoint> findPaperCornerBased (Mat frame, double avgBright) {
  //This stuff is setting up the images by
  //    Saving the HSV version of the image
  //    Saving the Gray version of the image
  //    Blurring the grayed image
  Mat colors[3];
  Mat colored;
  Mat copy;
  frame.copyTo(colored);
  cvtColor(colored,colored,CV_BGR2HSV);
  split(frame,colors);
  copy = frame;
  cvtColor(frame,frame,CV_BGR2GRAY);
  GaussianBlur(frame,frame,Size(3,3),0);
  ///////////////////////////////////////////
  //This is an iterative deepening kind of thing, starting keypoint detection 
  //with a large param to find points faster, then going smaller to find more
  //points
  for(int parameter = 10; parameter > 0; parameter--) {
    double offset = 1; //An offset to look at neighbors of key points later
    std:vector<KeyPoint> points;
    std::vector<KeyPoint> allPoints;
    //KeyPoint detection
    //////////////////////////////////////////////////////////////////////
    FAST(frame,allPoints,parameter);
    //////////////////////////////////////////////////////////////////////
    do {      //holy crap a do while loop
      points = std::vector<KeyPoint>(allPoints);
      std::vector<int> keep(points.size()); //Create a vector of ints that will store if they are good points or not
      for (std::vector<KeyPoint>::iterator it = points.begin(); it != points.end(); it++) { //Check all the points
	KeyPoint pt = *it;
	Vec3b at = colored.at<Vec3b>(pt.pt); //Get HSV value of the point
	int numRed = 0;
	int numWhite = 0;

	//This section defines the neighbors we are looking at. They are the change in row and change in column.
	//Im a math guy so I call them deltas
	 double dr[8] = { 0, -1,-1, -1, 0,  1, 1, 1};
	 double dc[8] = { 1, 1,  0, -1,-1,-1, 0, 1};
	 std::vector<int> deltaRow (dr, dr+sizeof(dr)/sizeof(int));
	 std::vector<int> deltaCol (dc, dc+sizeof(dc)/sizeof(int));
	 for(unsigned int i = 0; i < deltaRow.size(); i++) {
	   KeyPoint delta = pt; //Start with the keypoint we are looking at
	   int rowAdd = offset; //Save the offset
	   int colAdd = offset;
	   delta.pt.x = pt.pt.x + (int) (deltaRow[i] * rowAdd); //Offset the coordinates
	   delta.pt.y = pt.pt.y + (int) (deltaCol[i] * colAdd);

	   //Makes sure the neighbor coordinate makes sense to check
	   if(delta.pt.x < 0 || delta.pt.y < 0 || delta.pt.x >= frame.cols || delta.pt.y >= frame.rows) { 
	     continue;
	   }
	   
	   int colorCode = 0; //This variable is the "code" of the color, either 0 if neigher red or white, 1 if red, 2 if white
	   Vec3b neighbor = colored.at<Vec3b>(delta.pt);

	   int count = 0; //This counts the number of values of 255 in the HSV colors. Each value is probably not exactly 255, unless I had previously messed with it, which I do below
	   for(int i = 0; i < 3; i++) {
	     if((int)neighbor[i] == 255){count++;} 
	   }
	   
	   if(count >= 2){ //If it has 2 or more 255s, I changed it before. This seems to make the algorithm faster, but I'm not sure whyy
	     continue;
	   }
	   
	   if((neighbor[0] > LOW_RED || neighbor[0] < HIGH_RED) && neighbor[1] > 73 && neighbor[2] > 50) { //Carefully defined values for red
	     numRed++;
	     colorCode = 1;
	   } else if( neighbor[2] > avgBright) { //I figure, white is bright in HSV, so if the value of a pixel is greater than the average, it is probably white
	     numWhite++;
	     colorCode = 2;
	   }
	  
	   //Debugging code, this doesn't do anything important.
	   /////////////////////////////////////////////////////////
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
	   ////////////////////////////////////////////////////////
	}  

	 //If the keypoint we are looking at does not have 5 red neighbors and 3 white, we don't want it
	if(numRed != 5 ||( numWhite != 3 )) {
	  keep[it - points.begin()] = -1;
	}
      }
      
      //Deletes all of the points that do not satisfy the criteria
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
    } while(points.size() != 4 && offset < 20); //Stops when the offset is large enough, to go back and try FAST again

    if(points.size()%4 == 0){return points;}
    else {
      return std::vector<KeyPoint>();} //If the points couldn't form a box, return empty vector, else return the points
  } 
}



//Python version of display Image method
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

//Debugging method. Use this to display an image, which will be displayed until you hit ESC (at least on Linux/Pi) 
//and then the code will keep going
void displayImageInner(Mat image) {
  for(;;) {
    cv::imshow("",image);
    char c = cv::waitKey(1);
    if(c == 27) {
      break;
    }
  }
}

//This little method gets the width of the piece of paper in pixels.

//It works by finding the distances between the first point and all the other points.
//The width is going to be the smallest out of the width,height, and diagonal, or else the paper would be rectangular
//This method takes a vector of KeyPoints, best used with FAST KeyPoint detection and similar algorithms
double getWidth(std::vector<KeyPoint> paper) {
  assert(paper.size() % 4 == 0);
   double dist1 = std::sqrt(std::pow(paper[0].pt.x - paper[1].pt.x,2) + std::pow(paper[0].pt.y - paper[1].pt.y,2));
  double dist2 = std::sqrt(std::pow(paper[0].pt.x - paper[2].pt.x,2) + std::pow(paper[0].pt.y - paper[2].pt.y,2));
  double dist3 = std::sqrt(std::pow(paper[0].pt.x - paper[3].pt.x,2) + std::pow(paper[0].pt.y - paper[3].pt.y,2));
  double width = std::min(dist1, std::min(dist2,dist3));
  return width;
}

//This little method gets the width of the piece of paper in pixels.

//It works by finding the distances between the first point and all the other points.
//The width is going to be the smallest out of the width,height, and diagonal, or else the paper would be rectangular
//This method takes a vector of Points, best used when working with Contours.
double getWidth(std::vector<Point> paper) {
  assert(paper.size() == 4);
   double dist1 = std::sqrt(std::pow(paper[0].x - paper[1].x,2) + std::pow(paper[0].y - paper[1].y,2));
  double dist2 = std::sqrt(std::pow(paper[0].x - paper[2].x,2) + std::pow(paper[0].y - paper[2].y,2));
  double dist3 = std::sqrt(std::pow(paper[0].x - paper[3].x,2) + std::pow(paper[0].y - paper[3].y,2));
  double width = std::min(dist1, std::min(dist2,dist3));
  return width;
}

//Uses the fomulas Dr. Leonard gave us to determine a distance estimation
double calculateDistance(double width, double inchWidth) {
  double distance = 0;
  distance = FOCAL_LENGTH;
  distance = distance * inchWidth;
  distance = distance / width;
  return distance;


}

//Splits an image into HSV mats, nice little helper function
void hsvSplit (Mat const img, std::vector<Mat>& vects) {
  Mat hsv[3];
  Mat hsvVersion;
  cvtColor(img,hsvVersion,CV_BGR2HSV);
  split(img,hsv);
  vects.push_back(hsv[0]);
  vects.push_back(hsv[1]);
  vects.push_back(hsv[2]);
}

//Function that is called by Python to get the distance 
//to a sheet of paper. 

//A couple notes here:
/*

The char* parameter is the actual data of the Numpy Array that is given. 
The width and height are pretty self explanitory, but I'll describe it in
more detail in the ImageProcessing.py file
 */
double getPaperDistByCorner(char * img, int width, int height) {
   cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
   std::vector<Mat> hsv;
   hsvSplit(image,hsv);
   double avgBright = getAverageBright(hsv);
   std::vector<KeyPoint> paper = findPaperCornerBased(image,avgBright);
   double widthInPix = getWidth(paper);
   double distance = calculateDistance(widthInPix,PAPER_WIDTH);
   return distance;
}

/*
Uses the color of the white piece of paper and a bright red background to
find the corner points of the piece of paper. 

It returns the coordinates of the paper in the form

(xCoor,yCoor),(xCoor,yCoor),....

 */
boost::python::list getPaperByCorner(char * img, int width, int height) {
   cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
   std::vector<Mat> hsv;
   hsvSplit(image,hsv);
   double avgBright = getAverageBright(hsv);
   std::vector<KeyPoint> paper = findPaperCornerBased(image,avgBright);
   boost::python::list rtn;
   for(int i = 0; i < paper.size(); i++) {
     boost::python::list point;
     point.append(paper[i].pt.x);
     point.append(paper[i].pt.y);
     rtn.append(point);
   }
   return rtn;
}


/*
Filters out possible sheets of paper, based on how large it is. If there is a random contour
that has a similar AR to a regular sheet of paper, this method picks the largest contour based
on area, and returns it.

This method is meant for using contours to find the paper, not corner points.

*/
std::vector<Point> pickBiggestPaper(std::vector<std::vector<Point> >& possibles) {
  double bestArea = -1;
  std::vector<Point> bestVect;
  for(int i = 0; i < possibles.size(); i++) {
    double area = contourArea(possibles[i]);
    if(area > bestArea) {
      bestArea = area;
      bestVect = possibles[i];
    }
  }
  return bestVect;
}

/*
Searches through a list  of contours and a given aspect ratio to determine if any of the
contours follow a similar ar to the parameter. 

Every contour that has the AR we are looking for is returned in a vector.
 */
std::vector<std::vector<Point> > getPaperShapedContours(std::vector<std::vector<Point> > contours, double ar) {
  std::vector<std::vector<Point> > rtn;
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> cur =  contours[i];
    approxPolyDP(cur,cur,10,true);
    if(cur.size() >= 4) {
      Rect rectSize = boundingRect(cur);
      double aspectRatio = ((double)rectSize.width)/rectSize.height;
      if(abs(abs(aspectRatio)-abs(ar)) < 0.01) {
	rtn.push_back(cur);
      }
    }
  }
  
  return rtn;
}

/*
Gets the largest paperlike contour that is in the image.

Only returns the largest such contour, which could be problematic.
If you want to get more than one contour, you'll have to change
what it returns.

 */
std::vector<Point> getPaperContour(Mat& img) {
  Mat workOn;
  GaussianBlur(img,workOn,Size(7,7),0,0);
  cvtColor(workOn,workOn,CV_BGR2GRAY);
  Canny(workOn,workOn,50,150);
  std::vector<std::vector<Point> > contours;
  findContours(workOn,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
  contours = getPaperShapedContours(contours,PAPER_AR);
  std::vector<Point> paper = pickBiggestPaper(contours);
  return paper;
}

/*
Returns the python representation of the best paperLike contour,
if one exists.
 */
boost::python::list  paperContour(char* img, int width, int height) {
  cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
  std::vector<Point> paper = getPaperContour(image);
  int paperPoints[2*paper.size()];
  int paperIndex = 0;
 
  boost::python::list list;
  for(int i = 0; i < paper.size(); i++) {
    list.append(paper[i].x);
    list.append(paper[i].y);
  }
  return list;

}

/*
Automatically returns the approximated distance to the sheet of paper
that is found.
 */
double getPaperDistContour(char * img, int width, int height) {
  cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
  std::vector<Point> paper = getPaperContour(image);
  double paperWidth = getWidth(paper);
  double distance = calculateDistance(paperWidth,PAPER_WIDTH);
  return distance;
}



//Returns the average brightness of an image, assuming that
//the image is divided into a vector of Mats, where each
//spot of the vector representing H S or V
double getAverageBright(std::vector<cv::Mat> const img) {
  Scalar averageBright (0);
  Scalar stdDevBright (0);
  meanStdDev(img[2],averageBright,stdDevBright);
  return averageBright[0];
}

/*
Returns the bitwise differance of the grayscale versions of two images

Really helpful for finding a laser dot that is present in one image and not
the other
*/
Mat getDiff (Mat& first, Mat& second) {
  Mat img1,img2;
  cvtColor(first,img1,CV_BGR2GRAY);
  cvtColor(second,img2,CV_BGR2GRAY);
  GaussianBlur(img1,img1,Size(3,3),0);
  GaussianBlur(img2,img2,Size(3,3),0);
  Mat rtn = abs(img1-img2);

  return rtn;
}

/*
Looks through a set of contours, and finds the smallest min enclosing circle.
If used correctly, this will figure out which contours are the laser dot

 */
Point2f determineLaserPoint(std::vector<std::vector<Point> >& pts) {
  Point2f bestPt; 
  double bestRad = 1000; //Just a big number
  for(int i = 0; i < pts.size(); i++) {
    std::vector<Point> cur = pts[i];
    Point2f curPoint;
    float curRad;
    minEnclosingCircle(cur,curPoint,curRad);
    if(curRad < bestRad) { //Pick for smallest area
      bestPt = curPoint;
      bestRad = curRad;
    }
  }
  return bestPt;
}

/*
Finds all of the circle contours is a circle, or close to one

 */
std::vector<std::vector<Point> > findCircleContours (const Mat& input) {
  std::vector<std::vector<Point> > contours;
  std::vector<std::vector<Point> > approxConts;
  std::vector<std::vector<Point> > circles;
  Mat image;
  Canny(input,image,50,150);
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

Point2f getLaserLoc( Mat& image1,  Mat & image2) {
  cv::Mat diff = getDiff(image1,image2);
  imwrite("FirstImg.png",image1);
  imwrite("SecondImg.png",image2);
  imwrite("Diff.png",diff);
  Rect ROI(image1.cols/2,0,100,3*image1.rows/4);
  Mat grayROI;
  diff(ROI).copyTo(grayROI);
  imwrite("Difference.png",grayROI);
  Mat diffMask;
  threshold(grayROI,diffMask,12,255,0);
  Mat other;
  image1(ROI).copyTo(other);
  mostRed(other);
  cvtColor(other,other,CV_BGR2GRAY);
  Mat totalMask;
  bitwise_and(other,diffMask,totalMask);
  Mat onlyLookAt;
  image1(ROI).copyTo(onlyLookAt,totalMask);
  cvtColor(onlyLookAt,onlyLookAt,CV_BGR2HSV);
  Mat hsv[3];
  split(onlyLookAt,hsv);  
  double max = 0;
  double min = 0;
  Point maxLoc = Point();
  Point minLoc = Point();
  minMaxLoc(hsv[2],&min,&max,&minLoc,&maxLoc);
  /* std::vector<std::vector<Point> > contours;
  findContours(onlyLookAt,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  drawContours(onlyLookAt, contours,-1,Scalar(255),3);
  //imwrite("OnlyLookAt.jpg",onlyLookAt);
  std::vector<Point2f > circularContours;
  Point2f bestPoint(0,(float)(image1.rows));
  float largestRad = 0;
  double smallestRatio = 10;
  for(int i = 0; i < contours.size(); i++) {
    Point2f center;
    float rad = 0;
    minEnclosingCircle(contours[i],center,rad);
    double ratio = (rad * rad * PI)/contourArea(contours[i]);
    if(ratio < 2) {
      circularContours.push_back(center);
    }
  }
  
  std::cout << circularContours.size() << std::endl;
  for(int i = 0; i < circularContours.size(); i++) {
    if(circularContours[i].y < bestPoint.y) {
      bestPoint = circularContours[i];
    }
    }*/
    return maxLoc;
}


boost::python::list getLaserLocation (char * img1, char * img2, int width, int height) {
 cv::Mat image1(cv::Size(width,height),CV_8UC3,img1,cv::Mat::AUTO_STEP);
 cv::Mat image2(cv::Size(width,height),CV_8UC3,img2,cv::Mat::AUTO_STEP);
 imwrite("Img1.jpg",image1);
 imwrite("Img2.jpg",image2);
 Point2f bestPoint = getLaserLoc(image1, image2);
  boost::python::list rtn;
  rtn.append(bestPoint.x);
  rtn.append(bestPoint.y);
  return rtn;
}

Mat valThresholding(const Mat& val) {
  Mat cpy;
  threshold(val,cpy,90,255,3);
  threshold(cpy,cpy,150,255,4);
  threshold(cpy,cpy,1,255,0);
  return cpy;
}

Mat hueThresholding(const Mat& hue) {
  Mat cpy;
  threshold(hue,cpy,15,255,4);
  threshold(cpy,cpy,1,255,0);
  return cpy;
}

Mat satThresholding(const Mat& sat) {
  Mat cpy;
  threshold(sat,cpy,55,255,3);
  threshold(cpy,cpy,150,255,4);
  threshold(cpy,cpy,1,255,0);
  return cpy;
}

Mat filterOutNonBox(const Mat&  box) {
  Mat blurred,hsvImg,rtn,mask;
  Mat hsv[3];
  GaussianBlur(box,blurred,Size(7,7),0);
  cvtColor(blurred,hsvImg,CV_BGR2HSV);
  split(hsvImg,hsv);
  Mat maskH = hueThresholding(hsv[0]);
  Mat maskV = valThresholding(hsv[2]);
  Mat maskS = satThresholding(hsv[1]);
  bitwise_and(maskH,maskV,mask);
  bitwise_and(mask,maskS,mask);
  hsv[2].copyTo(rtn,mask);
  return rtn;
}

std::vector<std::vector<Point> > filterBasedOnArea(const std::vector<std::vector<Point> >& contours) {
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

std::vector<Rect> determineBoxes(const std::vector<std::vector<Point> >& contours) {
  std::vector<Rect> rtn;
  for(int i = 0; i < contours.size(); i++) {
    Rect possible = boundingRect(contours[i]);
    int width = possible.width;
    int height = possible.height;
    double larger = (double) std::max(width,height);
    double smaller = (double) std::min(width,height);
    if(larger/smaller < BOX_AR + 0.5 && larger/smaller > BOX_AR - 0.5) {
      rtn.push_back(possible);
    }
  }
  return rtn;
}

boost::python::list getBoxes(char * img, int width, int height) {
  cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
  image = filterOutNonBox(image);
  Canny(image,image,50,100);
  std::vector<std::vector<Point> > contours;
  findContours(image,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  contours = filterBasedOnArea(contours);
  drawContours(image,contours,-1,Scalar(255),3);
  resize(image,image,Size(1000,1000));
  displayImageInner(image);
  contours = filterBasedOnArea(contours);
  std::vector<Rect> boxes = determineBoxes(contours);
  boost::python::list rtn;
  for(int i = 0; i < boxes.size(); i++) {
    boost::python::list rect;
    rect.append(boxes[i].x);
    rect.append(boxes[i].y);
    rect.append(boxes[i].width);
    rect.append(boxes[i].height);
    rtn.append(rect);
  }
  return rtn;
}


double calcDistance(double yCoor) {
  double z = FB;
  double denom = std::abs(yCoor - FM);
  z = z / denom;
  return z;
}

double getLaserDist (char * img1, char* img2, int width, int height) {
 cv::Mat image1(cv::Size(width,height),CV_8UC3,img1,cv::Mat::AUTO_STEP);
 cv::Mat image2(cv::Size(width,height),CV_8UC3,img2,cv::Mat::AUTO_STEP);
 Point2f pt = getLaserLoc(image1,image2);
 return calcDistance(pt.y);
}

void threadDoEveryOther(Mat& img) {
  for(int i = 0; i < img.rows; i++) {
    for(int t = 0; t < img.cols; t++) {
      Vec3b at = img.at<Vec3b>(i,t);
      if(std::abs(at[2]-at[1]) < 20) {
	Vec3b newVec(0,0,0);
	img.at<Vec3b>(i,t) = newVec;
      } else if(at[0] > at[1] && at[0] > at[2]) {
	Vec3b newVec(0,0,0);
	img.at<Vec3b>(i,t) = newVec;
      } else if(at[1] > at[0] && at[1] > at[2]) {
	Vec3b newVec(0,0,0);
	img.at<Vec3b>(i,t) = newVec;
      } else if(at[2] > at[0] && at[2] > at[1]) {
	Vec3b newVec(0,0,255);
	img.at<Vec3b>(i,t) = newVec;
      } else {
	Vec3b newVec(0,0,0);
	img.at<Vec3b>(i,t) = newVec;
      }
    }
  }
}

Mat changeToMax (const Mat& in) {
  Mat img;
  GaussianBlur(in,img,Size(7,7),0);
  threadDoEveryOther(img);
  return img;
}

Mat makeMask (const Mat& in) {
  Mat bgr[3];
  split(in,bgr);
  return bgr[2];
}


std::vector<Rect> findBoxes(const Mat& in) {
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
  cvtColor(maskedBox,gray,CV_BGR2GRAY);
  GaussianBlur(gray,gray,Size(7,7),0);
  Canny(gray,gray,20,50);
  findContours(gray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  drawContours(gray,contours,-1,Scalar(255),3);
  findContours(gray,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
  std::vector<Rect > possibleBoxes = determineBoxes(contours); 
  std::vector<Rect> approximated;
  std::vector<Rect> rtn;
  for(int i = 0; i < contours.size(); i++) {
    std::vector<Point> appr;
    approxPolyDP(contours[i],appr,1,true);
    approximated.push_back(boundingRect(appr));
  }
  possibleBoxes = approximated;
  for(int i = 0; i < possibleBoxes.size(); i++) {
    if(possibleBoxes[i].area() > 100000 && std::abs(((double)possibleBoxes[i].height/possibleBoxes[i].width) - BOX_AR) < 0.6) {
      rtn.push_back(possibleBoxes[i]);
    }
  }
  return rtn;
}

bool isUseful(unsigned char i) {
  return i > 1;
}

boost::python::list getAllBoxes (char * img1, int width, int height) {
 cv::Mat image1(cv::Size(width,height),CV_8UC3,img1,cv::Mat::AUTO_STEP);
 std::vector<Rect> boxes = findBoxes(image1);
 boost::python::list rtn;
 for(int i = 0; i < boxes.size(); i++) {
   boost::python::list rect;
   rect.append(boxes[i].x);
   rect.append(boxes[i].y);
   rect.append(boxes[i].width);
   rect.append(boxes[i].height);
   rtn.append(rect);
 }
 return rtn;
}

Point findLeftCorner(const Mat in, int curX, int curY) {
  int startX = curX;
  int startY = curY;
  Mat lookingAt;
  in.copyTo(lookingAt);
  int changeX[] = {2,1,0,-1,-2};
  int changeY[] = {-1,-1,-1,-1,-1};
  std::vector<int> dx (changeX,changeX+sizeof(changeX)/ sizeof(changeX[0]));
  std::vector<int> dy (changeY, changeY+sizeof(changeY)/ sizeof(changeY[0]));
	     
  bool cont = true;
  int numDiag  = 0;
  int direction = 0;
 
  while(true) {
    std::vector<unsigned char> children;
    for(int i = 0; i < dx.size(); i++) {
      if(curX+dx[i] >= 0 && curX+dx[i] < in.rows && curY+dy[i] >= 0 && curY+dy[i] < in.cols) {
	children.push_back(in.at<unsigned char>(curX+dx[i],curY+dy[i]));
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
 
  return Point(curY,curX);
}




Point findRightCorner(const Mat in, int curX, int curY) {
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
      }
      Vec3b stuff(255,255,0);
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


std::vector<Point> findBoxCorner (const Mat& color) {
  Mat in;
  color.copyTo(in);
  removeAverageBlackRows(in);
  removeUnsaturated(in);
  removeUnsaturatedSecondPass(in);
  GaussianBlur(in,in,Size(7,7),0);
  cvtColor(in,in,CV_BGR2GRAY);
  Canny(in,in,50,50);
  imwrite("BoxOnPi.png",in);
  int curX = 0;
  int curY = in.cols/2;
  unsigned char cur = in.at<unsigned char>(curX,curY);
  Vec3b lookedAtColor = Vec3b(0,255,255);
  while(cur < 1) {
    curX++;
    cur = in.at<unsigned char>(curX,curY);
    if(curX > color.rows/2) {
      return std::vector<Point>();
    }
  }
  int topLeftX = curX;
  int topLeftY = curY;
  int topRightX = curX;
  int topRightY = curY;
  Point topLeft, topRight;

   topLeft = findLeftCorner(in,topLeftX,topLeftY);
   topRight = findRightCorner(in,topRightX,topRightY);
   curX = in.rows;
  
   curY = in.cols/2;
   cur = in.at<unsigned char>(curX,curY);
  while(cur < 1) {
    curX--;
    in.at<unsigned char>(curX,curY-1) = (unsigned char) (100);
    cur = in.at<unsigned char>(curX,curY);
    if(curX < color.rows/2) {
      return std::vector<Point>();
    }
  }
  int bottomLeftX = curX;
  int bottomLeftY = curY;
  int bottomRightX = curX;
  int bottomRightY = curY;
  Point bottomLeft = findLeftCorner(in,bottomLeftX,bottomLeftY);
  Point bottomRight = findRightCorner(in,bottomRightX,bottomRightY);
  std::vector<Point> corners;
  corners.push_back(topLeft);
  corners.push_back(topRight);
  corners.push_back(bottomLeft);
  corners.push_back(bottomRight);
		   
  for(int i =0; i < corners.size(); i++) {
    circle(in,corners[i],10,Scalar(255),2);
  }
  return corners;
}

void changeToMin (Mat& in) {
  Vec3b black(0,0,0);
  for(int i = 0; i < in.rows; i++) {
    for(int t = 0; t < in.cols; t++) {
      Vec3b at = in.at<Vec3b>(i,t);
      if(std::abs(at[0] - at[1]) < 20 && std::abs(at[0]-at[2]) < 20 && std::abs(at[1]-at[2]) < 20) {
	  in.at<Vec3b>(i,t) = black;
	}
    }
  }
}



boost::python::list getBoxCorner(char* img, int width, int height) {
  cv::Mat image(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
  std::vector<Rect> boxes = findBoxes(image);
  boost::python::list rtn;
  for(int i = 0; i < boxes.size(); i++) {
    Rect cur = boxes[i];
    cur.x = std::max(cur.x-=10,0);
    cur.y = std::max(cur.y-=10,0);
    cur.width+=20;
    cur.height+=20;
    if(cur.x + cur.width > image.cols) {
      cur.width = image.cols-cur.x;
      continue;
    }	
    if(cur.y+cur.height > image.rows) {
      continue;
      cur.height = image.rows-cur.y;
    }
    Mat curLookAt = image(cur);
    std::cout << "A" << std::endl;
    imwrite("curLookedAt.png",curLookAt);
    changeToMin(curLookAt);
    std::vector<Point> corners = findBoxCorner(curLookAt);
    boost::python::list box;
    for(int t = 0; t < corners.size(); t++) {
      corners[t].x+=cur.x;
      corners[t].y+=cur.y;
      circle(image,corners[t],5,Scalar(0,0,0),5);
      box.append(corners[t].x);
      box.append(corners[t].y);
    }
    rtn.append(box);
  }
  return rtn;
}

double calculateSxy (std::vector<double> xVals, std::vector<double> yVal) {
  double sumX = std::accumulate(xVals.begin(),xVals.end(),0.0);
  double sumY = std::accumulate(yVal.begin(),yVal.end(),0.0);
  double sumXY = 0;
  for(int i = 0; i < xVals.size(); i++) {
    sumXY += xVals[i]*yVal[i];
  }
  return sumXY - (sumX * sumY)/xVals.size();
}

double calcMean(std::vector<double> vals) {
  double sum = std::accumulate(vals.begin(),vals.end(),0.0);
  return sum/vals.size();
}


Point determineConstants(std::vector<double> xVals, std::vector<double> yVal) {
    std::vector<double> Y(xVals.begin(),xVals.end());
    for(std::vector<double>::iterator i = yVal.begin(); i < yVal.end(); i++) {
      Y[i-yVal.begin()]= Y[i-yVal.begin()] * *i;
    }
    double Sxy = calculateSxy(xVals,Y);
    double Sxx = calculateSxy(xVals,xVals);
    double b1 = Sxy/Sxx;
    double yBar = calcMean(Y);
    double xBar = calcMean(xVals);
    return Point(b1,yBar-b1*xBar);
    
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


void getLaserConstants(boost::python::list xyPairs) {
  std::vector<double> xVals;
  std::vector<double> yVals;
  for( int i = 0; i < boost::python::len(xyPairs); i++) {
    boost::python::list pt = boost::python::extract<boost::python::list>(xyPairs[i]);
    xVals.push_back(boost::python::extract<double>(pt[0]));
    yVals.push_back(boost::python::extract<double>(pt[1]));
  }
  Point p  = determineConstants(xVals,yVals);
  std::cout << "FM " << p.x << std::endl;
  std::cout << "FB " << p.y << std::endl;

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
     avgB+=curVal[0];
    avgG+=curVal[1];
    avgR+=curVal[2];
    //  std::cout << "START POINT" << cur << std::endl;
    for(int i = 0; i < deltaRow.size(); i++) {
      Point neighbor = Point(cur.x+deltaRow[i],cur.y+deltaCol[i]);
      Vec3b val = in.at<Vec3b>(neighbor);
      if((std::abs(middleVal[0]-val[0]) >= 10 || std::abs(middleVal[1] - val[1]) >= 10 || std::abs(middleVal[2]-val[2]) >= 10) ||( val[2] < 150))  {
	 continue;
       }
       std::vector<Point>::const_iterator found = std::find(lookedAt.begin(),lookedAt.end(),neighbor);
	if(found != lookedAt.end()) {
	  continue;
	}
	double insideContour = pointPolygonTest(contour,neighbor,false);
	if(insideContour > 0){ 
	  in.at<Vec3b>(neighbor) = Vec3b(0,0,255);
	  //  circle(other,neighbor,5,Scalar(0,0,255));
	  queue.push_back(neighbor);;
	  }
    }
  }
  return std::pair<int,Vec3b>(numLooked, Vec3b(avgB/numLooked, avgG/numLooked, avgR/numLooked));
} 


Point getLaserDotLoc(Mat& laser) {
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
    if(goodVal > 100 ) {
      lastStage.push_back(keep[i]);
    }
  }

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
      if(contourArea(stuff[i])/(rad * rad * PI) > 0.70 && contourArea(stuff[i])/(rad * rad * PI) > bestRatio) {
	bestPoint = Point((int)cent.x + cur.x, (int)cent.y + cur.y);
	bestRatio = contourArea(stuff[i])/(rad * rad * PI);
      }
    }
  }
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
    int numRed = 0;
    for(int t = 0; t < contour.rows; t++) {
      for(int q = 0; q < contour.cols; q++) {
	Vec3b thisColor = contour.at<Vec3b>(t,q);
	if((thisColor[2] - thisColor[1] > 50 && thisColor[2] - thisColor[0] > 50) && thisColor[1] < 100) {
	  numRed++;
	}
      }
    }
    double ratioOfRedToNot = (double)(numRed) / (contour.rows * contour.cols);
    if(ratioOfRedToNot > bestRatio) {
      bestRatio = ratioOfRedToNot;
      bestPoint = getCenter(lastStage[i]);
      bestPoint.x += ROI.x;
      }
    }
  }
  return Point(bestPoint.x+ROI.x,bestPoint.y);
}


void getLaserDistOneImgLoc(char * img, int width, int height) {
  cv::Mat laser(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
  Point best = getLaserDotLoc(laser);
  std::cout << best << std::endl;
}




double getLaserDistOneImg(char * img, int width, int height) {
  cv::Mat laser(cv::Size(width,height),CV_8UC3,img,cv::Mat::AUTO_STEP);
  Point best = getLaserDotLoc(laser);
  return calcDistance(best.y);
}

double getLaserLocTest(char * img1, char* img2 , int width, int height) {
  cv::Mat laser(cv::Size(width,height),CV_8UC3,img1,cv::Mat::AUTO_STEP);
  cv::Mat noLaser(cv::Size(width,height),CV_8UC3,img2,cv::Mat::AUTO_STEP);
  Mat grayLaser;
  cvtColor(laser,grayLaser,CV_BGR2GRAY);
  cvtColor(noLaser,noLaser,CV_BGR2GRAY);
  Mat mask;
  Mat goodStuff;
  absdiff(grayLaser,noLaser,mask);
  threshold(mask,mask,0,255,0);
  Mat doWorkOn;
  laser.copyTo(doWorkOn,mask);
  Point best = getLaserDotLoc(doWorkOn);
  std::cout << best.x << "," << best.y << std::endl;
}




//This part is what Python can see
//when the module is imported.
BOOST_PYTHON_MODULE(DroneUtils) {
  using namespace boost::python;
  def("displayImage",displayImage);
  def("getLaserLocTest",getLaserLocTest);
  def("getLaserConstants",getLaserConstants);
  def("getLaserLocation",getLaserLocation);
  def("getLaserDist",getLaserDist);
  def("getPaperContour",getPaperContour);
  def("getPaperDistContour", getPaperDistContour);
  def("getPaperDistByCorner",getPaperDistByCorner);
  def("getPaperByCorner",getPaperByCorner);
  def("paperContour",paperContour);
  def("getAllBoxes",getAllBoxes);
  def("getBoxCorner", getBoxCorner);
  def("getLaserDistOneImg",getLaserDistOneImg);
  def("getLaserDistOneImgLoc",getLaserDistOneImgLoc);
  def("getLaserConstants",getLaserConstants);
  

}
