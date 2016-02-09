
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

Point2f determineLaserPoint(std::vector<std::vector<Point> >& pts) {
  Point2f bestPt;
  double bestRad = 1000;
  for(int i = 0; i < pts.size(); i++) {
    std::vector<Point> cur = pts[i];
    Point2f curPoint;
    float curRad;
    minEnclosingCircle(cur,curPoint,curRad);
    if(curRad < bestRad) {
      bestPt = curPoint;
      bestRad = curRad;
    }
  }
  return bestPt;
}

std::vector<std::vector<Point> > findCircleContours (Mat& image) {
  std::vector<std::vector<Point> > contours;
  Canny(image,image,50,150);
  std::vector<std::vector<Point> > approxConts;
  std::vector<std::vector<Point> > circles;
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

boost::python::list getLaserLocation (char * img1, char * img2, int width, int height) {
 cv::Mat image1(cv::Size(width,height),CV_8UC3,img1,cv::Mat::AUTO_STEP);
 cv::Mat image2(cv::Size(width,height),CV_8UC3,img2,cv::Mat::AUTO_STEP);
 cv::Mat diff = getDiff(image1,image2);
 std::vector<std::vector<Point> > contours = findCircleContours(diff);
 Point pt = determineLaserPoint(contours);
 boost::python::list list;
 list.append(pt.x);
 list.append(pt.y);
 return list;
}

Mat valThresholding(Mat& val) {
  Mat cpy;
  threshold(val,cpy,90,255,3);
  threshold(cpy,cpy,150,255,4);
  threshold(cpy,cpy,1,255,0);
  return cpy;
}

Mat hueThresholding(Mat& hue) {
  Mat cpy;
  threshold(hue,cpy,15,255,4);
  threshold(cpy,cpy,1,255,0);
  return cpy;
}

Mat satThresholding(Mat& sat) {
  Mat cpy;
  threshold(sat,cpy,55,255,3);
  threshold(cpy,cpy,150,255,4);
  threshold(cpy,cpy,1,255,0);
  return cpy;
}

Mat filterOutNonBox(Mat&  box) {
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

std::vector<std::vector<Point> > filterBasedOnArea(std::vector<std::vector<Point> >& contours) {
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

std::vector<Rect> determineBoxes(std::vector<std::vector<Point> >& contours) {
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
  double denom = yCoor - FM;
  z = z / denom;
  return z;
}

double getLaserDist (char * img1, char* img2, int width, int height) {
 cv::Mat image1(cv::Size(width,height),CV_8UC3,img1,cv::Mat::AUTO_STEP);
 cv::Mat image2(cv::Size(width,height),CV_8UC3,img2,cv::Mat::AUTO_STEP);
 cv::Mat diff = getDiff(image1,image2);
 std::vector<std::vector<Point> > contours = findCircleContours(diff);
 Point pt = determineLaserPoint(contours);
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

Mat changeToMax (Mat& in) {
  Mat img;
  GaussianBlur(in,img,Size(7,7),0);
  threadDoEveryOther(img);
  return img;
}

Mat makeMask (Mat& in) {
  Mat bgr[3];
  split(in,bgr);
  return bgr[2];
}


std::vector<Rect> findBoxes(Mat& in) {
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
    if(possibleBoxes[i].area() > 10000 && std::abs(((double)possibleBoxes[i].height/possibleBoxes[i].width) - BOX_AR) < 0.6) {
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

Point findLeftCorner(Mat const in, int curX, int curY) {
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




Point findRightCorner(Mat const in, int curX, int curY) {
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


std::vector<Point> findBoxCorner (Mat& color) {
  Mat in;
  GaussianBlur(color,color,Size(7,7),0);
  removeAverageBlackRows(color);
  cvtColor(color,in,CV_BGR2GRAY);
  Canny(in,in,50,50);

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
    changeToMin(curLookAt);
    std::vector<Point> corners = findBoxCorner(curLookAt);
    boost::python::list box;
    for(int t = 0; t < corners.size(); t++) {
      corners[t].x+=cur.x;
      corners[t].y+=cur.y;
      box.append(corners[t].x);
      box.append(corners[t].y);
    }
    rtn.append(box);
  }
  return rtn;
}


//This part is what Python can see
//when the module is imported.
BOOST_PYTHON_MODULE(DroneUtils) {
  using namespace boost::python;
  def("displayImage",displayImage);
  def("getLaserLocation",getLaserLocation);
  def("getLaserDist",getLaserDist);
  def("getPaperContour",getPaperContour);
  def("getPaperDistContour", getPaperDistContour);
  def("getPaperDistByCorner",getPaperDistByCorner);
  def("getPaperByCorner",getPaperByCorner);
  def("paperContour",paperContour);
  def("getAllBoxes",getAllBoxes);
  def("getBoxCorner", getBoxCorner);

}
