#include <iostream>
#include <vector>
#include <algorithm>

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


void determineConstants(std::vector<double> xVals, std::vector<double> yVal) {
    std::vector<double> Y(xVals.begin(),xVals.end());
    for(std::vector<double>::iterator i = yVal.begin(); i < yVal.end(); i++) {
      Y[i-yVal.begin()]= Y[i-yVal.begin()] * *i;
    }
    //   std::for_each(Y.begin(),Y.end(),[](double i) { std::cout << i << std::endl;});
    double Sxy = calculateSxy(xVals,Y);
    double Sxx = calculateSxy(xVals,xVals);
    double b1 = Sxy/Sxx;
    double yBar = calcMean(Y);
    double xBar = calcMean(xVals);
    std::cout << b1 << std::endl;
    std::cout << yBar - b1*xBar << std::endl;
}

int main() {
  double xVals[] = {2+(1.0/3),4+(1.0/3),6+(1.0/3),8+(1.0/3),10+(1.0/3),12+(1.0/3),14+(1.0/3),16+(1.0/3),30+(1.0/3)};
  double yVals[] = {936,891,873,864,857,853,852,848,843};
  std::vector<double> x(xVals,xVals+sizeof xVals / sizeof xVals[0]);
  std::vector<double> y(yVals,yVals+sizeof yVals / sizeof yVals[0]);
  determineConstants(x,y);
  
}


