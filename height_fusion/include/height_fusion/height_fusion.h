#ifndef __HEIGHT_FUSION_H
#define __HEIGHT_FUSION_H
#include <iostream>
#include <Eigen/Eigen>


using namespace std;
using namespace Eigen;
  class Height_fusion
  {
  public:
    Height_fusion();
    ~Height_fusion();

    double height;
    double curr_time;
    double R = 1.0;
    double Q = 0.01;
    double P;
    bool height_initialized;

    void predict(double t,  double az);//! in up is positive
    void correct(double t, double h_);


  };

#endif
