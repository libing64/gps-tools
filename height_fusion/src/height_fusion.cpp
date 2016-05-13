#include "height_fusion/height_fusion.h"
#include <Eigen/Eigen>
#include <iostream>

using namespace std;
using namespace Eigen;


Height_fusion::Height_fusion()
{
	P = Q;
	height_initialized = false;
}

Height_fusion::~Height_fusion()
{


}

void Height_fusion::predict(double t, double az)
{
	if(!height_initialized)
		return;


	if(t <= curr_time)
		return;

	height += az*(t - curr_time);
	curr_time = t;
	P = P + Q;

}

void Height_fusion::correct(double t, double h)
{
	if(!height_initialized)
	{
		curr_time = t;
		height = h;
		return;
	}

	double H = 1; 
    double K = P*H/(H*P*H + R);
    height += K*(h - height);
    P = (1 - K*H)*P;
   // cout << "P: " << P << endl;
}



