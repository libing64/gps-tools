#include <ros/ros.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include "height_fusion/height_fusion.h"
#include <deque>

using namespace std;
using namespace Eigen;

Height_fusion height_fusion;
deque<pair<double, double> > height_q;
deque<pair<double, double> > az_q;
ros::Publisher height_pub;

void heightCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  double h = msg->vector.z;
  double t = msg->header.stamp.toSec();
  //height_fusion.correct(h, t);
  height_q.push_back(make_pair(t, h));
}

void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  double t = msg->header.stamp.toSec();
  Quaterniond q;
  q.w() = msg->orientation.w;
  q.x() = msg->orientation.x;
  q.y() = msg->orientation.y;
  q.z() = msg->orientation.z;
 // cout << "q: " << q.w() << " " << q.vec().transpose() << endl;
  Vector3d acc;
  acc(0) = msg->linear_acceleration.x;
  acc(1) = msg->linear_acceleration.y;
  acc(2) = msg->linear_acceleration.z;

  Vector3d acc_g = q.toRotationMatrix().transpose()*acc - Vector3d(0, 0, 9.8);
  //cout << "acc_g: " << acc_g.transpose() << endl;
  az_q.push_back(make_pair(t, acc_g(2)));

  while(az_q.size() > 0)
  {
    if(height_q.empty())
    {
      height_fusion.predict(az_q.front().first, az_q.front().second);
      az_q.pop_front();
    }else if(az_q.front().first < height_q.front().first)
    {
      height_fusion.predict(az_q.front().first, az_q.front().second);
      az_q.pop_front();
    }else
    {
      height_fusion.correct(height_q.front().first, height_q.front().second);
      height_q.pop_front();
    }

  }
 
  geometry_msgs::Vector3Stamped height_msg;
  height_msg.header.stamp = msg->header.stamp;
  height_msg.vector.z = height_fusion.height;
  height_pub.publish(height_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "height_fusion");
  ros::NodeHandle n("~");

  ros::Subscriber height_sub = n.subscribe("height", 10, heightCallback);
  ros::Subscriber imu_sub = n.subscribe("imu", 10, imuCallback);
  height_pub = n.advertise<geometry_msgs::Vector3Stamped>("filtered_height", 10);
  ros::spin();

  return  0;
}

