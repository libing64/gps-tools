/*
 * node.cpp
 *
 *  This file is part of pressure_altimeter.
 *
 *	Created on: 24/08/2014
 */

#include <pressure_altimeter/node.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <cmath>

namespace galt {
namespace pressure_altimeter {

Node::Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  subPressure_ =
      pnh_.subscribe("pressure", 1, &Node::pressureCallback, this);
  pubHeight_ = pnh_.advertise<geometry_msgs::Vector3Stamped>("height", 1);

  pnh_.param("fixed_frame", worldFrameId_, std::string("world"));
}

void Node::pressureCallback(
    const sensor_msgs::FluidPressureConstPtr &pressure) {
  geometry_msgs::Vector3Stamped height_msg;
  height_msg.header.stamp = pressure->header.stamp;
  height_msg.header.frame_id = worldFrameId_;

  const double pressurePA = pressure->fluid_pressure * 100;  //  mb to Pa
  if (pressurePA > 0) {
    //  calculate height from barometer
    const double c = kG * kM / (kR * kL);
    const double lhs = std::exp(std::log(pressurePA / kP0) * (1 / c));
    const double h = (1 - lhs) * kT0 / kL;

    //  value of jacobian to propagate variance
    //const double J = -lhs * kT0 / (kL * c * pressurePA);
    //const double h_var = J * J * (pressure->variance * 100 * 100);  //  mb to Pa

    height_msg.vector.z  = h;

  } else {
    height_msg.vector.z = 0;
  }

  pubHeight_.publish(height_msg);
}

}  //  pressure_altimeter
}  //  galt
