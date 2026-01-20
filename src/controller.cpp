/****************************************************************************
 *
 *   Copyright (c) 2023, SMART Research Group, Saxion University of 
 *   Applied Sciences.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/



#include "../include/px4_offboard_lowlevel/controller.h"
#include <fstream>
#include <iomanip>  // For setting the precision
#include <iostream>
#include <cmath>

namespace {
inline double clampd(double v, double lo, double hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

// MATLAB equivalent: rotm2eul(R, "YXZ")
// Assumes rotation matrix composition: R = Ry(theta) * Rx(phi) * Rz(psi)
// Returns [theta, phi, psi].
inline Eigen::Vector3d rotm2eul_YXZ(const Eigen::Matrix3d &R) {
    const double phi = -std::asin(clampd(R(1, 2), -1.0, 1.0));
    const double cphi = std::cos(phi);

    double theta = 0.0;
    double psi = 0.0;

    // Non-singular case
    if (std::abs(cphi) > 1e-9) {
        theta = std::atan2(R(0, 2), R(2, 2));
        psi = std::atan2(R(1, 0), R(1, 1));
    } else {
        // Gimbal lock: fall back to a reasonable decomposition
        // (theta and psi become coupled; pick theta = 0)
        theta = 0.0;
        psi = std::atan2(-R(0, 1), R(0, 0));
    }

    return Eigen::Vector3d(theta, phi, psi);
}
} // namespace

controller::controller(){
    zeta1 = 9.81;
    zeta2 = 0.0;

    position_W_.setZero();
    velocity_W_.setZero();
    R_B_W_.setIdentity();
    orientation_B_W_.setIdentity();
    angular_velocity_B_.setZero();

    position_W_NED_.setZero();
    velocity_W_NED_.setZero();
    R_B_W_NED_.setIdentity();
    orientation_B_W_NED_.setIdentity();
    angular_velocity_B_FRD_.setZero();
}

void controller::calculateFBLControllerOutout(
        Eigen::VectorXd *controller_torque_thrust) {
    assert(controller_torque_thrust);
    double thrust;
    // Attitude tracking.
    Eigen::Vector3d tau;

    controller_torque_thrust->resize(4);
   // Initialize variables with current states
   // Position (x, y, z)
   double x = position_W_(0);
   double y = position_W_(1);
   double z = position_W_(2);

   // Orientation (roll, pitch, yaw)
   double ps = std::atan2(R_B_W_(1,0), R_B_W_(0,0));

    // Pitch (θ, theta) - Rotation around Y-axis
    double th= std::asin(-R_B_W_(2,0));

    // Roll (ϕ, phi) - Rotation around X-axis
    double ph= std::atan2(R_B_W_(2,1), R_B_W_(2,2));

   // Linear velocities (vx, vy, vz)
   double vx = velocity_W_(0);
   double vy = velocity_W_(1);
   double vz = velocity_W_(2);

   // Angular velocities (p, q, r)
   double p = angular_velocity_B_(0);
   double q = angular_velocity_B_(1);
   double r = angular_velocity_B_(2);
    // Control gains
    double k1 = 200.0;
    double k2 = 400.0;
    double k3 = 90.0;
    double k4 = 30.0;
    double k5 = 100.0;
    double k6 = 300.0;
    double k7 = 60.0;
    double k8 = 30.0;
    double k9 = 10.0;
    double k10 = 40.0;
    double k11 = 30.0;
    double k12 = 30.0;
    double k13 = 10.0;
    double k14 = 12.0;

    // Physical constants
    double L = 0.2656;
    double Ix = 0.01152;
    double Iy = 0.01152;
    double Iz = 0.0218;
    double g = 9.81;
    double m = 1.923;

    // Desired states
    double des_radius = 5.0;
    double des_height = 5.0;
    double des_yaw = 0.0;
    double d = 1.0;

    // Distance variables
    double dist_x = 0.0;
    double dist_y = 0.0;
    double dist_z = 0.0;
    // Control gains
    double kt = 0.01;
    double kr = 0.01;

    double xi1_1 =-des_radius*des_radius+x*x+y*y;
    double xi1_2 =vx*x*2.0+vy*y*2.0;
    double xi1_3 =((dist_x*x+dist_y*y+m*(vx*vx)+m*(vy*vy)-kt*vx*x-kt*vy*y-y*zeta1*cos(ps)*sin(ph)+x*zeta1*sin(ph)*sin(ps)+x*zeta1*cos(ph)*cos(ps)*sin(th)+y*zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m;
    double xi1_4 = (zeta2*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th))*2.0)/m-1.0/(m*m)*(kt*x-m*vx*2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0+1.0/(m*m)*(m*vy*2.0-kt*y)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0+(vx*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m+(vy*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m+(zeta1*(r*cos(ph)+q*sin(ph))*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))-(zeta1*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*cos(ph)*cos(th)*(q*cos(ph)-r*sin(ph))*(x*cos(ps)+y*sin(ps))*2.0)/m;
    double Lg1Lf3S1 =((-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th))*2.0)/m;
    double Lg2Lf3S1 = (zeta1*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*-2.0)/(Ix*m);
    double Lg3Lf3S1 = ((zeta1*sin(ph)*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*pow(cos(ph),2.0)*cos(th)*(x*cos(ps)+y*sin(ps))*2.0)/m-(zeta1*sin(ph)*sin(th)*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th)))/Iy;
    double Lg4Lf3S1 = -((zeta1*cos(ph)*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*-2.0)/(m*cos(th))+(zeta1*cos(ph)*sin(th)*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*cos(ph)*cos(th)*sin(ph)*(x*cos(ps)+y*sin(ps))*2.0)/m)/Iz;
    double Lf4S1 =   (q*cos(ph)-r*sin(ph))*((zeta2*(x*cos(ph)*cos(ps)*cos(th)+y*cos(ph)*cos(th)*sin(ps))*2.0)/m-(zeta1*(x*cos(ps)*cos(th)*sin(ph)+y*cos(th)*sin(ph)*sin(ps))*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*2.0)/(m*cos(th))-(zeta1*(-p*sin(th)+r*cos(ph)*cos(th)+q*cos(th)*sin(ph))*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*(y*cos(ph)*cos(ps)*cos(th)-x*cos(ph)*cos(th)*sin(ps))*(r*cos(ph)+q*sin(ph))*2.0)/(m*cos(th))-1.0/(m*m)*zeta1*cos(ph)*cos(ps)*cos(th)*(kt*x-m*vx*2.0)*2.0+1.0/(m*m)*zeta1*cos(ph)*cos(th)*sin(ps)*(m*vy*2.0-kt*y)*2.0-(zeta1*1.0/pow(cos(th),2.0)*sin(th)*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/m+(vx*zeta1*cos(ph)*cos(ps)*cos(th)*2.0)/m+(vy*zeta1*cos(ph)*cos(th)*sin(ps)*2.0)/m-(zeta1*cos(ph)*sin(th)*(q*cos(ph)-r*sin(ph))*(x*cos(ps)+y*sin(ps))*2.0)/m+(zeta1*1.0/pow(cos(th),2.0)*sin(th)*(r*cos(ph)+q*sin(ph))*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/m)-(p+(r*cos(ph)*sin(th))/cos(th)+(q*sin(ph)*sin(th))/cos(th))*((vx*(zeta1*cos(ph)*sin(ps)-zeta1*cos(ps)*sin(ph)*sin(th))*-2.0)/m+(vy*(zeta1*cos(ph)*cos(ps)+zeta1*sin(ph)*sin(ps)*sin(th))*2.0)/m+(zeta2*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/m+1.0/(m*m)*(zeta1*cos(ph)*sin(ps)-zeta1*cos(ps)*sin(ph)*sin(th))*(kt*x-m*vx*2.0)*2.0+1.0/(m*m)*(zeta1*cos(ph)*cos(ps)+zeta1*sin(ph)*sin(ps)*sin(th))*(m*vy*2.0-kt*y)*2.0-(zeta1*(r*cos(ph)+q*sin(ph))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))-(zeta1*(q*cos(ph)-r*sin(ph))*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*(q*cos(ph)*sin(th)-r*sin(ph)*sin(th))*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*cos(ph)*cos(th)*(r*cos(ph)+q*sin(ph))*(x*cos(ps)+y*sin(ps))*2.0)/m+(zeta1*cos(th)*sin(ph)*(q*cos(ph)-r*sin(ph))*(x*cos(ps)+y*sin(ps))*2.0)/m)-zeta2*((vx*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*-2.0)/m+(vy*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*2.0)/m+1.0/(m*m)*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*(kt*x-m*vx*2.0)*2.0+1.0/(m*m)*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*(m*vy*2.0-kt*y)*2.0+((p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))-((r*cos(ph)+q*sin(ph))*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))-(cos(ph)*cos(th)*(q*cos(ph)-r*sin(ph))*(x*cos(ps)+y*sin(ps))*2.0)/m)+((r*cos(ph))/cos(th)+(q*sin(ph))/cos(th))*((vx*(zeta1*cos(ps)*sin(ph)-zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m+(vy*(zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m+(zeta2*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/m-1.0/(m*m)*(zeta1*cos(ps)*sin(ph)-zeta1*cos(ph)*sin(ps)*sin(th))*(kt*x-m*vx*2.0)*2.0+1.0/(m*m)*(zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*(m*vy*2.0-kt*y)*2.0-(zeta1*(r*cos(ph)+q*sin(ph))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*cos(ph)*cos(th)*(q*cos(ph)-r*sin(ph))*(y*cos(ps)-x*sin(ps))*2.0)/m)+vx*((zeta2*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*2.0)/m-kt*1.0/(m*m)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0+(zeta1*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*(r*cos(ph)+q*sin(ph))*2.0)/(m*cos(th))+(zeta1*(cos(ph)*sin(ps)-cos(ps)*sin(ph)*sin(th))*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*2.0)/(m*cos(th))+(zeta1*cos(ph)*cos(ps)*cos(th)*(q*cos(ph)-r*sin(ph))*2.0)/m)-vy*((zeta2*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*2.0)/m+kt*1.0/(m*m)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0-(zeta1*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*(r*cos(ph)+q*sin(ph))*2.0)/(m*cos(th))+(zeta1*(cos(ph)*cos(ps)+sin(ph)*sin(ps)*sin(th))*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*2.0)/(m*cos(th))-(zeta1*cos(ph)*cos(th)*sin(ps)*(q*cos(ph)-r*sin(ph))*2.0)/m)+(dist_x/m-(kt*vx)/m+(zeta1*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th)))/m)*(((dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*6.0)/m-(kt*vx*2.0)/m+kt*1.0/(m*m)*(kt*x-m*vx*2.0)*2.0)+(-dist_y/m+(kt*vy)/m+(zeta1*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th)))/m)*(((dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*-6.0)/m+(kt*vy*2.0)/m+kt*1.0/(m*m)*(m*vy*2.0-kt*y)*2.0)+(((zeta1*cos(ph)*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*-2.0)/(m*cos(th))+(zeta1*cos(ph)*sin(th)*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*cos(ph)*cos(th)*sin(ph)*(x*cos(ps)+y*sin(ps))*2.0)/m)*(kr*r-Ix*p*q+Iy*p*q))/Iz-(((zeta1*sin(ph)*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(zeta1*pow(cos(ph),2.0)*cos(th)*(x*cos(ps)+y*sin(ps))*2.0)/m-(zeta1*sin(ph)*sin(th)*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th)))*(kr*q+Ix*p*r-Iz*p*r))/Iy+(zeta1*(kr*p-Iy*q*r+Iz*q*r)*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th))*2.0)/(Ix*m);
    double xi2_1 =   -des_height+z;
    double xi2_2 =  vz;
    double xi2_3 =  (dist_z-g*m-kt*vz+zeta1*cos(ph)*cos(th))/m;
    double xi2_4 =  -1.0/(m*m)*(dist_z*kt-(kt*kt)*vz-g*kt*m+m*q*zeta1*sin(th)+kt*zeta1*cos(ph)*cos(th)-m*zeta2*cos(ph)*cos(th)+m*p*zeta1*cos(th)*sin(ph));
    double Lg1Lf3S2 = (cos(ph)*cos(th))/m;
    double Lg2Lf3S2 = -(zeta1*cos(th)*sin(ph))/(Ix*m);
    double Lg3Lf3S2 = -(zeta1*sin(th))/(Iy*m);
    double Lg4Lf3S2 = 0.0;
    double Lf4S2 =   -1.0/(m*m)*(p+(r*cos(ph)*sin(th))/cos(th)+(q*sin(ph)*sin(th))/cos(th))*(-kt*zeta1*cos(th)*sin(ph)+m*zeta2*cos(th)*sin(ph)+m*p*zeta1*cos(ph)*cos(th))+1.0/(m*m)*(q*cos(ph)-r*sin(ph))*(kt*zeta1*cos(ph)*sin(th)-m*zeta2*cos(ph)*sin(th)-m*q*zeta1*cos(th)+m*p*zeta1*sin(ph)*sin(th))-1.0/(m*m)*zeta2*(m*q*sin(th)+kt*cos(ph)*cos(th)+m*p*cos(th)*sin(ph))-(kt*kt)*1.0/(m*m)*(g-dist_z/m+(kt*vz)/m-(zeta1*cos(ph)*cos(th))/m)+(zeta1*sin(th)*(kr*q+Ix*p*r-Iz*p*r))/(Iy*m)+(zeta1*cos(th)*sin(ph)*(kr*p-Iy*q*r+Iz*q*r))/(Ix*m);
    double eta1_1 =   atan(y/x);
    double eta1_2 = (vy*x-vx*y)/(x*x+y*y);
    double eta1_3 = vx*1.0/pow(x*x+y*y,2.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)-vy*1.0/pow(x*x+y*y,2.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)+(x*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th)))/(m*(x*x+y*y))-(y*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th)))/(m*(x*x+y*y));
    double eta1_4 = -vx*(vx*1.0/pow(x*x+y*y,2.0)*(vy*x*2.0-vx*y*2.0)+vy*1.0/pow(x*x+y*y,2.0)*(vx*x*2.0+vy*y*2.0)-(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))/(m*(x*x+y*y))+vx*x*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0-vy*x*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0+((x*x)*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m-(x*y*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m)-vy*(-vx*1.0/pow(x*x+y*y,2.0)*(vx*x*2.0+vy*y*2.0)+vy*1.0/pow(x*x+y*y,2.0)*(vy*x*2.0-vx*y*2.0)+(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))/(m*(x*x+y*y))+vx*y*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0-vy*y*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0-((y*y)*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m+(x*y*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m)-1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)-(zeta2*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th)))/(m*(x*x+y*y))+(zeta1*(r*cos(ph)+q*sin(ph))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))-(zeta1*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))-(zeta1*cos(ph)*cos(th)*(q*cos(ph)-r*sin(ph))*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y));
    double Lg1Lf3P1 = -(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))/(m*(x*x+y*y));
    double Lg2Lf3P1 = -(zeta1*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(Ix*m*(x*x+y*y));
    double Lg3Lf3P1 = -(-(zeta1*sin(ph)*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*pow(cos(ph),2.0)*cos(th)*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y))+(zeta1*sin(ph)*sin(th)*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y)))/Iy;
    double Lg4Lf3P1 = ((zeta1*cos(ph)*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*cos(ph)*cos(th)*sin(ph)*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y))-(zeta1*cos(ph)*sin(th)*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y)))/Iz;
    double Lf4P1 =   ((r*cos(ph))/cos(th)+(q*sin(ph))/cos(th))*(vx*((zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))/(m*(x*x+y*y))-((x*x)*1.0/pow(x*x+y*y,2.0)*(zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m+(x*y*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ps)*sin(ph)-zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m)-vy*((zeta1*cos(ps)*sin(ph)-zeta1*cos(ph)*sin(ps)*sin(th))/(m*(x*x+y*y))-((y*y)*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ps)*sin(ph)-zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m+(x*y*1.0/pow(x*x+y*y,2.0)*(zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m)+(zeta2*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*(x*x+y*y))-1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ps)*sin(ph)-zeta1*cos(ph)*sin(ps)*sin(th))*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)+(zeta1*(r*cos(ph)+q*sin(ph))*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))-(zeta1*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*cos(ph)*cos(th)*(q*cos(ph)-r*sin(ph))*(x*cos(ps)+y*sin(ps)))/(m*(x*x+y*y)))-(dist_x/m-(kt*vx)/m+(zeta1*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th)))/m)*(-vy*(1.0/pow(x*x+y*y,2.0)*(vx*x*2.0+vy*y*2.0)+kt/(m*(x*x+y*y))+vx*x*1.0/pow(x*x+y*y,2.0)*2.0+vy*y*1.0/pow(x*x+y*y,2.0)*2.0-y*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0+vy*y*1.0/pow(x*x+y*y,3.0)*(x*x-y*y)*4.0-vx*x*(y*y)*1.0/pow(x*x+y*y,3.0)*8.0-(kt*(y*y)*1.0/pow(x*x+y*y,2.0)*2.0)/m)+vx*(1.0/pow(x*x+y*y,2.0)*(vy*x*2.0-vx*y*2.0)+vy*x*1.0/pow(x*x+y*y,2.0)*2.0-vx*y*1.0/pow(x*x+y*y,2.0)*2.0+x*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0-vy*x*1.0/pow(x*x+y*y,3.0)*(x*x-y*y)*4.0+vx*(x*x)*y*1.0/pow(x*x+y*y,3.0)*8.0+(kt*x*y*1.0/pow(x*x+y*y,2.0)*2.0)/m)+vx*1.0/pow(x*x+y*y,2.0)*(vy*x*2.0-vx*y*2.0)+vy*1.0/pow(x*x+y*y,2.0)*(vx*x*2.0+vy*y*2.0)-(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))/(m*(x*x+y*y))+vx*x*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0-vy*x*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(m*(x*x)*2.0-m*(y*y)*2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))+((x*x)*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m+kt*1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)-(x*y*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*6.0)/m)+(-dist_y/m+(kt*vy)/m+(zeta1*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th)))/m)*(vx*(1.0/pow(x*x+y*y,2.0)*(vx*x*2.0+vy*y*2.0)+kt/(m*(x*x+y*y))+vx*x*1.0/pow(x*x+y*y,2.0)*2.0+vy*y*1.0/pow(x*x+y*y,2.0)*2.0-x*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0-vx*x*1.0/pow(x*x+y*y,3.0)*(x*x-y*y)*4.0-vy*(x*x)*y*1.0/pow(x*x+y*y,3.0)*8.0-(kt*(x*x)*1.0/pow(x*x+y*y,2.0)*2.0)/m)-vy*(-1.0/pow(x*x+y*y,2.0)*(vy*x*2.0-vx*y*2.0)-vy*x*1.0/pow(x*x+y*y,2.0)*2.0+vx*y*1.0/pow(x*x+y*y,2.0)*2.0+y*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0+vx*y*1.0/pow(x*x+y*y,3.0)*(x*x-y*y)*4.0+vy*x*(y*y)*1.0/pow(x*x+y*y,3.0)*8.0+(kt*x*y*1.0/pow(x*x+y*y,2.0)*2.0)/m)-vx*1.0/pow(x*x+y*y,2.0)*(vx*x*2.0+vy*y*2.0)+vy*1.0/pow(x*x+y*y,2.0)*(vy*x*2.0-vx*y*2.0)+(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))/(m*(x*x+y*y))+vx*y*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0-vy*y*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(m*(x*x)*2.0-m*(y*y)*2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))-((y*y)*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m-kt*1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)+(x*y*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*6.0)/m)+(p+(r*cos(ph)*sin(th))/cos(th)+(q*sin(ph)*sin(th))/cos(th))*(vx*(-(zeta1*cos(ph)*cos(ps)+zeta1*sin(ph)*sin(ps)*sin(th))/(m*(x*x+y*y))+((x*x)*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ph)*cos(ps)+zeta1*sin(ph)*sin(ps)*sin(th))*2.0)/m+(x*y*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ph)*sin(ps)-zeta1*cos(ps)*sin(ph)*sin(th))*2.0)/m)+vy*(-(zeta1*cos(ph)*sin(ps)-zeta1*cos(ps)*sin(ph)*sin(th))/(m*(x*x+y*y))+((y*y)*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ph)*sin(ps)-zeta1*cos(ps)*sin(ph)*sin(th))*2.0)/m+(x*y*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ph)*cos(ps)+zeta1*sin(ph)*sin(ps)*sin(th))*2.0)/m)-(zeta2*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*(x*x+y*y))+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ph)*cos(ps)+zeta1*sin(ph)*sin(ps)*sin(th))*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(zeta1*cos(ph)*sin(ps)-zeta1*cos(ps)*sin(ph)*sin(th))*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)-(zeta1*(q*cos(ph)*sin(th)-r*sin(ph)*sin(th))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))-(zeta1*(r*cos(ph)+q*sin(ph))*(y*cos(ph)*cos(ps)-x*cos(ph)*sin(ps)+x*cos(ps)*sin(ph)*sin(th)+y*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*(q*cos(ph)-r*sin(ph))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*cos(ph)*cos(th)*(r*cos(ph)+q*sin(ph))*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y))+(zeta1*cos(th)*sin(ph)*(q*cos(ph)-r*sin(ph))*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y)))+(q*cos(ph)-r*sin(ph))*(-vy*((zeta1*cos(ph)*cos(ps)*cos(th))/(m*(x*x+y*y))-((y*y)*zeta1*cos(ph)*cos(ps)*cos(th)*1.0/pow(x*x+y*y,2.0)*2.0)/m+(x*y*zeta1*cos(ph)*cos(th)*sin(ps)*1.0/pow(x*x+y*y,2.0)*2.0)/m)+vx*((zeta1*cos(ph)*cos(th)*sin(ps))/(m*(x*x+y*y))-((x*x)*zeta1*cos(ph)*cos(th)*sin(ps)*1.0/pow(x*x+y*y,2.0)*2.0)/m+(x*y*zeta1*cos(ph)*cos(ps)*cos(th)*1.0/pow(x*x+y*y,2.0)*2.0)/m)-(zeta2*(y*cos(ph)*cos(ps)*cos(th)-x*cos(ph)*cos(th)*sin(ps)))/(m*(x*x+y*y))+(zeta1*(x*cos(ph)*cos(ps)*cos(th)+y*cos(ph)*cos(th)*sin(ps))*(r*cos(ph)+q*sin(ph)))/(m*cos(th)*(x*x+y*y))+(zeta1*(y*cos(ps)*cos(th)*sin(ph)-x*cos(th)*sin(ph)*sin(ps))*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th)))/(m*cos(th)*(x*x+y*y))-(zeta1*(-p*sin(th)+r*cos(ph)*cos(th)+q*cos(th)*sin(ph))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+1.0/(m*m)*zeta1*cos(ph)*cos(ps)*cos(th)*1.0/pow(x*x+y*y,2.0)*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)-1.0/(m*m)*zeta1*cos(ph)*cos(th)*sin(ps)*1.0/pow(x*x+y*y,2.0)*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)+(zeta1*cos(ph)*sin(th)*(q*cos(ph)-r*sin(ph))*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y))+(zeta1*1.0/pow(cos(th),2.0)*sin(th)*(r*cos(ph)+q*sin(ph))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*(x*x+y*y))-(zeta1*1.0/pow(cos(th),2.0)*sin(th)*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*(x*x+y*y)))+vy*(vx*((vx*vx)*1.0/pow(x*x+y*y,2.0)*2.0-(vy*vy)*1.0/pow(x*x+y*y,2.0)*2.0+(x*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m-(y*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m-vx*x*1.0/pow(x*x+y*y,3.0)*(vx*x*2.0+vy*y*2.0)*4.0+vy*x*1.0/pow(x*x+y*y,3.0)*(vy*x*2.0-vx*y*2.0)*4.0+vx*y*1.0/pow(x*x+y*y,3.0)*(vy*x*2.0-vx*y*2.0)*4.0+vy*y*1.0/pow(x*x+y*y,3.0)*(vx*x*2.0+vy*y*2.0)*4.0+vx*x*y*1.0/pow(x*x+y*y,4.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*2.4E+1-vy*x*y*1.0/pow(x*x+y*y,4.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*2.4E+1-(x*(y*y)*1.0/pow(x*x+y*y,3.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*8.0)/m+((x*x)*y*1.0/pow(x*x+y*y,3.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*8.0)/m)+vy*(vx*vy*1.0/pow(x*x+y*y,2.0)*4.0-vx*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0+vy*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0-(x*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m+(y*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*6.0)/m-((y*y*y)*1.0/pow(x*x+y*y,3.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*8.0)/m+vx*(y*y)*1.0/pow(x*x+y*y,4.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*2.4E+1-vy*(y*y)*1.0/pow(x*x+y*y,4.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*2.4E+1-vx*y*1.0/pow(x*x+y*y,3.0)*(vx*x*2.0+vy*y*2.0)*8.0+vy*y*1.0/pow(x*x+y*y,3.0)*(vy*x*2.0-vx*y*2.0)*8.0+(x*(y*y)*1.0/pow(x*x+y*y,3.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*8.0)/m)-1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(m*vy*x*4.0+kt*x*y*2.0-m*vx*y*4.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))-(zeta2*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th)))/(m*(x*x+y*y))+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*(kt*(x*x)+kt*(y*y)*3.0+m*vx*x*4.0+m*vy*y*4.0)+1.0/(m*m)*y*1.0/pow(x*x+y*y,3.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)*4.0-1.0/(m*m)*y*1.0/pow(x*x+y*y,3.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)*4.0+(y*zeta2*1.0/pow(x*x+y*y,2.0)*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/m-(zeta1*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*(r*cos(ph)+q*sin(ph)))/(m*cos(th)*(x*x+y*y))-(zeta1*(cos(ph)*sin(ps)-cos(ps)*sin(ph)*sin(th))*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th)))/(m*cos(th)*(x*x+y*y))-(zeta1*cos(ph)*cos(ps)*cos(th)*(q*cos(ph)-r*sin(ph)))/(m*(x*x+y*y))+(y*zeta1*1.0/pow(x*x+y*y,2.0)*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))-(y*zeta1*1.0/pow(x*x+y*y,2.0)*(r*cos(ph)+q*sin(ph))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(y*zeta1*cos(ph)*cos(th)*1.0/pow(x*x+y*y,2.0)*(q*cos(ph)-r*sin(ph))*(y*cos(ps)-x*sin(ps))*2.0)/m)+vx*(vy*((vx*vx)*1.0/pow(x*x+y*y,2.0)*2.0-(vy*vy)*1.0/pow(x*x+y*y,2.0)*2.0+(x*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m-(y*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*2.0)/m-vx*x*1.0/pow(x*x+y*y,3.0)*(vx*x*2.0+vy*y*2.0)*4.0+vy*x*1.0/pow(x*x+y*y,3.0)*(vy*x*2.0-vx*y*2.0)*4.0+vx*y*1.0/pow(x*x+y*y,3.0)*(vy*x*2.0-vx*y*2.0)*4.0+vy*y*1.0/pow(x*x+y*y,3.0)*(vx*x*2.0+vy*y*2.0)*4.0+vx*x*y*1.0/pow(x*x+y*y,4.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*2.4E+1-vy*x*y*1.0/pow(x*x+y*y,4.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*2.4E+1-(x*(y*y)*1.0/pow(x*x+y*y,3.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*8.0)/m+((x*x)*y*1.0/pow(x*x+y*y,3.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*8.0)/m)+vx*(vx*vy*1.0/pow(x*x+y*y,2.0)*-4.0-vx*1.0/pow(x*x+y*y,3.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*4.0+vy*1.0/pow(x*x+y*y,3.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*4.0-(x*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*6.0)/m+(y*1.0/pow(x*x+y*y,2.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*2.0)/m+((x*x*x)*1.0/pow(x*x+y*y,3.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*8.0)/m+vx*(x*x)*1.0/pow(x*x+y*y,4.0)*(-vy*(x*x)+vy*(y*y)+vx*x*y*2.0)*2.4E+1-vy*(x*x)*1.0/pow(x*x+y*y,4.0)*(vx*(x*x)-vx*(y*y)+vy*x*y*2.0)*2.4E+1+vx*x*1.0/pow(x*x+y*y,3.0)*(vy*x*2.0-vx*y*2.0)*8.0+vy*x*1.0/pow(x*x+y*y,3.0)*(vx*x*2.0+vy*y*2.0)*8.0-((x*x)*y*1.0/pow(x*x+y*y,3.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*8.0)/m)+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(m*vy*x*-4.0+kt*x*y*2.0+m*vx*y*4.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))-(zeta2*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th)))/(m*(x*x+y*y))-1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*(kt*(x*x)*3.0+kt*(y*y)+m*vx*x*4.0+m*vy*y*4.0)+1.0/(m*m)*x*1.0/pow(x*x+y*y,3.0)*(dist_y-kt*vy-zeta1*cos(ps)*sin(ph)+zeta1*cos(ph)*sin(ps)*sin(th))*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)*4.0-1.0/(m*m)*x*1.0/pow(x*x+y*y,3.0)*(dist_x-kt*vx+zeta1*sin(ph)*sin(ps)+zeta1*cos(ph)*cos(ps)*sin(th))*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)*4.0+(x*zeta2*1.0/pow(x*x+y*y,2.0)*(x*cos(ps)*sin(ph)+y*sin(ph)*sin(ps)+y*cos(ph)*cos(ps)*sin(th)-x*cos(ph)*sin(ps)*sin(th))*2.0)/m+(zeta1*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*(r*cos(ph)+q*sin(ph)))/(m*cos(th)*(x*x+y*y))-(zeta1*(cos(ph)*cos(ps)+sin(ph)*sin(ps)*sin(th))*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*cos(ph)*cos(th)*sin(ps)*(q*cos(ph)-r*sin(ph)))/(m*(x*x+y*y))+(x*zeta1*1.0/pow(x*x+y*y,2.0)*(p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))-(x*zeta1*1.0/pow(x*x+y*y,2.0)*(r*cos(ph)+q*sin(ph))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th))*2.0)/(m*cos(th))+(x*zeta1*cos(ph)*cos(th)*1.0/pow(x*x+y*y,2.0)*(q*cos(ph)-r*sin(ph))*(y*cos(ps)-x*sin(ps))*2.0)/m)+zeta2*(vx*(-(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))/(m*(x*x+y*y))+((x*x)*1.0/pow(x*x+y*y,2.0)*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*2.0)/m+(x*y*1.0/pow(x*x+y*y,2.0)*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*2.0)/m)+vy*(-(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))/(m*(x*x+y*y))+((y*y)*1.0/pow(x*x+y*y,2.0)*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*2.0)/m+(x*y*1.0/pow(x*x+y*y,2.0)*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*2.0)/m)+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(cos(ps)*sin(ph)-cos(ph)*sin(ps)*sin(th))*(kt*(x*x*x)+m*vx*(x*x)*2.0+kt*x*(y*y)-m*vx*(y*y)*2.0+m*vy*x*y*4.0)+1.0/(m*m)*1.0/pow(x*x+y*y,2.0)*(sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th))*(kt*(y*y*y)-m*vy*(x*x)*2.0+kt*(x*x)*y+m*vy*(y*y)*2.0+m*vx*x*y*4.0)-((p*cos(th)+r*cos(ph)*sin(th)+q*sin(ph)*sin(th))*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+((r*cos(ph)+q*sin(ph))*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))-(cos(ph)*cos(th)*(q*cos(ph)-r*sin(ph))*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y)))+((-(zeta1*sin(ph)*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*pow(cos(ph),2.0)*cos(th)*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y))+(zeta1*sin(ph)*sin(th)*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y)))*(kr*q+Ix*p*r-Iz*p*r))/Iy-((kr*r-Ix*p*q+Iy*p*q)*((zeta1*cos(ph)*(-y*cos(ps)*sin(ph)+x*sin(ph)*sin(ps)+x*cos(ph)*cos(ps)*sin(th)+y*cos(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))+(zeta1*cos(ph)*cos(th)*sin(ph)*(y*cos(ps)-x*sin(ps)))/(m*(x*x+y*y))-(zeta1*cos(ph)*sin(th)*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(m*cos(th)*(x*x+y*y))))/Iz+(zeta1*(kr*p-Iy*q*r+Iz*q*r)*(x*cos(ph)*cos(ps)+y*cos(ph)*sin(ps)-y*cos(ps)*sin(ph)*sin(th)+x*sin(ph)*sin(ps)*sin(th)))/(Ix*m*(x*x+y*y));
    double eta2_1 =   -des_yaw+ps;
    double eta2_2 =   (r*cos(ph)+q*sin(ph))/cos(th);
    double Lg1LfP2 =  0.0;
    double Lg2LfP2 =  0.0;
    double Lg3LfP2 = sin(ph)/(Iy*cos(th));
    double Lg4LfP2 = cos(ph)/(Iz*cos(th));
    double Lf2P2 =((q*cos(ph)-r*sin(ph))*(p+(r*cos(ph)*sin(th))/cos(th)+(q*sin(ph)*sin(th))/cos(th)))/cos(th)-(cos(ph)*(kr*r-Ix*p*q+Iy*p*q))/(Iz*cos(th))-(sin(ph)*(kr*q+Ix*p*r-Iz*p*r))/(Iy*cos(th))+1.0/pow(cos(th),2.0)*sin(th)*(r*cos(ph)+q*sin(ph))*(q*cos(ph)-r*sin(ph));

    // Define the D matrix
    Eigen::Matrix4d D;
    D << Lg1Lf3S1, Lg2Lf3S1, Lg3Lf3S1, Lg4Lf3S1,
        Lg1Lf3S2, Lg2Lf3S2, Lg3Lf3S2, Lg4Lf3S2,
        Lg1Lf3P1, Lg2Lf3P1, Lg3Lf3P1, Lg4Lf3P1,
        Lg1LfP2, Lg2LfP2, Lg3LfP2, Lg4LfP2;

    // Compute inverse and determinant of D
    Eigen::Matrix4d inv_D = D.inverse();
    double det_d = D.determinant();

    // Calculate v1_tran, v2_tran, v1_tang, and v2_tang
    double v1_tran = -Lf4S1 - k1 * xi1_1 - k2 * xi1_2 - k3 * xi1_3 - k4 * xi1_4;
    double v2_tran = -Lf4S2 - k5 * xi2_1 - k6 * xi2_2 - k7 * xi2_3 - k8 * xi2_4;

    double v1_tang = -Lf4P1 - k10 * (eta1_2 + 0.3) - k11 * eta1_3 - k12 * eta1_4;
    //double v1_tang = -Lf4P1- k9 * (eta1_1 ) - k10 * (eta1_2) - k11 * eta1_3 - k12 * eta1_4;
    double v2_tang = -Lf2P2 - k13 * (eta2_1 - 0*M_PI / 4) - k14 * eta2_2;


   // double v1_tang = -Lf4P1- k9 * (eta1_1 - M_PI/4) - k10 * (eta1_2) - k11 * eta1_3 - k12 * eta1_4;


    // Compute control inputs
    Eigen:: Vector4d v;
    v<<v1_tran, v2_tran, v1_tang, v2_tang;
    Eigen::Vector4d u = inv_D * v;

    double u_new = u(0);
    tau(0) = u(1);
    tau(1) = u(2);
    tau(2) = u(3);
    double dt = 0.01; 
    // Update zeta
   
    zeta2=zeta2+(dt*u_new);
    zeta1=zeta1+ (dt*zeta2);
    thrust=zeta1;
    // Output the wrench
    *controller_torque_thrust << tau, thrust;
    std::ofstream file("controller_dataFBL.txt", std::ios::app);

    if (file.is_open()) {


        // Save data with high precision
        file << std::fixed << std::setprecision(6);

        // Save the current state to the file
        file << xi1_1 << "," << xi1_2 << "," << xi1_3 << "," << xi1_4 << ","
                << xi2_1 << "," << xi2_2 << "," << xi2_3 << "," << xi2_4 << ","
                << eta1_1 << "," << eta1_2 << "," << eta1_3 << "," << eta1_4 << ","
                << eta2_1 << "," << eta2_2 << "," << thrust << "," << tau << "\n";
        file.close();
    }


   /*std::ofstream file("controller_data_FBL.txt", std::ios::app);

    if (file.is_open()) {
        ps = ps * 180.0 / M_PI;
        th = th * 180.0 / M_PI;
        ph = ph * 180.0 / M_PI;


        // Save data with high precision
        file << std::fixed << std::setprecision(6);

        // Save the current state to the file
        file << "R_B_W_:\n" << R_B_W_ << "\n";
        file << "position_W_: " << position_W_.transpose() << "\n";
        file << "velocity_B_: " << (R_B_W_.transpose() * velocity_W_).transpose() << "\n";
        file << "angular_velocity_B_: " << angular_velocity_B_.transpose() << "\n";
        file << "Euler Angles (ph, th, ps): " << ph << " " << th << " " << ps << "\n";
        file << "tau: " << tau.transpose() << "\n";
        file << "thrust: " << thrust << "\n";
        file<<"u_new:  "<<u_new<<"\n\n";

        file.close();
    } else {
        std::cerr << "Unable to open file for writing!" << std::endl;
    }*/
}

void controller::calculateControllerOutput(
        Eigen::VectorXd *controller_torque_thrust, Eigen::Quaterniond *desired_quaternion) {
    assert(controller_torque_thrust);
    
    controller_torque_thrust->resize(4);
    // Geometric controller based on: 
    // T. Lee, M. Leok and N. H. McClamroch, "Geometric tracking control of a quadrotor UAV on SE(3),
    // " 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, USA, 2010.


    // Trajectory tracking.
    double thrust;
    Eigen::Matrix3d R_d_w;

    // Compute translational tracking errors.
    const Eigen::Vector3d e_p =
                position_W_ - r_position_W_;
    
    const Eigen::Vector3d e_v = 
                velocity_W_ - r_velocity_W_;
    
    const Eigen::Vector3d I_a_d = -position_gain_.cwiseProduct(e_p)
                                -velocity_gain_.cwiseProduct(e_v)
                                +_uav_mass * _gravity * Eigen::Vector3d::UnitZ() + _uav_mass * r_acceleration_W_;
    thrust = I_a_d.dot(R_B_W_.col(2));
    Eigen::Vector3d B_z_d;
    B_z_d = I_a_d;
    B_z_d.normalize();

    // Calculate Desired Rotational Matrix
    const Eigen::Vector3d B_x_d(std::cos(r_yaw), std::sin(r_yaw), 0.0);
    Eigen::Vector3d B_y_d = B_z_d.cross(B_x_d);
    B_y_d.normalize();
    R_d_w.col(0) = B_y_d.cross(B_z_d);
    R_d_w.col(1) = B_y_d;
    R_d_w.col(2) = B_z_d;
    
    Eigen::Quaterniond q_temp(R_d_w);
    *desired_quaternion = q_temp;
    
    // Attitude tracking.
    Eigen::Vector3d tau;

    const Eigen::Matrix3d e_R_matrix =
            0.5 * (R_d_w.transpose() * R_B_W_ - R_B_W_.transpose() * R_d_w)   ;
    Eigen::Vector3d e_R;
    e_R << e_R_matrix(2, 1), e_R_matrix(0, 2), e_R_matrix(1, 0);
    const Eigen::Vector3d omega_ref =
            r_yaw_rate * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d e_omega = angular_velocity_B_ - R_B_W_.transpose() * R_d_w * omega_ref;
    tau = -attitude_gain_.cwiseProduct(e_R)
           - angular_rate_gain_.cwiseProduct(e_omega)
           + angular_velocity_B_.cross(_inertia_matrix.asDiagonal() * angular_velocity_B_);

    // Output the wrench
    *controller_torque_thrust << tau, thrust;
}

void controller::calculateQuasiControllerOutput(Eigen::VectorXd *controller_torque_thrust) {
    assert(controller_torque_thrust);
    controller_torque_thrust->resize(4);

    // Frame alignment note:
    // MATLAB uses p,v in NED world frame, OM in body frame, R as body->world.
    // Use raw PX4 states (world NED, body FRD) provided via setOdometryPX4Raw().
    const Eigen::Vector3d p_W = position_W_NED_;
    const Eigen::Vector3d v_W = velocity_W_NED_;
    const Eigen::Quaterniond q_BW_NED = orientation_B_W_NED_;
    const Eigen::Matrix3d R_BW_NED = R_B_W_NED_; // body(FRD)->world(NED)
    const Eigen::Vector3d omega_B_FRD = angular_velocity_B_FRD_;

    // Local variables matching MATLAB names
    const Eigen::Vector3d p = p_W;
    const Eigen::Vector3d v = v_W;
    const Eigen::Matrix3d R = R_BW_NED;
    const Eigen::Vector3d OM = omega_B_FRD;

    // === MATLAB snippet equivalents ===
    // r11 = R(1,1); ... (MATLAB is 1-based; C++ is 0-based)
    const double r11 = R(0,0);
    const double r12 = R(0,1);
    const double r13 = R(0,2);
    const double r21 = R(1,0);
    const double r22 = R(1,1);
    const double r23 = R(1,2);
    const double r31 = R(2,0);
    const double r32 = R(2,1);
    const double r33 = R(2,2);

    // Euler YXZ (theta, phi, psi) like rotm2eul(R,"YXZ"), computed from quaternion.
    const double qw = q_BW_NED.w();
    const double qx = q_BW_NED.x();
    const double qy = q_BW_NED.y();
    const double qz = q_BW_NED.z();

    // Only the needed rotation matrix terms (from quaternion) to match the existing YXZ extraction.
    const double m02 = 2.0 * (qx * qz + qw * qy);
    const double m12 = 2.0 * (qy * qz - qw * qx);
    const double m10 = 2.0 * (qx * qy + qw * qz);
    const double m11 = 1.0 - 2.0 * (qx * qx + qz * qz);

    const double sx = -m12;
    const double phi = std::asin(clampd(sx, -1.0, 1.0));
    const double cx = std::cos(phi);
    const double sy = (std::abs(cx) > 1e-6) ? (m02 / cx) : 0.0;
    const double theta = std::asin(clampd(sy, -1.0, 1.0));
    const double psi = std::atan2(m10, m11);
    const Eigen::Vector3d eta(theta, phi, psi);

    // Relabeling for Maple
    const double p__1 = p(0);
    const double p__2 = p(1);
    const double p__3 = p(2);
    const double v__1 = v(0);
    const double v__2 = v(1);
    const double v__3 = v(2);

    // MATLAB: Omega__1 = OM(1); ...
    const double Omega__1 = OM(0);
    const double Omega__2 = OM(1);
    const double Omega__3 = OM(2);

    (void)r11; (void)r12; (void)r13; (void)r21; (void)r22; (void)r23; (void)r31; (void)r32; (void)r33;
    (void)p__1; (void)p__2; (void)p__3;
    (void)v__1; (void)v__2; (void)v__3;
    (void)Omega__1; (void)Omega__2; (void)Omega__3;

    // Physical parameters from main_quasi.m
    static const double r1 = 5.0;
    static const double r2 = -3.0;
    static const double vd = 0.01;
    static const double m = 1.725;
    static const double g = 9.81;
    static const Eigen::Matrix3d J = (Eigen::Matrix3d() << 
        0.029125*3, 0,          0,
        0,          0.029125*3, 0,
        0,          0,          0.055225*3).finished();

    // MATLAB: J__1 = J(1,1); J__2 = J(2,2); J__3 = J(3,3);
    const double J__1 = J(0,0);
    const double J__2 = J(1,1);
    const double J__3 = J(2,2);

    // Gains (initialize here instead of reading Gains.m)
    // K1 multiplies [h1; h1_dot; h1_ddot; h1_dddot]
    // K2 multiplies [h2; h2_dot]
    // K3 multiplies [h3; h3_dot; h3_ddot]
    // K4 multiplies [h4; h4_dot]
    static const Eigen::Vector4d K1( /*k11*/0.0, /*k12*/0.0, /*k13*/0.0, /*k14*/0.0 );
    static const Eigen::Vector2d K2( /*k21*/0.0, /*k22*/0.0 );
    static const Eigen::Vector3d K3( /*k31*/0.0, /*k32*/0.0, /*k33*/0.0 );
    static const Eigen::Vector2d K4( /*k41*/0.0, /*k42*/0.0 );

    // MATLAB: k__21 = K2(1); k__22 = K2(2);
    const double k__21 = K2(0);
    const double k__22 = K2(1);

    // Helper functions for MATLAB-style trig
    const auto sec = [](double a) { return 1.0 / std::cos(a); };
    const auto sq = [](double a) { return a * a; };

    // MATLAB: e3 = [0;0;1];
    const Eigen::Vector3d e3(0.0, 0.0, 1.0);
    (void)e3;

    // MATLAB:
    // W =[sin(psi) * sec(phi) cos(psi) * sec(phi) 0;...
    //     cos(psi) -sin(psi) 0;...
    //     sin(psi) * tan(phi) cos(psi) * tan(phi) 1;];
    const double sec_phi = sec(phi);
    const double tan_phi = std::tan(phi);
    const double tan_theta = std::tan(theta);
    const double sec_theta = sec(theta);

    Eigen::Matrix3d W;
    W << std::sin(psi) * sec_phi, std::cos(psi) * sec_phi, 0.0,
         std::cos(psi),          -std::sin(psi),         0.0,
         std::sin(psi) * tan_phi, std::cos(psi) * tan_phi, 1.0;

    // MATLAB: eta_dt = W * OM;
    const Eigen::Vector3d eta_dt = W * OM;
    (void)eta_dt;

    // MATLAB W_dt (copied from your script)
    Eigen::Matrix3d W_dt;
    W_dt <<
        sec_phi * (tan_phi * sq(std::cos(psi)) * Omega__2 + (std::sin(psi) * (Omega__2 * sec_phi + Omega__1) * tan_phi + Omega__3) * std::cos(psi) + sec_phi * sq(std::sin(psi)) * tan_phi * Omega__1),
        sec_phi * (-sq(std::sin(psi)) * tan_phi * Omega__1 + (std::cos(psi) * (Omega__1 * sec_phi - Omega__2) * tan_phi - Omega__3) * std::sin(psi) + sq(std::cos(psi)) * Omega__2 * sec_phi * tan_phi),
        0.0,
        -std::sin(psi) * (Omega__3 + tan_phi * (Omega__1 * std::sin(psi) + Omega__2 * std::cos(psi))),
        -std::cos(psi) * (Omega__3 + tan_phi * (Omega__1 * std::sin(psi) + Omega__2 * std::cos(psi))),
        0.0,
        sq(tan_phi) * sq(std::cos(psi)) * Omega__2 + ((Omega__2 * std::pow(sec_phi, 3) + sq(tan_phi) * Omega__1) * std::sin(psi) + Omega__3 * tan_phi) * std::cos(psi) + sq(std::sin(psi)) * Omega__1 * std::pow(sec_phi, 3) - sq(std::sin(psi)) * sq(tan_phi) * Omega__1 + ((Omega__1 * std::pow(sec_phi, 3) - Omega__2 * sq(tan_phi)) * std::cos(psi) - Omega__3 * tan_phi) * std::sin(psi) + sq(std::cos(psi)) * Omega__2 * std::pow(sec_phi, 3),
        0.0,
        0.0;
    (void)W_dt;

    // ======================
    // The output h
    // ======================
    const double h1 = sq(p__1) + sq(p__2) - sq(r1);
    const double h2 = p__3 - r2;
    const double h3 = -(vd * std::sqrt(sq(p__1) + sq(p__2)) + p__1 * v__2 - p__2 * v__1) * std::pow(sq(p__1) + sq(p__2), -0.5);
    const double h4 = psi - std::atan2(p__2, p__1);

    // 1st time derivative of h
    const double h1__dt = 2.0 * p__1 * v__1 + 2.0 * p__2 * v__2;
    const double h2__dt = v__3;
    const double h3__dt = -sec_theta * (-std::cos(theta) * (p__1 * v__1 + p__2 * v__2) * (p__1 * v__2 - p__2 * v__1) + (p__1 * tan_phi + p__2 * std::sin(theta)) * (g + k__21 * (p__3 - r2) + k__22 * v__3) * (sq(p__1) + sq(p__2))) * std::pow(sq(p__1) + sq(p__2), -1.5);
    const double h4__dt = v__1 * p__2 / (sq(p__1) + sq(p__2)) - v__2 * p__1 / (sq(p__1) + sq(p__2)) + std::sin(psi) * tan_phi * Omega__1 + std::cos(psi) * tan_phi * Omega__2 + Omega__3;

    // 2nd time derivative of h1 and h3
    const double h1__ddt = (2.0 * sq(v__1)) + (2.0 * sq(v__2)) - 2.0 * (g + k__21 * (p__3 - r2) + k__22 * v__3) * (p__1 * tan_theta - p__2 * tan_phi * sec_theta);

    const double h3__ddt = -std::pow(sec_theta, 2) * (2.0 * (p__1 * v__2 - p__2 * v__1) * (sq(p__2) * (-sq(v__1) / 2.0 + sq(v__2)) + 3.0 * p__1 * p__2 * v__1 * v__2 + (sq(v__1) - sq(v__2) / 2.0) * sq(p__1)) * sq(std::cos(theta)) +
        ((-v__3 * p__2 * (sq(p__1) + sq(p__2)) * sq(k__22) + ((2.0 * sq(p__1) * v__2 - 3.0 * p__1 * p__2 * v__1 - sq(p__2) * v__2) * v__3 + k__21 * p__2 * (sq(p__1) + sq(p__2)) * (r2 - p__3)) * k__22 + k__21 * p__2 * (sq(p__1) + sq(p__2)) * v__3 + 2.0 * (sq(p__1) * v__2 - 3.0 / 2.0 * p__1 * p__2 * v__1 - sq(p__2) * v__2 / 2.0) * (k__21 * (p__3 - r2) + g)) * std::sin(theta) +
        (g + k__21 * (p__3 - r2) + k__22 * v__3) * std::pow(sec_phi, 2) * p__1 * Omega__1 * (sq(p__1) + sq(p__2)) * std::cos(psi) -
        (g + k__21 * (p__3 - r2) + k__22 * v__3) * std::sin(psi) * p__1 * Omega__2 * (sq(p__1) + sq(p__2)) * std::pow(sec_phi, 2) -
        tan_phi * (v__3 * p__1 * (sq(p__1) + sq(p__2)) * sq(k__22) + ((sq(p__1) * v__1 + 3.0 * p__1 * p__2 * v__2 - 2.0 * sq(p__2) * v__1) * v__3 - k__21 * p__1 * (sq(p__1) + sq(p__2)) * (r2 - p__3)) * k__22 - k__21 * p__1 * (sq(p__1) + sq(p__2)) * v__3 + (sq(p__1) * v__1 + 3.0 * p__1 * p__2 * v__2 - 2.0 * sq(p__2) * v__1) * (k__21 * (p__3 - r2) + g))) * (sq(p__1) + sq(p__2)) * std::cos(theta) +
        (Omega__1 * std::sin(psi) + Omega__2 * std::cos(psi)) * (g + k__21 * (p__3 - r2) + k__22 * v__3) * sec_phi * (tan_phi * std::sin(theta) * p__1 + p__2) * std::pow(sq(p__1) + sq(p__2), 2)) * std::pow(sq(p__1) + sq(p__2), -2.5);

    // 3rd time derivative of h1
    const double h1__dddt = (2.0 * (Omega__1 * std::sin(psi) + Omega__2 * std::cos(psi)) * p__2 * tan_theta * (g + k__21 * (p__3 - r2) + k__22 * v__3) * tan_phi * sec_phi +
        2.0 * (std::cos(psi) * Omega__1 - std::sin(psi) * Omega__2) * p__2 * (g + k__21 * (p__3 - r2) + k__22 * v__3) * sq(tan_phi) +
        ((((2.0 * r2 - 2.0 * p__3) * k__22 + 2.0 * v__3) * p__2 - 6.0 * v__2 * (r2 - p__3)) * k__21 - 2.0 * sq(k__22) * p__2 * v__3 + 6.0 * v__2 * (k__22 * v__3 + g)) * tan_phi +
        2.0 * (Omega__1 * std::cos(psi) * p__2 - Omega__2 * std::sin(psi) * p__2 + std::sin(theta) * (k__22 * p__1 - (2.0 * v__1))) * (g + k__21 * (p__3 - r2) + k__22 * v__3)) * sec_theta -
        2.0 * (1.0 + sq(tan_theta)) * (Omega__1 * std::sin(psi) + Omega__2 * std::cos(psi)) * (g + k__21 * (p__3 - r2) + k__22 * v__3) * p__1 * sec_phi -
        2.0 * tan_theta * ((p__1 * v__3 - v__1 * (r2 - p__3)) * k__21 + g * k__22 * p__1 + v__1 * (k__22 * v__3 + g));

    // Aux inputs
    const double nu__1 = -K1.dot(Eigen::Vector4d(h1, h1__dt, h1__ddt, h1__dddt));
    const double nu__2 = -K2.dot(Eigen::Vector2d(h2, h2__dt));
    const double nu__3 = -K3.dot(Eigen::Vector3d(h3, h3__dt, h3__ddt));
    const double nu__4 = -K4.dot(Eigen::Vector2d(h4, h4__dt));

    // QSF thrust
    const double denom = (std::cos(theta) * std::cos(phi));
    const double u__t = m * (g - nu__2) / denom;

    // ======================
    // tau (MATLAB: neat way)
    // ======================
    const Eigen::Vector3d dh_1(2.0 * p__1, 2.0 * p__2, 0.0);
    const Eigen::Vector3d dh_2(0.0, 0.0, 1.0);

    const Eigen::Vector3d cross_dh12 = dh_1.cross(dh_2);
    const double cross_norm = cross_dh12.norm();
    Eigen::Vector3d rho = Eigen::Vector3d::Zero();
    if (cross_norm > 1e-12) {
        rho = cross_dh12 / cross_norm;
    }

    const double cospsi = std::cos(psi);
    const double sinpsi = std::sin(psi);
    const double costh = std::cos(theta);
    const double sinth = std::sin(theta);
    const double tanth = std::tan(theta);

    // b1, b3, b4 (direct translation from MATLAB)
    const double b1 = 2.0 * (-(k__22 * v__3 + g + k__21 * (p__3 - r2)) * (-p__2 * (2.0 * tan_phi * J__1 * J__2 * std::pow(cospsi * Omega__2 + sinpsi * Omega__1, 2.0) * std::pow(tanth, 2.0)
        + (Omega__1 * Omega__2 * std::pow(cospsi, 2.0) * tan_phi * J__1 * J__2 + J__1 * (tan_phi * J__2 * (Omega__1 - Omega__2) * (Omega__1 + Omega__2) * sinpsi - Omega__1 * Omega__3 * (J__1 - J__2 - J__3)) * cospsi
        - sinpsi * (sinpsi * tan_phi * J__1 * Omega__1 + Omega__3 * (J__1 - J__2 + J__3)) * Omega__2 * J__2) * std::sin(phi) * tanth
        + Omega__3 * (-Omega__2 * J__2 * (J__1 - J__2 + J__3) * cospsi + Omega__1 * sinpsi * J__1 * (J__1 - J__2 - J__3))) * sec_theta
        + (2.0 * J__1 * J__2 * std::pow(cospsi * Omega__2 + sinpsi * Omega__1, 2.0) * tanth
        + (Omega__1 * Omega__2 * std::pow(cospsi, 2.0) * tan_phi * J__1 * J__2 + J__1 * (tan_phi * J__2 * (Omega__1 - Omega__2) * (Omega__1 + Omega__2) * sinpsi - Omega__1 * Omega__3 * (J__1 - J__2 - J__3)) * cospsi
        - sinpsi * (sinpsi * tan_phi * J__1 * Omega__1 + Omega__3 * (J__1 - J__2 + J__3)) * Omega__2 * J__2) * std::cos(phi)) * p__1 * (1.0 + std::pow(tanth, 2.0))) * std::pow(sec_phi, 2.0)
        - 2.0 * J__1 * ((-costh * (k__22 * v__3 + g + k__21 * (p__3 - r2)) * k__22 * p__1 * std::pow(tanth, 2.0) / 2.0
        + (-3.0 / 2.0 * (k__22 * v__3 + g + k__21 * (p__3 - r2)) * p__2 * Omega__1 * (std::pow(tan_phi, 2.0) + 2.0 / 3.0) * cospsi
        + 3.0 / 2.0 * (k__22 * v__3 + g + k__21 * (p__3 - r2)) * p__2 * Omega__2 * (std::pow(tan_phi, 2.0) + 2.0 / 3.0) * sinpsi
        + ((((p__3 - r2) * k__22 - v__3) * p__2 - 2.0 * v__2 * (p__3 - r2)) * k__21 + v__3 * std::pow(k__22, 2.0) * p__2 - 2.0 * v__2 * (k__22 * v__3 + g)) * tan_phi
        + (k__22 * v__3 + g + k__21 * (p__3 - r2)) * (v__1 - k__22 * p__1 / 2.0) * sinth) * tanth
        - costh * (k__22 * v__3 + g + k__21 * (p__3 - r2)) * k__22 * p__1 / 2.0) * sec_theta
        + ((k__22 * v__3 + g + k__21 * (p__3 - r2)) * tan_phi * p__1 * Omega__1 * cospsi / 2.0
        - (k__22 * v__3 + g + k__21 * (p__3 - r2)) * tan_phi * Omega__2 * p__1 * sinpsi / 2.0
        + (p__1 * v__3 + v__1 * (p__3 - r2)) * k__21 + g * p__1 * k__22 + v__1 * (k__22 * v__3 + g)) * std::pow(tanth, 2.0)
        + (k__22 * v__3 + g + k__21 * (p__3 - r2)) * tan_phi * p__1 * Omega__1 * cospsi / 2.0
        - (k__22 * v__3 + g + k__21 * (p__3 - r2)) * tan_phi * Omega__2 * p__1 * sinpsi / 2.0
        + (((-p__3 + r2) * k__22 / 2.0 + v__3) * p__1 + 2.0 * v__1 * (p__3 - r2)) * k__21 + k__22 * (-k__22 * v__3 + g) * p__1 / 2.0 + 2.0 * v__1 * (k__22 * v__3 + g)) * J__2 * (cospsi * Omega__2 + sinpsi * Omega__1) * sec_phi
        + 3.0 * J__1 * (std::pow(k__22 * v__3 + g + k__21 * (p__3 - r2), 2.0) * std::pow(tan_phi, 2.0) * std::pow(sec_theta, 2.0)
        + (2.0 * (k__22 * v__3 + g + k__21 * (p__3 - r2)) * tan_phi * p__2 * (1.0 + std::pow(tan_phi, 2.0)) * std::pow(Omega__1, 2.0) * std::pow(cospsi, 2.0)
        + 4.0 * (1.0 + std::pow(tan_phi, 2.0)) * (-(k__22 * v__3 + g + k__21 * (p__3 - r2)) * tan_phi * p__2 * Omega__2 * sinpsi
        + (((-p__3 + r2) * k__22 + v__3) * p__2 / 2.0 + v__2 * (p__3 - r2)) * k__21 - v__3 * std::pow(k__22, 2.0) * p__2 / 2.0 + v__2 * (k__22 * v__3 + g)) * Omega__1 * cospsi
        + 2.0 * (k__22 * v__3 + g + k__21 * (p__3 - r2)) * tan_phi * p__2 * std::pow(Omega__2, 2.0) * (1.0 + std::pow(tan_phi, 2.0)) * std::pow(sinpsi, 2.0)
        - 4.0 * ((((-p__3 + r2) * k__22 + v__3) * p__2 / 2.0 + v__2 * (p__3 - r2)) * k__21 - v__3 * std::pow(k__22, 2.0) * p__2 / 2.0 + v__2 * (k__22 * v__3 + g)) * Omega__2 * (1.0 + std::pow(tan_phi, 2.0)) * sinpsi
        + (-p__2 * (p__3 - r2) * std::pow(k__21, 2.0) + (k__22 * ((p__3 - r2) * k__22 - 2.0 * v__3) * p__2 - 4.0 * v__2 * ((p__3 - r2) * k__22 - v__3)) * k__21
        - 4.0 * v__3 * std::pow(k__22, 2.0) * (-k__22 * p__2 / 4.0 + v__2)) * tan_phi
        + (p__1 * (p__3 - r2) * std::pow(k__21, 2.0) + (((-p__3 + r2) * std::pow(k__22, 2.0) + 2.0 * k__22 * v__3 + g) * p__1 + 3.0 * ((p__3 - r2) * k__22 - 2.0 / 3.0 * v__3) * v__1) * k__21
        + k__22 * (-v__3 * std::pow(k__22, 2.0) * p__1 + v__1 * (3.0 * k__22 * v__3 + g))) * sinth) * sec_theta / 3.0
        + tanth * (std::pow(k__22 * v__3 + g + k__21 * (p__3 - r2), 2.0) * tanth + (-g * p__1 + ((p__3 - r2) * k__22 - 2.0 * v__3) * v__1) * k__21 / 3.0 - v__1 * k__22 * (-k__22 * v__3 + g) / 3.0)) * J__2) / J__1 / J__2;

    const double p12 = sq(p__1) + sq(p__2);
    const double b3 = (2.0 * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * J__1 * std::pow(p12, 3.0) * sec_theta
        * ((p__1 * (std::pow(sec_theta, 2.0) - 2.0) * (Omega__1 - Omega__2) * (Omega__1 + Omega__2) * sec_phi - 2.0 * Omega__1 * Omega__2 * sec_theta * p__2) * tan_phi
        - 4.0 * tanth * (Omega__1 * std::pow(sec_phi, 2.0) * p__1 + Omega__2 * sec_phi * sec_theta * p__2 / 4.0 - Omega__1 * p__1 / 2.0) * Omega__2) * sec_phi * J__2 * std::pow(cospsi, 2.0)
        + (J__1 * (-2.0 * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * (2.0 * (std::pow(sec_theta, 2.0) - 2.0) * Omega__1 * Omega__2 * p__1 * sec_phi + (Omega__1 - Omega__2) * (Omega__1 + Omega__2) * sec_theta * p__2) * p12 * J__2 * sinpsi
        + tanth * (2.0 * v__3 * Omega__2 * J__2 * p__1 * p12 * std::pow(k__22, 2.0)
        + (((Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 - 3.0 * v__1 * Omega__2 * J__2) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__2 * J__2 * p__1 * p__2 + (Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 + v__1 * Omega__2 * J__2) * std::pow(p__1, 2.0)) * v__3
        + 2.0 * k__21 * Omega__2 * J__2 * p__1 * p12 * (p__3 - r2)) * k__22
        - 2.0 * k__21 * Omega__2 * J__2 * p__1 * p12 * v__3
        + ((Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 - 3.0 * v__1 * Omega__2 * J__2) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__2 * J__2 * p__1 * p__2 + (Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 + v__1 * Omega__2 * J__2) * std::pow(p__1, 2.0)) * (k__21 * p__3 - k__21 * r2 + g))) * tan_phi
        - 4.0 * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * J__1 * p12 * tanth * (p__1 * (std::pow(Omega__1, 2.0) - std::pow(Omega__2, 2.0)) * std::pow(sec_phi, 2.0)
        + Omega__1 * Omega__2 * sec_phi * sec_theta * p__2 + p__1 * (-std::pow(Omega__1, 2.0) / 2.0 + std::pow(Omega__2, 2.0) / 2.0)) * J__2 * sinpsi
        + J__2 * (2.0 * v__3 * Omega__1 * J__1 * p__1 * p12 * std::pow(k__22, 2.0)
        + (((Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 - 3.0 * v__1 * Omega__1 * J__1) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__1 * J__1 * p__1 * p__2 + (Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 + v__1 * Omega__1 * J__1) * std::pow(p__1, 2.0)) * v__3
        + 2.0 * k__21 * Omega__1 * J__1 * p__1 * p12 * (p__3 - r2)) * k__22
        - 2.0 * k__21 * Omega__1 * J__1 * p__1 * p12 * v__3
        + ((Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 - 3.0 * v__1 * Omega__1 * J__1) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__1 * J__1 * p__1 * p__2 + (Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 + v__1 * Omega__1 * J__1) * std::pow(p__1, 2.0)) * (k__21 * p__3 - k__21 * r2 + g)) * sec_phi
        + 4.0 * J__1 * sec_theta * (v__3 * Omega__2 * J__2 * p__2 * p12 * std::pow(k__22, 2.0) / 2.0
        + ((Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * std::pow(p__2, 3.0) / 4.0 + v__2 * Omega__2 * J__2 * std::pow(p__2, 2.0) / 4.0 + (Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 / 4.0 + v__1 * Omega__2 * J__2) * p__1 * p__2
        - 3.0 / 4.0 * v__2 * Omega__2 * J__2 * std::pow(p__1, 2.0)) * v__3 + k__21 * Omega__2 * J__2 * p__2 * p12 * (p__3 - r2) / 2.0) * k__22
        - k__21 * Omega__2 * J__2 * p__2 * p12 * v__3 / 2.0
        + (k__21 * p__3 - k__21 * r2 + g) * (Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * std::pow(p__2, 3.0) / 4.0 + v__2 * Omega__2 * J__2 * std::pow(p__2, 2.0) / 4.0
        + (Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 / 4.0 + v__1 * Omega__2 * J__2) * p__1 * p__2 - 3.0 / 4.0 * v__2 * Omega__2 * J__2 * std::pow(p__1, 2.0)))) * std::pow(p12, 2.0) * sec_theta * sec_phi * cospsi
        - 3.0 * p12 * sec_theta * (-(p12) * tanth * sec_phi
        * (2.0 * v__3 * Omega__1 * J__1 * p__1 * p12 * std::pow(k__22, 2.0)
        + (((Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 - 3.0 * v__1 * Omega__1 * J__1) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__1 * J__1 * p__1 * p__2 + (Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 + v__1 * Omega__1 * J__1) * std::pow(p__1, 2.0)) * v__3
        + 2.0 * k__21 * Omega__1 * J__1 * p__1 * p12 * (p__3 - r2)) * k__22
        - 2.0 * k__21 * Omega__1 * J__1 * p__1 * p12 * v__3
        + ((Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 - 3.0 * v__1 * Omega__1 * J__1) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__1 * J__1 * p__1 * p__2 + (Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 + v__1 * Omega__1 * J__1) * std::pow(p__1, 2.0)) * (k__21 * p__3 - k__21 * r2 + g)) * sinpsi / 3.0
        + J__1 * (2.0 / 3.0 * std::pow(Omega__1, 2.0) * std::pow(sec_phi, 2.0) * p__1 * std::pow(p12, 2.0) * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * std::pow(tanth, 2.0)
        + (p__1 - p__2) * (p__1 + p__2) * p12 * std::pow(k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g, 2.0) * tanth
        + 2.0 / 3.0 * std::pow(Omega__2, 2.0) * p__1 * std::pow(p12, 2.0) * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * std::pow(sec_phi, 2.0)
        - 2.0 / 3.0 * Omega__1 * Omega__2 * sec_theta * p__2 * std::pow(p12, 2.0) * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * sec_phi
        + v__3 * p__1 * std::pow(p12, 2.0) * std::pow(k__22, 3.0) / 3.0
        + p12 * ((std::pow(p__1, 2.0) * v__1 + 4.0 * p__1 * p__2 * v__2 - 3.0 * std::pow(p__2, 2.0) * v__1) * v__3 + k__21 * p__1 * p12 * (p__3 - r2)) * std::pow(k__22, 2.0) / 3.0
        + ((-2.0 / 3.0 * p__1 * std::pow(p12, 2.0) * k__21 - 3.0 * v__1 * v__2 * std::pow(p__2, 3.0) + (-5.0 * std::pow(v__1, 2.0) + 4.0 * std::pow(v__2, 2.0)) * p__1 * std::pow(p__2, 2.0)
        + 9.0 * v__1 * v__2 * std::pow(p__1, 2.0) * p__2 + std::pow(p__1, 3.0) * (std::pow(v__1, 2.0) - 2.0 * std::pow(v__2, 2.0))) * v__3
        + k__21 * p12 * (std::pow(p__1, 2.0) * v__1 + 4.0 * p__1 * p__2 * v__2 - 3.0 * std::pow(p__2, 2.0) * v__1) * (p__3 - r2) / 3.0) * k__22
        - k__21 * p12 * (std::pow(p__1, 2.0) * v__1 + 4.0 * p__1 * p__2 * v__2 - 3.0 * std::pow(p__2, 2.0) * v__1) * v__3 / 3.0
        + (-p__1 * std::pow(p12, 2.0) * k__21 / 3.0 - 3.0 * v__1 * v__2 * std::pow(p__2, 3.0) + (-5.0 * std::pow(v__1, 2.0) + 4.0 * std::pow(v__2, 2.0)) * p__1 * std::pow(p__2, 2.0)
        + 9.0 * v__1 * v__2 * std::pow(p__1, 2.0) * p__2 + std::pow(p__1, 3.0) * (std::pow(v__1, 2.0) - 2.0 * std::pow(v__2, 2.0))) * k__21 * p__3
        - (-p__1 * std::pow(p12, 2.0) * k__21 / 3.0 - 3.0 * v__1 * v__2 * std::pow(p__2, 3.0) + (-5.0 * std::pow(v__1, 2.0) + 4.0 * std::pow(v__2, 2.0)) * p__1 * std::pow(p__2, 2.0)
        + 9.0 * v__1 * v__2 * std::pow(p__1, 2.0) * p__2 + std::pow(p__1, 3.0) * (std::pow(v__1, 2.0) - 2.0 * std::pow(v__2, 2.0))) * k__21 * r2
        + g * (-3.0 * v__1 * v__2 * std::pow(p__2, 3.0) + (-5.0 * std::pow(v__1, 2.0) + 4.0 * std::pow(v__2, 2.0)) * p__1 * std::pow(p__2, 2.0)
        + 9.0 * v__1 * v__2 * std::pow(p__1, 2.0) * p__2 + std::pow(p__1, 3.0) * (std::pow(v__1, 2.0) - 2.0 * std::pow(v__2, 2.0)))) * J__2 * tan_phi
        - 2.0 * std::pow(Omega__1, 2.0) * std::pow(sec_phi, 2.0) * std::pow(sec_theta, 2.0) * tanth * J__1 * J__2 * p__2 * std::pow(p12, 3.0) * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * std::pow(sinpsi, 2.0)
        + 4.0 * std::pow(p12, 2.0) * sec_theta * sec_phi * (-J__1 * (2.0 * v__3 * Omega__2 * J__2 * p__1 * p12 * std::pow(k__22, 2.0)
        + (((Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 - 3.0 * v__1 * Omega__2 * J__2) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__2 * J__2 * p__1 * p__2 + (Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 + v__1 * Omega__2 * J__2) * std::pow(p__1, 2.0)) * v__3
        + 2.0 * k__21 * Omega__2 * J__2 * p__1 * p12 * (p__3 - r2)) * k__22
        - 2.0 * k__21 * Omega__2 * J__2 * p__1 * p12 * v__3
        + ((Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 - 3.0 * v__1 * Omega__2 * J__2) * std::pow(p__2, 2.0) + 4.0 * v__2 * Omega__2 * J__2 * p__1 * p__2 + (Omega__1 * Omega__3 * (J__1 - J__2 - J__3) * p__1 + v__1 * Omega__2 * J__2) * std::pow(p__1, 2.0)) * (k__21 * p__3 - k__21 * r2 + g)) * sec_phi / 4.0
        + sec_theta * J__2 * (v__3 * Omega__1 * J__1 * p__2 * p12 * std::pow(k__22, 2.0) / 2.0
        + ((Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * std::pow(p__2, 3.0) / 4.0 + v__2 * Omega__1 * J__1 * std::pow(p__2, 2.0) / 4.0 + (Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 / 4.0 + v__1 * Omega__1 * J__1) * p__1 * p__2
        - 3.0 / 4.0 * v__2 * Omega__1 * J__1 * std::pow(p__1, 2.0)) * v__3 + k__21 * Omega__1 * J__1 * p__2 * p12 * (p__3 - r2) / 2.0) * k__22
        - k__21 * Omega__1 * J__1 * p__2 * p12 * v__3 / 2.0
        + (Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * std::pow(p__2, 3.0) / 4.0 + v__2 * Omega__1 * J__1 * std::pow(p__2, 2.0) / 4.0 + (Omega__2 * Omega__3 * (J__1 - J__2 + J__3) * p__1 / 4.0 + v__1 * Omega__1 * J__1) * p__1 * p__2
        - 3.0 / 4.0 * v__2 * Omega__1 * J__1 * std::pow(p__1, 2.0)) * (k__21 * p__3 - k__21 * r2 + g))) * sinpsi
        + 3.0 * J__1 * (-4.0 * p12 * (-Omega__1 * Omega__2 * sec_theta * p__1 * std::pow(p12, 2.0) * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * std::pow(sec_phi, 3.0) / 3.0
        + Omega__1 * Omega__2 * sec_theta * p__1 * std::pow(p12, 2.0) * (k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g) * sec_phi / 6.0
        + v__3 * p__2 * std::pow(p12, 2.0) * std::pow(k__22, 3.0) / 12.0
        + ((-3.0 * v__2 * std::pow(p__1, 2.0) + 4.0 * v__1 * p__1 * p__2 + v__2 * std::pow(p__2, 2.0)) * v__3 + k__21 * p__2 * p12 * (p__3 - r2)) * p12 * std::pow(k__22, 2.0) / 12.0
        + ((-p__2 * std::pow(p12, 2.0) * k__21 / 6.0 + (-std::pow(v__1, 2.0) / 2.0 + std::pow(v__2, 2.0) / 4.0) * std::pow(p__2, 3.0) + 9.0 / 4.0 * std::pow(p__2, 2.0) * v__1 * p__1 * v__2
        + std::pow(p__1, 2.0) * (std::pow(v__1, 2.0) - 5.0 / 4.0 * std::pow(v__2, 2.0)) * p__2 - 3.0 / 4.0 * v__1 * std::pow(p__1, 3.0) * v__2) * v__3
        + (p__3 - r2) * p12 * (v__1 * p__1 * p__2 - 3.0 / 4.0 * v__2 * std::pow(p__1, 2.0) + v__2 * std::pow(p__2, 2.0) / 4.0) * k__21 / 3.0) * k__22
        - p12 * (v__1 * p__1 * p__2 - 3.0 / 4.0 * v__2 * std::pow(p__1, 2.0) + v__2 * std::pow(p__2, 2.0) / 4.0) * k__21 * v__3 / 3.0
        + (-p__2 * std::pow(p12, 2.0) * k__21 / 12.0 + (-std::pow(v__1, 2.0) / 2.0 + std::pow(v__2, 2.0) / 4.0) * std::pow(p__2, 3.0) + 9.0 / 4.0 * std::pow(p__2, 2.0) * v__1 * p__1 * v__2
        + std::pow(p__1, 2.0) * (std::pow(v__1, 2.0) - 5.0 / 4.0 * std::pow(v__2, 2.0)) * p__2 - 3.0 / 4.0 * v__1 * std::pow(p__1, 3.0) * v__2) * k__21 * p__3
        - (-p__2 * std::pow(p12, 2.0) * k__21 / 12.0 + (-std::pow(v__1, 2.0) / 2.0 + std::pow(v__2, 2.0) / 4.0) * std::pow(p__2, 3.0) + 9.0 / 4.0 * std::pow(p__2, 2.0) * v__1 * p__1 * v__2
        + std::pow(p__1, 2.0) * (std::pow(v__1, 2.0) - 5.0 / 4.0 * std::pow(v__2, 2.0)) * p__2 - 3.0 / 4.0 * v__1 * std::pow(p__1, 3.0) * v__2) * k__21 * r2
        + g * ((-std::pow(v__1, 2.0) / 2.0 + std::pow(v__2, 2.0) / 4.0) * std::pow(p__2, 3.0) + 9.0 / 4.0 * std::pow(p__2, 2.0) * v__1 * p__1 * v__2
        + std::pow(p__1, 2.0) * (std::pow(v__1, 2.0) - 5.0 / 4.0 * std::pow(v__2, 2.0)) * p__2 - 3.0 / 4.0 * v__1 * std::pow(p__1, 3.0) * v__2)) * tanth
        + std::pow(sec_theta, 2.0) * p__1 * p__2 * std::pow(p12, 2.0) * std::pow(k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g, 2.0) * std::pow(sec_phi, 2.0)
        - 2.0 * std::pow(sec_theta, 2.0) * p__1 * p__2 * std::pow(p12, 2.0) * std::pow(k__21 * p__3 - k__21 * r2 + k__22 * v__3 + g, 2.0)
        + std::pow(v__3, 2.0) * p__1 * p__2 * std::pow(p12, 2.0) * std::pow(k__22, 2.0)
        + 2.0 * v__3 * p__1 * p__2 * std::pow(p12, 2.0) * (k__21 * p__3 - k__21 * r2 + g) * k__22
        + std::pow(k__21, 2.0) * p__1 * p__2 * std::pow(p12, 2.0) * std::pow(p__3, 2.0)
        + 2.0 * k__21 * p__1 * p__2 * std::pow(p12, 2.0) * (-k__21 * r2 + g) * p__3
        + std::pow(k__21, 2.0) * p__1 * p__2 * std::pow(p12, 2.0) * std::pow(r2, 2.0)
        - 2.0 * g * k__21 * p__1 * p__2 * std::pow(p12, 2.0) * r2
        + std::pow(g, 2.0) * p__1 * std::pow(p__2, 5.0)
        + (3.0 * std::pow(v__1, 3.0) * v__2 - 2.0 * v__1 * std::pow(v__2, 3.0)) * std::pow(p__2, 4.0)
        + 2.0 * (std::pow(g, 2.0) * std::pow(p__1, 2.0) + 3.0 / 2.0 * std::pow(v__1, 4.0) - 15.0 / 2.0 * std::pow(v__2, 2.0) * std::pow(v__1, 2.0) + std::pow(v__2, 4.0)) * p__1 * std::pow(p__2, 3.0)
        + (-15.0 * std::pow(v__1, 3.0) * v__2 + 15.0 * v__1 * std::pow(v__2, 3.0)) * std::pow(p__2, 2.0) * std::pow(p__1, 2.0)
        + std::pow(p__1, 3.0) * (std::pow(g, 2.0) * std::pow(p__1, 2.0) - 2.0 * std::pow(v__1, 4.0) + 15.0 * std::pow(v__2, 2.0) * std::pow(v__1, 2.0) - 3.0 * std::pow(v__2, 4.0)) * p__2
        + (2.0 * std::pow(v__1, 3.0) * v__2 - 3.0 * v__1 * std::pow(v__2, 3.0)) * std::pow(p__1, 4.0)) * J__2))
        * std::pow(p12, -3.5) / J__1 / J__2;

    const double b4 = (1.0 / (J__1 * J__2 * J__3)) * (-(2.0 * J__1 * J__2 * J__3 * p__1 * p__2 * std::pow(v__1, 2.0) / std::pow(sq(p__1) + sq(p__2), 2.0))
        + (2.0 * J__1 * J__2 * J__3 * v__1 * v__2 / std::pow(sq(p__1) + sq(p__2), 2.0) * (std::pow(p__1, 2.0) - std::pow(p__2, 2.0)))
        + (2.0 * J__1 * J__2 * J__3 * p__1 * p__2 * std::pow(v__2, 2.0) / std::pow(sq(p__1) + sq(p__2), 2.0))
        + 2.0 * J__1 * J__2 * J__3 * (cospsi * Omega__2 + sinpsi * Omega__1) * (cospsi * Omega__1 - sinpsi * Omega__2) * std::pow(tan_phi, 2.0)
        + J__3 * Omega__3 * (J__1 * Omega__1 * (-J__1 + J__2 + J__3) * cospsi + J__2 * Omega__2 * (-J__1 + J__2 - J__3) * sinpsi) * tan_phi
        + J__2 * J__1 * (Omega__1 * Omega__2 * std::pow(cospsi, 2.0) * J__3 + sinpsi * J__3 * (Omega__1 - Omega__2) * (Omega__1 + Omega__2) * cospsi - Omega__1 * Omega__2 * std::pow(sinpsi, 2.0) * J__3 + J__1 * Omega__1 * Omega__2 - J__2 * Omega__1 * Omega__2));

    // Atilde matrix
    const double tmp = dh_2.dot(R.col(2));
    Eigen::Matrix3d Atilde;
    const Eigen::Vector3d R1 = R.col(0);
    const Eigen::Vector3d R2 = R.col(1);
    const Eigen::Vector3d R3 = R.col(2);

    const double Atilde11 = u__t * (cross_dh12.dot(R1)) / (m * J__1 * tmp);
    const double Atilde12 = u__t * (cross_dh12.dot(R2)) / (m * J__2 * tmp);
    const double Atilde21 = u__t * ((dh_2.cross(R1)).dot(rho)) / (m * J__1 * tmp);
    const double Atilde22 = u__t * ((dh_2.cross(R2)).dot(rho)) / (m * J__2 * tmp);
    const double Atilde31 = sinpsi * tan_phi / J__1;
    const double Atilde32 = cospsi * tan_phi / J__2;
    const double Atilde33 = 1.0 / J__3;

    Atilde << Atilde11, Atilde12, 0.0,
              Atilde21, Atilde22, 0.0,
              Atilde31, Atilde32, Atilde33;

    // tau = Atilde \ ([nu__1;nu__3;nu__4] - [b1;b3;b4])
    const Eigen::Vector3d rhs(nu__1 - b1, nu__3 - b3, nu__4 - b4);
    const Eigen::Vector3d tau_frd = Atilde.fullPivLu().solve(rhs);

    // ControllerNode publishes torque after converting FLU->FRD.
    // Since the quasi math is now using FRD body axes, convert back FRD->FLU here
    // so the rest of the pipeline stays consistent.
    const Eigen::Vector3d tau(tau_frd.x(), -tau_frd.y(), -tau_frd.z());

    double thrust = u__t;

    *controller_torque_thrust << tau, thrust;

    std::ofstream file("controller_dataQuasi.txt", std::ios::app);
    if (file.is_open()) {
        file << std::fixed << std::setprecision(6);
        file << p__1 << "," << p__2 << "," << p__3 << ","
             << v__1 << "," << v__2 << "," << v__3 << ","
             << theta << "," << phi << "," << psi << ","
             << thrust << "," << tau.transpose() << "\n";
        file.close();
    }
}