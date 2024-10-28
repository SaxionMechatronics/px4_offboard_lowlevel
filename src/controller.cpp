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

controller::controller(){
    zeta1 = 9.81;
    zeta2 = 0.0;
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