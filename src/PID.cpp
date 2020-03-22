#include "PID.h"
#include <vector>
#include <iostream>

using std::vector;
using std::endl;
using std::cout;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double &Kp, double &Ki, double &Kd) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   //std::cout<< "init done" << std::endl;
   if(!is_init_){
     Kp_ = Kp;
     Ki_ = Ki;
     Kd_ = Kd;
     prev_cte_ = 0.0;
     int_cte_ = 0.0;
     lastclock = 0;
     is_init_ = true;
   }



}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */


}
void PID::Run(double &cte){

  if(is_init_){

    clock_t t;
    t = clock();
    int delta_t = t - lastclock;
    lastclock = t;


    double diff_cte_ = cte - prev_cte_;
    prev_cte_ = cte;
    int_cte_ += cte;
    int_cte_ /= delta_t;


    steering_angle = -Kp_ * cte - Kd_ * diff_cte_ - Ki_ * int_cte_;
    //steering_angle = -Kp_ * cte;
    //steering_angle = -Kp_ * cte - Kd_ * diff_cte_;
  }


}

void PID::Twiddle(vector <double> dp){

  vector <double> p;
  p.push_back(Kp_);
  p.push_back(Ki_);
  p.push_back(Kd_);

  // run pid loop for a bit and then check the summed errors()


}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  //err += cte*cte;
  return 0.0;  // TODO: Add your total error calc here!
}
