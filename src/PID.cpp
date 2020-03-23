#include "PID.h"
#include <vector>
#include <iostream>
#include <numeric>

using std::vector;
using std::endl;
using std::cout;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(vector <double> &p) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   p_ = p;
   prev_cte_ = 0.0;
   int_cte_ = 0.0;
   lastclock = 0;
   is_init_ = true;
   loops = 0;
   total_error_ = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */


}
double PID::Run(double &cte, double &setpoint){

  loops += 1;
  if(is_init_){

    cte_ = cte - setpoint;
    clock_t t;
    t = clock();
    int delta_t = t - lastclock;
    lastclock = t;

    double diff_cte_ = cte_ - prev_cte_;
    prev_cte_ = cte_;
    int_cte_ += cte_;
    int_cte_ /= delta_t;

    TotalError();

    return (-p_[0] * cte_ - p_[2] * diff_cte_ - p_[1] * int_cte_);
  }

  std::cout << "PID controller not Initialized" << std::endl;
  return 0;

}

double PID::Twiddle(double &cte, double &setpoint, int &n){
  double control_val = 0;

  vector <double> dp{1,1,1};
  double tol = 0.2;


  if(loops<=n){
  // run until n samples are collected
  control_val =  Run(cte,setpoint);
  SIM_RESET = false;
  }

  else{
    // if loops are reached reset the sim and initalize the controller
    is_init_ = false;
    //Init(p[0],p[1],p[2]);
    SIM_RESET = true;
    best_error = TotalError();


    int it = 0;
    double dp_sum = std::accumulate(dp.begin(), dp.end(), 0.0);
    std::cout << dp_sum << std::endl;



    /*for(int i = 0; i < p.size(); i++){

      p[i] += dp[i];
    }*/

  }






  return control_val;

  //vector<int> dp{ 1, 1, 1 };

  // run pid loop for a bit and then check the summed errors()


}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  total_error_ += cte_*cte_;
  return total_error_/loops;  // TODO: Add your total error calc here!
}
