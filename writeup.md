# CarND-Controls-PID Writeup


---
This document describes the PID-Controller implementation. A quick demonstration was recorded and can be found by clicking the image below.


[![video](https://img.youtube.com/vi/BhFXAr6FzZE/0.jpg)](https://www.youtube.com/watch?v=BhFXAr6FzZE)

## Concept

Upon start, two Objects of the type `PID` are created, one controller for the steering and one for the throttle. Both are initialized using the method `Init()`:

```c++
void PID::Init(vector <double> &p) {
  //
   // TODO: Initialize PID coefficients (and errors, if needed)
   //
   p_ = p;
   prev_cte_ = 0.0;
   int_cte_ = 0.0;
   lastclock = 0;
   is_init_ = true;
   loops = 0;
   total_error_ = 0;

}
```
The method initializes variables for the I and D-Term, the elapsed loop count as well as the accumulated error. If the simulator is reset, it is therefore necesessary to reset the controllers. During operation within the main function, the method `Run` is used to return the necessary steering angle and throttle position. The current error as well as the desired setpoint value are passed:

```c++
double steering_value = 0;
double setpoint_steering = 0.0;

steering_value = pid.Run(cte, setpoint_steering);

double setpoint_speed = 20;
double throttle_value = thr_pid.Run(speed, setpoint_speed);
```
The concept is adopted from the previous PID-controller class: The time between is calculated by System-cycles and not time as a SI-unit. Since there is a factor to seconds for system cycles it can be ignored by the adjustment of the I-Gain.  

```c++
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
```

## Parameter Tuning

Parameter tuning was implemented using the method `pid_tune`. It raises P,I or D gain until there is no more improvement in the accumulated squared cte-error. Then it changes the parameter. To automate this, a loop count is defined. Is it reached the  `SIM_RESET` flag is raised which sents a "reset car" message to the simulator. The pid is reinitialized with the new PID Values and it keeps probing until it's finished. This is a very simplistic version of twiddle. 
