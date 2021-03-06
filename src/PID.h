#ifndef PID_H
#define PID_H

#include <vector>
using std::vector;



class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();



  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(vector <double> &p);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double Run(double &cte, double &setpoint);


  double Twiddle(double &cte, double &setpoint, int &n);
  /**
   * apply twiddle from the class.
   */

  double pid_tune(double &cte, double &setpoint, int &n, int param);


  double TotalError();
  int loops = 0;
  bool SIM_RESET = false;
  vector <double> p_;
  int finished = 0;

  private:

  bool is_init_ = false;
  double prev_cte_;
  double int_cte_;
  int lastclock;
  double cte_;
  double total_error_;

  double best_error = 10000;



  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;
};

#endif  // PID_H
