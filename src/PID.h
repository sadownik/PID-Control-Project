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

  double steering_angle;

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double &Kp, double &Ki, double &Kd);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  void Run(double &cte);


  void Twiddle(vector <double> dp);
  /**
   * apply twiddle from the class.
   */

  double TotalError();


  private:

  bool is_init_ = false;
  double prev_cte_;
  double int_cte_;
  int lastclock;
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
