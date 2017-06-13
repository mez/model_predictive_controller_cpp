#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include <vector>

class mpc {
 public:
  std::vector<double> predicted_x_vals;
  std::vector<double> predicted_y_vals;
  double steering;
  double throttle;

  // Solve the model given an initial state and polynomial coefficients.
  void Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
