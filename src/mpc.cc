#include "mpc.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <iostream>

using namespace std;
using CppAD::AD;

using Dvector = vector<double>;

const size_t N = 10;
const double dt = 0.1;

const size_t number_of_state_dimensions = 6;
const size_t number_of_control_dimensions = 2;
const size_t n_vars = number_of_state_dimensions * N + number_of_control_dimensions * N-1;
const size_t n_constraints = N * number_of_state_dimensions;

const double ref_cte = 0;
const double ref_epsi = 0;
const double ref_v = 99;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// weights for the objective function
const double cte_weight           = 1800.0;
const double epsi_weight          = 1800.0;
const double v_weight             = 1.0;
const double delta_weight         = 20.0;
const double throttle_weight      = 10.0;
const double delta_diff_weight    = 250.0;
const double throttle_diff_weight = 15.0;

struct FG_eval {
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs): coeffs(coeffs) {}

  using ADvector = CPPAD_TESTVECTOR(AD<double>);
  void operator()(ADvector& fg, const ADvector& vars) {
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    /**
     * Here we define our cost
     * The following are the cost factors:
     * 1. CTE
     * 2. EPSI
     * 3. VELOCITY
     *
     * // N-1 actuations
     * 4. DELTA (steering)
     * 5. THROTTLE
     *
     * // N-2
     * 6. DELTA diff
     * 7. THROTTLE diff
     */

    for (int i = 0; i < N; ++i) {
      auto cte_cost   = CppAD::pow(vars[cte_start + i] - ref_cte, 2);
      auto epsi_cost  = CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
      auto v_cost     = CppAD::pow(vars[v_start + i] - ref_v, 2);

      fg[0] += cte_weight*cte_cost + epsi_weight * epsi_cost + v_weight * v_cost;

      if (i < N-1) {
        auto delta_cost     = CppAD::pow(vars[delta_start + i], 2);
        auto throttle_cost  = CppAD::pow(vars[a_start + i], 2);

        fg[0] += delta_weight * delta_cost + throttle_weight * throttle_cost;
      }

      if (i < N-2) {
        auto delta_diff_cost    = CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
        auto throttle_diff_cost = CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);

        fg[0] += delta_diff_weight*delta_diff_cost + throttle_diff_weight*throttle_diff_cost;
      }
    }

    //enforce init state does not change.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    for (int i = 0; i < N - 1; i++) {
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0 ;
      AD<double> psides0 = CppAD::atan(3*coeffs[3]* x0 * x0 + 2*coeffs[2]* x0 + coeffs[1]);

      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * (-delta0) / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * (-delta0) / Lf * dt);
    }
  }
};

void mpc::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  /**
   * 1st we setup the vector to hold all variable to be optimized.
   *
   */
  // Initial value of the independent variables should be 0 except for the initial values.
  Dvector vars(n_vars,0);

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  /**
   * 2nd. We setup the vectors to hold variables upper and lower bounds. Optimizer cannot cross these
   * bounds!
   *
   */

  Dvector vars_lowerbound(n_vars,0);
  Dvector vars_upperbound(n_vars,0);

  // Set all non-actuators upper and lower limits to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25 degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/deceleration upper and lower limits.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.1;
    vars_upperbound[i] = 1.0;
  }

  /**
   * 3rd. Setup the vectors for the upper and lower constraints bounds.
   */

  // Lower and upper limits for the constraints should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints,0);
  Dvector constraints_upperbound(n_constraints,0);

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options,
                                        vars,
                                        vars_lowerbound,
                                        vars_upperbound,
                                        constraints_lowerbound,
                                        constraints_upperbound,
                                        fg_eval,
                                        solution);

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  steering = solution.x[delta_start];
  throttle = solution.x[a_start];

  predicted_x_vals.clear();
  predicted_y_vals.clear();
  for (int i = 0; i < N; ++i) {
    predicted_x_vals.push_back(solution.x[x_start + i]);
    predicted_y_vals.push_back(solution.x[y_start + i]);
  }
}
