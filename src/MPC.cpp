#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using Eigen::VectorXd;

/**
 * Set the timestep length and duration
 */
size_t N = 12;
double dt = 0.05;
int latency_step = 2;
double ref_v = 65;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
//   simulator around in a circle with a constant steering angle and velocity on
//   a flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
//   presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// calculate the starting index for decision variables
size_t x_i     = 0;
size_t y_i     = x_i + N;
size_t psi_i   = y_i + N;
size_t v_i     = psi_i + N;
size_t cte_i   = v_i + N;
size_t epsi_i  = cte_i + N;
size_t delta_i = epsi_i + N;
size_t a_i     = delta_i + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients
  VectorXd coeffs;
  FG_eval(VectorXd coeffs_) { 
    coeffs = coeffs_;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    /**
     * implement MPC
     * `fg` is a vector of the cost constraints, `vars` is a vector of variable 
     *   values (state & actuators)
     * NOTE: You'll probably go back and forth between this function and
     *   the Solver function below.
     */

    // the cost function
    fg[0] = 0;

    // cost for cte, epsi, v
    for (int i =0; i < N; i++) {
      fg[0] += CppAD::pow(vars[cte_i + i] - 0, 2);
      fg[0] += CppAD::pow(vars[epsi_i + i] - 0, 2);
      fg[0] += CppAD::pow(vars[v_i + i] - ref_v, 2);
    }

    // cost for the use of actuators
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_i + i], 2);
      fg[0] += 10 * CppAD::pow(vars[a_i + i], 2);
    }

    // cost for the difference between sequentil actuations
    for (int i = 0; i < N - 2; i++){
      fg[0] += 600 * CppAD::pow(vars[delta_i + i + 1] - vars[delta_i + i], 2);
      fg[0] += CppAD::pow(vars[a_i + i + 1] - vars[a_i + i], 2);
    }

    // set up constraints

    // constraints for the starting variables
    fg[1 + x_i]     = vars[x_i];
    fg[1 + y_i]     = vars[y_i];
    fg[1 + psi_i]   = vars[psi_i];
    fg[1 + v_i]     = vars[v_i];
    fg[1 + cte_i]   = vars[cte_i];
    fg[1 + epsi_i]  = vars[epsi_i];

    // other constraints
    for (int i = 1; i < N; i++) {
      // states at time t
      AD<double> x1     = vars[x_i + i];
      AD<double> y1     = vars[y_i + i];
      AD<double> psi1   = vars[psi_i + i];
      AD<double> v1     = vars[v_i + i];
      AD<double> cte1   = vars[cte_i + i];
      AD<double> epsi1  = vars[epsi_i + i];

      // states at time t-1
      AD<double> x0     = vars[x_i + i - 1];
      AD<double> y0     = vars[y_i + i - 1];
      AD<double> psi0   = vars[psi_i + i - 1];
      AD<double> v0     = vars[v_i + i - 1];
      AD<double> cte0   = vars[cte_i + i - 1];
      AD<double> epsi0  = vars[epsi_i + i - 1];
      
      // only consider control variables at time t
      AD<double>  delta0 = vars[delta_i + i - 1];
      AD<double>  a0     = vars[a_i + i - 1];

      AD<double>  f0     = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + \
                           coeffs[3] * x0 * x0 * x0;
      AD<double>  psi0_es  = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + \
                           3 * coeffs[3] * x0 * x0); 

      // model equations:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_i + i]    = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_i + i]    = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_i + i]  = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_i + i]    = v1 - (v0 + a0 * dt);
      fg[1 + cte_i + i]  = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_i + i] = epsi1 - (psi0 - psi0_es + v0 * delta0 / Lf * dt);
    }

  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

Solution MPC::Solve(const VectorXd x0, const VectorXd coeffs) {

  typedef CPPAD_TESTVECTOR(double) Dvector;

  /**
   * Set the number of model variables (includes both states and inputs).
   * For example: If the state is a 4 element vector, the actuators is a 2
   *   element vector and there are 10 timesteps. The number of variables is:
   *   4 * 10 + 2 * 9
   */
  size_t n_vars = N * 6 + (N - 1) * 2;
  /**
   * Set the number of constraints
   */
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  // set intial values for starting variables
  vars[x_i]    = x0[0];
  vars[y_i]    = x0[1];
  vars[psi_i]  = x0[2];
  vars[v_i]    = x0[3];
  vars[cte_i]  = x0[4];
  vars[epsi_i] = x0[5];

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  /**
   * Set lower and upper limits for variables.
   */
  
  // set bounds for non-actuators variables
  for (int i = 0; i < delta_i; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // set bounds for delta as [-25, 25] degrees
  for (int i = delta_i; i < a_i; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // constrain delta bounds for the latency time
  for (int i = delta_i; i < delta_i + latency_step; i++) {
    vars_lowerbound[i] = delta_prev;
    vars_upperbound[i] = delta_prev;
  }

  // set bounds for a
  for (int i = a_i; i < n_vars; i++) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }
  for (int i = a_i; i < a_i + latency_step; i++) {
    vars_lowerbound[i] = a_prev;
    vars_upperbound[i] = a_prev;    
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_i] = x0[0];
  constraints_lowerbound[y_i] = x0[1];
  constraints_lowerbound[psi_i] = x0[2];
  constraints_lowerbound[v_i] = x0[3];
  constraints_lowerbound[cte_i] = x0[4];
  constraints_lowerbound[epsi_i] = x0[5];

  constraints_upperbound[x_i] = x0[0];
  constraints_upperbound[y_i] = x0[1];
  constraints_upperbound[psi_i] = x0[2];
  constraints_upperbound[v_i] = x0[3];
  constraints_upperbound[cte_i] = x0[4];
  constraints_upperbound[epsi_i] = x0[5];
  

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // NOTE: You don't have to worry about these options
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  //   of sparse routines, this makes the computation MUCH FASTER. If you can
  //   uncomment 1 of these and see if it makes a difference or not but if you
  //   uncomment both the computation time should go up in orders of magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.05\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = true;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  Solution sol;
  for (int i = 0; i < N - 1; i++){
    //std::cout << i << ": " << "solution.x[x_start+i]: " << solution.x[x_i+i] \
    //<< "solution.x[delta_start+i]: " << solution.x[delta_i+i] << std::endl;
    
  	sol.Delta.push_back(solution.x[delta_i + i]);
  	sol.A.push_back(solution.x[a_i + i]);
  	sol.X.push_back(solution.x[x_i + i]);
  	sol.Y.push_back(solution.x[y_i + i]);

  }

  return sol;
}
