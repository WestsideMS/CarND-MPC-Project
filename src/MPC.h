#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using std::vector;

struct Solution {
  vector<double> Delta;
  vector<double> A;
  vector<double> X;
  vector<double> Y;
};

class MPC {
public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  Solution Solve(const Eigen::VectorXd state, const Eigen::VectorXd coeffs);

  double delta_prev {0};
  double a_prev {0.1};
};

#endif  // MPC_H
