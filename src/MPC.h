#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  MPC();
  virtual ~MPC();
  std::vector<double> mpc_x;
  std::vector<double> mpc_y;

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(const Eigen::VectorXd &state, 
                            const Eigen::VectorXd &coeffs);
};

#endif  // MPC_H
