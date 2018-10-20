#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

 protected:

  typedef CPPAD_TESTVECTOR(double) Dvector;

  void initVars(Dvector& vars, const Eigen::VectorXd state);

  void processVarsRange(Dvector& vars_lowerbound, Dvector& vars_upperbound);

  void processConstraintsRange(Dvector& constraints_lowerbound, Dvector& constraints_upperbound);

};

#endif /* MPC_H */
