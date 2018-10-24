#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
static double deg2rad(double x) { return x * pi() / 180; }

class MPC {
 public:

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  const std::vector<double>& getMpcX() const;

  const std::vector<double>& getMpcY() const;
  
  void setPrevDelta(const double);
  
  void setPrevA(const double);

 protected:

  typedef CPPAD_TESTVECTOR(double) Dvector;

  void initVars(Dvector& vars, const Eigen::VectorXd state);

  void setVarsLimitRange(Dvector& vars_lowerbound, Dvector& vars_upperbound);

  void initConstraints(Dvector& constraints_lowerbound, Dvector& constraints_upperbound, const Eigen::VectorXd state);
  
private:
    std::vector<double> mpc_x;
  	std::vector<double> mpc_y;
    double prev_delta;
 	double prev_a;

};

#endif /* MPC_H */
