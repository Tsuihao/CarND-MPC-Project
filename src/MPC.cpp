#include "MPC.h"
using CppAD::AD;
using namespace std;
// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.01;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Ref speed
const double ref_v = 70.0;
const double ref_cte = 0.0; // want cte = 0
const double ref_epsi = 0.0; // want the angle differnece = 0

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;  // only has N-1
size_t a_start = delta_start + N - 1; // only has N-1


class FG_eval 
{
public:

  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    /* Design cost function */
    buildCostFunc(fg, vars);

    /* Set up constriants */
    setUpConstraints(fg, vars);

  }
protected:

    void buildCostFunc(ADvector& fg, const ADvector& vars)
    {
      // fg[0] is the cost
      // TODO: can add the weights here!
      fg[0] = 0; // init the cost function with 0
      
      static const double weight_cte = 1;
      static const double weight_epsi = 1;
      static const double weight_v = 1;
      static const double weight_steering = 200;
      static const double weight_acceleration = 1;
      
      
      // The part of the cost based on the reference state.
      // 1. cte 2. epsi 3. velocity
      for (size_t t = 0; t < N; t++) {
        fg[0] += weight_cte * CppAD::pow(vars[cte_start + t] - ref_cte, 2);
        fg[0] += weight_epsi * CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
        fg[0] += weight_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
      }

      // Minimize the use of actuators.
      for (size_t t = 0; t < N - 1; t++) {
        fg[0] += weight_steering * CppAD::pow(vars[delta_start + t], 2);
        fg[0] += weight_acceleration * CppAD::pow(vars[a_start + t], 2);
      }

      // Minimize the value gap between sequential actuations.
      for (size_t t = 0; t < N - 2; t++) {
        fg[0] += 500 *  CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        fg[0] += 1 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      }
    }

     void setUpConstraints(ADvector& fg, const ADvector& vars)
     {
      //
      // Setup Constraints
      //
      // NOTE: In this section you'll setup the model constraints.

      // Initial constraints
      //
      // We add 1 to each of the starting indices due to cost being located at
      // index 0 of `fg`.
      // This bumps up the position of all the other values.
      fg[1 + x_start] = vars[x_start];
      fg[1 + y_start] = vars[y_start];
      fg[1 + psi_start] = vars[psi_start];
      fg[1 + v_start] = vars[v_start];
      fg[1 + cte_start] = vars[cte_start];
      fg[1 + epsi_start] = vars[epsi_start];

      // The rest of the constraints
      for (size_t t = 1; t < N; t++) {
        // The state at time t+1 .
        AD<double> x1 = vars[x_start + t];
        AD<double> y1 = vars[y_start + t];
        AD<double> psi1 = vars[psi_start + t];
        AD<double> v1 = vars[v_start + t];
        AD<double> cte1 = vars[cte_start + t];
        AD<double> epsi1 = vars[epsi_start + t];

        // The state at time t.
        AD<double> x0 = vars[x_start + t - 1];
        AD<double> y0 = vars[y_start + t - 1];
        AD<double> psi0 = vars[psi_start + t - 1];
        AD<double> v0 = vars[v_start + t - 1];
        AD<double> cte0 = vars[cte_start + t - 1];
        AD<double> epsi0 = vars[epsi_start + t - 1];

        // Only consider the actuation at time t.
        AD<double> steering0 = vars[delta_start + t - 1];
        AD<double> a0 = vars[a_start + t - 1];

        
     	AD<double> f0 = this->coeffs[0] + this->coeffs[1] * x0 + this->coeffs[2] * CppAD::pow(x0, 2) + this->coeffs[3] * CppAD::pow(x0, 3); // 3rd order fitting
        AD<double> psides0 = CppAD::atan(this->coeffs[1] + 2 * this->coeffs[2] * x0 + 3 * this->coeffs[3] * CppAD::pow(x0, 2));

        // Here's `x` to get you started.
        // The idea here is to constraint this value to be 0.
        //
        // Recall the equations for the model:
        // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
        // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
        // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
        // v_[t+1] = v[t] + a[t] * dt
        // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
        // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
        fg[1 + x_start + t]   = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
        fg[1 + y_start + t]   = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
        fg[1 + psi_start + t] = psi1 - (psi0 + v0 * steering0 / Lf * dt);
        fg[1 + v_start + t]   = v1 - (v0 + a0 * dt);
        fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
        fg[1 + epsi_start + t]= epsi1 - ((psi0 - psides0) + v0 * steering0 / Lf * dt);
      }
     }
     
public: 
    Eigen::VectorXd coeffs;  // Fitted polynomial coefficients
};


/////////////////////////////////////////////////////////////////////////////////////////////
MPC::MPC() {}

/////////////////////////////////////////////////////////////////////////////////////////////
MPC::~MPC() {}

/////////////////////////////////////////////////////////////////////////////////////////////
vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  
  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = 6 * N;
  
  /* Init the vars */
  Dvector vars(n_vars);
   cout<<"vars.size="<<vars.size()<<endl;
  initVars(vars, state);
  
  /* */
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  cout<<"vars_lowerbound.size="<<vars_lowerbound.size()<<endl;
  setVarsLimitRange(vars_lowerbound, vars_upperbound);

  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  cout<<"constraints_lowerbound.size="<<constraints_lowerbound.size()<<endl;
  initConstraints(constraints_lowerbound, constraints_upperbound, state);

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  
  mpc_x.clear();
  mpc_y.clear();
  mpc_x.resize(N, 0);
  mpc_y.resize(N, 0);
  // Store in member variable
  for(size_t i = 0; i < N; ++i)
  {
    mpc_x[i] = solution.x[x_start + i];
    mpc_y[i] = solution.x[y_start + i];
  }

  return {solution.x[delta_start], solution.x[a_start]};
}

/////////////////////////////////////////////////////////////////////////////////////////////
void MPC::initVars(Dvector& vars, const Eigen::VectorXd state)
{
  // Initial value of the independent variables.
  // Should be 0 except for the initial values.
  for (size_t i = 0; i < vars.size(); i++) {
    vars[i] = 0;
  }

    // Set the initial variable values
  vars[x_start]   =  state[0];  // x
  vars[y_start]   =  state[1];  // y
  vars[psi_start] =  state[2];  // psi
  vars[v_start]   =  state[3];  // v
  vars[cte_start] =  state[4];  // cte
  vars[epsi_start]=  state[5];  // epsi

}

/////////////////////////////////////////////////////////////////////////////////////////////
void MPC::setVarsLimitRange(Dvector& vars_lowerbound, Dvector& vars_upperbound)
{
  // Set all non-actuators upper and lower limits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] =  1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = - 25 * 0.01745329;
    vars_upperbound[i] =  25 * 0.01745329;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < vars_lowerbound.size(); i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void MPC::initConstraints(Dvector& constraints_lowerbound, Dvector& constraints_upperbound, const Eigen::VectorXd state)
{
    // Lower and upper limits for constraints
    // All of these should be 0 except the initial
    // state indices.
    for (size_t i = 0; i < constraints_lowerbound.size(); i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

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
    
}

const std::vector<double>& MPC::getMpcX() const
{
    return mpc_x; 
}


const std::vector<double>& MPC::getMpcY() const
{
    return mpc_y; 
}