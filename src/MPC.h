#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// TODO: Set the timestep length and duration
// const size_t N = 25;  //crash 
// const size_t N = 12;  //ok
// const size_t N = 19;  //ok, not stable
const size_t N = 16;     
const double dt = 0.05;
// const int latency_idx = 1; //latency in units of dt (100ms) // crash
const int latency_idx = 2; //latency in units of dt (100ms)

struct Solution {

		vector<double> x;
		vector<double> y;
		vector<double> psi;
		vector<double> v;
		vector<double> cte;
		vector<double> epsi;
		vector<double> delta;
		vector<double> a;
};

class MPC {

 public:

 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  // vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  double delta_prev {0};
  double a_prev {0.1};  
    
};

#endif /* MPC_H */
