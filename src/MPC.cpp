#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
// #include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using CppAD::AD;

// TODO: Set the timestep length and duration
//size_t N = 0;
//double dt = 0;
//size_t N = 25;
//double dt = 0.05;
// T = (N * dt) seconds aka 25 * 0.05 = 10 seconds of prediction into the future
size_t N = 10;
double dt = 0.1;  // T = (N * dt) seconds aka 10 * 0.1 = 1 second of prediction into the future

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

// NOTE: feel free to play around with this
// or do something completely different
//double ref_v = 40;
double ref_v = 80;
//double ref_v = 120;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    /**
     * Cost is measured by the Cross Track Error (CTE), which is made up of 2 parts :
     *  1. Current CTE : the predicted distance of the vehicle from the trajectory
     *  2. Bearing error : the predicted difference (or angle) between
     *      the vehicle orientation and
     *      the trajectory orientation.
     *
     * Variables aka vars[] :
     *  State
     *      AD<double> x0 = x starting coordinate
     *      AD<double> y0 = y starting coordinate
     *      AD<double> psi0 = vehicle starting yaw angle
     *      AD<double> v0 = vehicle starting velocity
     *      AD<double> cte0 = starting cross track error to the trajectory polynomial
     *      AD<double> epsi0 = starting vehicle bearing error to the trajectory polynomial
     *  Actuation
     *      AD<double> delta0 = starting steering angle
     *      AD<double> a0 = starting acceleration
     *
     * @param fg a vector containing the cost and vehicle model/constraints.
     * @param vars a vector containing the variable values (state & actuators).
     */
    void operator()(ADvector &fg, const ADvector &vars) {
        // TODO: implement MPC
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.

        // The cost is stored in first element of 'fg'
        fg[0] = 0;

        // Tune the cost weights to get a smooth ride!
        const int cte_cost_weight = 2000;
        const int epsi_cost_weight = 2000;
        const int v_cost_weight = 1;
        const int delta_cost_weight = 10;
        const int a_cost_weight = 10;
        const int delta_change_cost_weight = 196418;  // Previous values tried : 4181. 10946. 28657. 75025
        const int a_change_cost_weight = 10946;  // Previous values tried : 89. 610. 1597. 4181

        // Aggregate cost
        for (int t = 0; t < N; t++) {
            fg[0] += cte_cost_weight * CppAD::pow(vars[cte_start + t], 2);  // cross track error
            fg[0] += epsi_cost_weight * CppAD::pow(vars[epsi_start + t], 2);  // heading error
            fg[0] += v_cost_weight * CppAD::pow(vars[v_start + t] - ref_v, 2);  // velocity error
        }

        // Min use of actuators
        // A further enhancement is to constrain erratic control inputs
        // as we want the turns to be smooth and the vehicle velocity not change too radically.
        for (int t = 0; t < N - 1; t++) {
            //fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2);  // steering angle
            fg[0] += delta_cost_weight * CppAD::pow(vars[delta_start + t], 2);  // multiplying steering angle keeps
            // sequential steering values closer together
            // resulting in smoother steering corrections.
            // see Delta section in generated graph
            fg[0] += a_cost_weight * CppAD::pow(vars[a_start + t], 2);  // acceleration
        }

        // Min value gap between sequential actuations
        // Goal here is to make control decisions more consistent or smoother.
        for (int t = 0; t < N - 2; t++) {
            fg[0] += delta_change_cost_weight * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
            fg[0] += a_change_cost_weight * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
        }

        // Initial model constraints
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the model constraints
        for (int t = 1; t < N; t++) {
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
            AD<double> delta0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];

            //AD<double> f0 = coeffs[0] + coeffs[1] * x0;
            AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
            //AD<double> psides0 = CppAD::atan(coeffs[1]);
            AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2));

            // Here's `x` to get you started.
            // The idea here is to constraint this value to be 0.
            //
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            // TODO: Setup the rest of the model constraints
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            //fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            //fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
            fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
        }
    }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    //size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // states
    double x = state[0];
    double y = state[1];
    double psi = state[2];
    double v = state[3];
    double cte = state[4];
    double epsi = state[5];

    // TODO: Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    //
    // 4 * 10 + 2 * 9
    //size_t n_vars = 0;
    size_t n_vars = N * 6 + (N - 1) * 2;
    // TODO: Set the number of constraints
    //size_t n_constraints = 0;
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        //vars[i] = 0;
        vars[i] = 0.0;
    }
    // Set the initial variable values
    //vars[x_start] = x;
    //vars[y_start] = y;
    //vars[psi_start] = psi;
    //vars[v_start] = v;
    //vars[cte_start] = cte;
    //vars[epsi_start] = epsi;

    // Lower and upper limits for state
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // TODO: Set lower and upper limits for variables.
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

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
    //return {};
    vector<double> solved;
    solved.push_back(solution.x[delta_start]);
    solved.push_back(solution.x[a_start]);
    for (int i = 0; i < N; ++i) {
        solved.push_back(solution.x[x_start + i]);
        solved.push_back(solution.x[y_start + i]);
    }

    return solved;

}
