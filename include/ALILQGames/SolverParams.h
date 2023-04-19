#pragma once
using Eigen::VectorXd;

/*
    Struct for Solver (low-level) parameters
*/

struct SolverParams
{

    // General Parameters
    int H = 100;                        // horizon length (MPC or Entire solution)
    int H_all = 400;                    // Entire horizon length (mainly used for MPC)
    float dt = 0.1;                     // discretization time
    bool MPC = false;                   // Solve in RH fashion or no
    int nx;                             // number of states for one agent
    int nu;                             // number of inputs for one agent
    int n_agents;                       // number of agents
    int Nx;                             // total number of concatenated states
    int Nu;                             // total number of controls
    VectorXd x0;                        // initial state

    // ILQGames Solver params
    int max_iter_ilq = 100;             // Maximum number of iterations for ILQGames before terminating
    double grad_tol = 1e-2;             // Solver gradient convergence tolerance
    double cost_tol = 1e-2;             // Solver cost convergence tolerance: cost_now - cost_prev
    float rho_obs = 500.0;              // penalty for obstacle
    float alpha = 1.0;                  // linesearch parameter

    // Augmented Lagrangian params
    int max_iter_al = 20;               // Maximum number of iterations for the AL before terminating
    int p_inq;                          // number of inequality constraints
    double max_constraint_violation = 1e-2;
    double penalty = 10.0;              // penalty value for augmented lagrangian
    double penalty_scale = 1.5;         // penalty schedule for augmented lagrangian
    int reset_schedule = 20;          // resetting dual variable every number of iterations (for MPC)

    bool isGoalChanging = false;
};
