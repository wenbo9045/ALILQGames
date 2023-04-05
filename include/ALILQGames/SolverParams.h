#pragma once

struct SolverParams
{
    int H = 100;                        // horizon length
    float dt = 0.1;                     // discretization time
    int nx;                             // number of states for one agent
    int nu;                             // number of inputs for one agent
    int n_agents;                       // number of agents
    int Nx;                             // total number of concatenated states
    int Nu;                             // total number of controls

    // ILQGames Solver params
    int max_iter_ilq = 100;             // Maximum number of iterations for ILQGames before terminating
    double grad_tol = 1e-3;             // Solver gradient convergence tolerance
    double cost_tol = 1e-3;             // Solver cost convergence tolerance: cost_now - cost_prev
    float rho_obs = 500.0;              // penalty for obstacle
    float alpha = 1.0;                  // linesearch parameter

    // Augmented Lagrangian params
    int max_iter_al = 25;               // Maximum number of iterations for the AL before terminating
    int p_inq;                          // number of inequality constraints
    double max_constraint_violation = 1e-3;
    double penalty = 10.0;              // penalty value for augmented lagrangian
    double penalty_scale = 1.5;         // penalty schedule for augmented lagrangian
};
