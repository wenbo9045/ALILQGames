#pragma once

struct SolverParams
{
    int nx;                             // number of states for one agent
    int nu;                             // number of inputs for one agent
    int n_agents;                       // number of agents
    int Nx;                             // total number of concatenated states
    int Nu;                             // total number of controls
    int p_inq;                          // number of inequality constraints
    double tol = 1e-4;                  // Solver convergence tolerance
    double penalty_scale = 1.5;         // penalty schedule for augmented lagrangian

};
