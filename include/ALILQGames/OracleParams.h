#pragma once

using Eigen::VectorXd;

/*
    Struct for high-level (orcacle) parameters
*/

struct OracleParams
{
    VectorXd x0goal;                            // initial state
    VectorXd xfgoal;                         // final state
    VectorXd RotGoalOrigin;
    int n_agents;
    bool GoalisChanging = false;            // Is the goal changing or not
};
