#pragma once
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>
#include <cassert>
#include <memory>
#include "SolverParams.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;


/*
    Base Struct/Class for a Solver in this package.
    Current Solvers are ILQGames, ALILQGames.
    All solvers inherit from this base class.
*/


struct Solver
{

    virtual ~Solver() = default;


    virtual double initial_rollout(const VectorXd& x0) = 0;

    virtual double forward_rollout(const VectorXd& x0) = 0;

    virtual double backward_pass(const int k_now) = 0;

    virtual void BackTrackingLineSearch(const VectorXd& x0) {}

    virtual void ArmuijoLineSearch(const VectorXd& x0) {}

    virtual void solve(SolverParams& params, const VectorXd& x0) = 0;

    virtual void recedingHorizon(SolverParams& params, const VectorXd& x0) {}

    virtual void ChangeStrategy(const int i, const float delta) {}

    virtual void MPCWarmStart(SolverParams& params, const VectorXd& x0) {}

    virtual VectorXd getState(const int k) = 0;

    virtual VectorXd getControl(const int k) = 0;

    virtual VectorXd getMPCState(const int k) = 0;

    virtual VectorXd getMPCControl(const int k) = 0;

    virtual double getStageCost(const int i, const int k) = 0;

    virtual double getTerminalCost(const int i) = 0;

    // Fix this cheap hack to remove warnings
    virtual VectorXd getGoalState(const int i, const int k) 
    {
        VectorXd xtemp = VectorXd::Zero(8);
        return xtemp;
    };

};