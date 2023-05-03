#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <cassert>  
#include "SolverParams.h"
#include "utils.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

/*
    Base cost class for a SINGLE agent
*/

struct Cost
{

    virtual ~Cost() = default;

    virtual double StageCost(const int i, const VectorXd& x, const VectorXd& u) = 0;                          // cost function

    virtual double TerminalCost(const int i, const VectorXd& x) = 0;

    // virtual double TerminalCost(const int k, const int i, const VectorXd& x) = 0;                                          // cost function
                                            

    virtual double TotalCost(const int i, const int H, const std::vector<VectorXd>& x, const std::vector<VectorXd>& u) = 0;                        // cost function

    virtual void StageCostGradient(const int i, VectorXd& lx, VectorXd& lu, const VectorXd& x, const VectorXd& u) = 0;

    virtual void TerminalCostGradient(const int i, VectorXd& lx, const VectorXd& x) = 0;

    virtual void StageCostHessian(const int i, MatrixXd& lxx, MatrixXd& luu, const VectorXd& x, const VectorXd& u) = 0;

    virtual void TerminalCostHessian(const int i, MatrixXd& lxx, const VectorXd& x) = 0;

    virtual bool setGoal(const VectorXd& xgoal_in) {return false;};

    // Used for live interaction in ImGui
    virtual void BezierCurveGoal(const std::vector<Agent>& agent_pts, const int k, const int H) {};

    // Used for the solver
    virtual void BezierCurveGoal(const int k, const int H) {};

    virtual void setCtrlPts(std::vector<Agent>& agent_pts) {};

    // virtual bool NAgentGoalChange(SolverParams& params, std::vector<float> control_pts) {};

    // Fix this cheap hack to suppress warnings
    virtual VectorXd getGoal() 
    {
        VectorXd xtemp = VectorXd::Zero(8);
        return xtemp;
    };

};