#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <cassert>  

using Eigen::VectorXd;
using Eigen::MatrixXd;

struct Cost
{
    virtual ~Cost() = default;

    virtual double StageCost(const VectorXd &x, const VectorXd &u) = 0;                          // cost function

    virtual double TerminalCost(const VectorXd &x) = 0;                                          // cost function

    //virtual double TotalCost(const VectorXd &x, const VectorXd &u) = 0;                        // cost function

    virtual void StageCostGradient(VectorXd &lx, VectorXd &lu, const VectorXd& x, const VectorXd& u) = 0;

    virtual void TerminalCostGradient(VectorXd &lx, const VectorXd& x) = 0;

    virtual void StageCostHessian(MatrixXd &lxx, MatrixXd &luu, const VectorXd& x, const VectorXd& u) = 0;

    virtual void TerminalCostHessian(MatrixXd &lxx, const VectorXd& x) = 0;

};