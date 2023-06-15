#pragma once
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>
#include <cassert>
#include <memory>

using Eigen::VectorXd;
using Eigen::MatrixXd;

/*
    Base Struct/Class for any Single agent dynamics with feedback linearizable model
*/


struct FeedbackLinearization
{

    virtual ~FeedbackLinearization() = default;

    virtual void decoupling_matrix(MatrixXd& M, const VectorXd& x) = 0;

    virtual void inverse_decoupling_matrix(MatrixXd& Minv, const VectorXd& x) = 0;
    
    virtual void conversion_map(VectorXd& x, const VectorXd& gamma) = 0;

    virtual void inv_conversion_map(VectorXd& gamma, const VectorXd& x) = 0;

};