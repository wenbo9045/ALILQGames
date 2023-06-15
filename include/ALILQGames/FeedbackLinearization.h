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

    virtual MatrixXd decoupling_matrix(const VectorXd& x) = 0;

    virtual MatrixXd inverse_decoupling_matrix(const VectorXd& x) = 0;
    
    virtual VectorXd conversion_map(const VectorXd& zeta) = 0;

    virtual VectorXd inv_conversion_map(const VectorXd& x) = 0;

};