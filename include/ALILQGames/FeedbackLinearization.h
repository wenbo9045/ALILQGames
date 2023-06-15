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

    dynamics(): base function for handling continous state space linear/nonlinear models
                with the form ẋ = f(x,u)
    
    stateJacob(): base function that overwrites argument "fx" with the state jacobian 
                Overwrites DISCRETE state Jacobian
    
    controlJacob(): base function that overwrites argument "fu" with the control jacobian 
                Overwrites DISCRETE control Jacobian

    RK4(): Integrates dynamics with explicit RK4
            Returns vector of state
*/


struct FeedbackLinearization
{

    virtual ~FeedbackLinearization() = default;

    virtual VectorXd decoupling_matrix(const VectorXd& x) = 0;

    virtual VectorXd inverse_decoupling_matrix(const VectorXd& x) = 0;
    
    virtual VectorXd map_x(const VectorXd& gamma) = 0;

    virtual VectorXd inverse_map_x(const VectorXd& x) = 0;

};