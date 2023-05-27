#pragma once
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>
#include <cassert>
#include <memory>

using Eigen::VectorXd;
using Eigen::MatrixXd;


/**
    @brief Base Struct/Class for any Single agent dynamics model

    dynamics(): base function for handling continous state space linear/nonlinear models
                with the form ẋ = f(x,u)
    
    stateJacob(): base function that overwrites argument "fx" with the state jacobian 
                Overwrites DISCRETE state Jacobian
    
    controlJacob(): base function that overwrites argument "fu" with the control jacobian 
                Overwrites DISCRETE control Jacobian

    RK4(): Integrates dynamics with explicit RK4
            Returns vector of state
**/


struct Model
{
    int nx;                  // Number of states
    int nu;                  // Number of control inputs
    double dt;

    virtual ~Model() = default;


    virtual VectorXd dynamics(const VectorXd& x, const VectorXd& u) = 0;          // dynamics function, arguments are constant references of the state and control
    // = 0, It's a pure virtual function. It makes it so you MUST derive a class (and implement said function) in order to use it.

    virtual void stateJacob(Eigen::Ref<MatrixXd> fx, const VectorXd& x, const VectorXd& u) = 0;

    virtual void controlJacob(Eigen::Ref<MatrixXd> fu, const VectorXd& x, const VectorXd& u) = 0;

    // virtual VectorXd dynamics(Eigen::Ref<VectorXd> x, Eigen::Ref<VectorXd> u) = 0;          // dynamics function, arguments are constant references of the state and control
    // // = 0, It's a pure virtual function. It makes it so you MUST derive a class (and implement said function) in order to use it.

    // virtual void stateJacob(Eigen::Ref<MatrixXd> fx, Eigen::Ref<VectorXd> x, Eigen::Ref<VectorXd> u) = 0;

    // virtual void controlJacob(Eigen::Ref<MatrixXd> fu, Eigen::Ref<VectorXd> x, Eigen::Ref<VectorXd> u) = 0;
    // TODO: Add RK4 linearization instead of forward euler


    // pass-by-reference if you want to modify the argument value in the calling function
    VectorXd RK4(const VectorXd &x, const VectorXd &u, const double dt){
        const VectorXd k1 = dynamics(x, u);
        const VectorXd k2 = dynamics(x + 0.5*dt*k1, u); 
        const VectorXd k3 = dynamics(x + 0.5*dt*k2, u); 
        const VectorXd k4 = dynamics(x + dt*k3, u); 

        return (x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4));
    };

};