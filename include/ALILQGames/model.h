#pragma once
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>
#include <cassert>  

using Eigen::VectorXd;
using Eigen::MatrixXd;


struct Model
{
    int nx;                  // Number of states
    int mu;                  // Number of control inputs
    double dt;

    // ~Model() {}

    virtual VectorXd dynamics(const VectorXd &x, const VectorXd &u) = 0;          // dynamics function, arguments are constant references of the state and control
    // = 0, It's a pure virtual function. It makes it so you MUST derive a class (and implement said function) in order to use it.

    virtual void stateJacob(MatrixXd &fx, const VectorXd& x, const VectorXd& u) = 0;

    virtual void controlJacob(MatrixXd &fu, const VectorXd& x, const VectorXd& u) = 0;

    // TODO: Add RK4 linearization instead of forward euler


    // pass-by-reference if you want to modify the argument value in the calling function
    VectorXd RK4(const VectorXd &x, const VectorXd &u, double dt){
        VectorXd k1 = dynamics(x, u);
        VectorXd k2 = dynamics(x + 0.5*dt*k1, u); 
        VectorXd k3 = dynamics(x + 0.5*dt*k2, u); 
        VectorXd k4 = dynamics(x + dt*k3, u); 

        return (x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4));
    };

};