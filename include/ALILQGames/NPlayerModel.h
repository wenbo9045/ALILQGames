#pragma once
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>
#include <cassert>  
#include "model.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;


struct NPlayerModel
{
    Model model;
    int MPlayers;                 // Total Number of players
    int nx;                       // Single agent number of states inputs (TODO: should change this for hetero)
    int nu;                       // Single agent number of control inputs (TODO: should change this for hetero)
    int Nx;                       // Total Number of states
    int Nu;                       // Total Number of control inputs
    double dt;

    // ~Model() {}

    virtual VectorXd dynamics_concat(const VectorXd &x, const VectorXd &u) = 0;          // dynamics function, arguments are constant references of the state and control
    // = 0, It's a pure virtual function. It makes it so you MUST derive a class (and implement said function) in order to use it.

    virtual void dynamicsJacobConcat(MatrixXd &fx, MatrixXd &fu, const VectorXd& x, const VectorXd& u) = 0;

    // pass-by-reference if you want to modify the argument value in the calling function
    VectorXd RK4(const VectorXd &x, const VectorXd &u, double dt){
        VectorXd k1 = dynamics(x, u);
        VectorXd k2 = dynamics(x + 0.5*dt*k1, u); 
        VectorXd k3 = dynamics(x + 0.5*dt*k2, u); 
        VectorXd k4 = dynamics(x + dt*k3, u); 

        return (x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4));
    };

};