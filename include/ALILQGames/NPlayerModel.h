#pragma once
//#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <iostream>
#include <cassert>  
#include "model.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

/*
    Base Class for a N agent dynamics model (Now Homogenous)

    dynamics_concat(): base function for concatenating all agents' states
                into one big vector ẋ. 
                It calls model->dynamics function (check model.h)
    
    dynamicsJacobConcat(): base function for concatenating all agents' state and
                and control Jacobian into the following:
                It is a bit different from the documentation.
    
                xₖ₊₁ = [A1  0  0; xₖ + [B1  0   0; [u_1; u_2; u_3]
                       0  A2  0;       0  B2   0;
                       0   0 A3]       0   0  B3]

                It calls model->stateJacob and model->controlJacob function (check model.h)

    RK4(): Integrates dynamics with explicit RK4
            Returns vector of state
*/


class NPlayerModel
{   
    public:
        NPlayerModel(Model* DynModel, int players)
        {
            model.reset(DynModel);
            MPlayers = players;
            nx = model->nx;
            nu = model->nu;
            Nx = model->nx * MPlayers;
            Nu =  model->nu * MPlayers;
            dt = model->dt;
        }

        NPlayerModel();

        std::shared_ptr<Model> model;//model;

        int MPlayers;                 // Total Number of players
        int nx;                       // Single agent number of states inputs (TODO: should change this for hetero)
        int nu;                       // Single agent number of control inputs (TODO: should change this for hetero)
        int Nx;                       // Total Number of states
        int Nu;                       // Total Number of control inputs
        double dt;

        // ~Model() {}

        // virtual VectorXd dynamics_concat(const VectorXd &x, const VectorXd &u) = 0;          // dynamics function, arguments are constant references of the state and control
        // // = 0, It's a pure virtual function. It makes it so you MUST derive a class (and implement said function) in order to use it.

        // virtual void dynamicsJacobConcat(MatrixXd &fx, MatrixXd &fu, const VectorXd& x, const VectorXd& u) = 0;

        VectorXd dynamics_concat(const VectorXd& x, const VectorXd& u);    

        void dynamicsJacobConcat(MatrixXd &fx, MatrixXd &fu, const VectorXd& x, const VectorXd& u);

        // pass-by-reference if you want to modify the argument value in the calling function
        VectorXd RK4(const VectorXd &x, const VectorXd &u, const double dt){
            const VectorXd k1 = dynamics_concat(x, u);
            const VectorXd k2 = dynamics_concat(x + 0.5*dt*k1, u); 
            const VectorXd k3 = dynamics_concat(x + 0.5*dt*k2, u); 
            const VectorXd k4 = dynamics_concat(x + dt*k3, u); 

            return (x + (dt/6.0)*(k1 + 2*k2 + 2*k3 + k4));
        };

};