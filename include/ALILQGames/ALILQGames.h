#pragma once
#include "NPlayerModel.h"
#include <iostream>
#include <vector>
#include <memory>
#include "cost.h"

class ALILQGames
{
    public:
        // Pass a Nplayer Model system, and a vector of costs of each player
        ALILQGames(NPlayerModel* ptr_model, std::vector<Cost*> ptr_costs)
        {
            Nmodel.reset(ptr_model);                 // reset shared pointer to the model 
            Ncost = ptr_costs;
            dt = Nmodel->dt;
        }

        std::shared_ptr<NPlayerModel> Nmodel;                   // N player model;

        std::vector<Cost*> Ncost;                   // Might make it a shared pointer or pass by reference
        //std::shared_ptr<Cost> cost;

        ALILQGames();                                     // Constructor

        void init(int Horizon);

        void initial_rollout(const VectorXd& x0);

        void forward_rollout(const VectorXd& x0);

        void backward_pass();

        void solve(const VectorXd& x0);

        void recedingHorizon(const VectorXd& x0);

        VectorXd getState(int k);

        double dt;                                   // delta t

        int H;                                      // Horizon length

        double iter_cost;

    private:

        std::vector<VectorXd> x_k;           // States over the entire Horizon 

        std::vector<VectorXd> u_k;           // Control inputs over the entire Horizon

        std::vector<VectorXd> x_hat;         // previous state rollout
        std::vector<VectorXd> u_hat;         // previous controls

        std::vector<MatrixXd> K_k;           // Proportional gain

        std::vector<VectorXd> d_k;           // Feedforward Gain (AKA gradient)

        MatrixXd fx, fu;        // Dynamics Jacobian

        std::vector<VectorXd> lx,lu;         // Cost Gradients

        std::vector<MatrixXd> lxx, luu, lux;       // Cost Hessians

        double max_grad;

        
        int nx;                                             // Number of states for agent i
        int nu;                                     // Number of controls for agent i
        int Nx;                                     // Total number of states for all agents
        int Nu;                                     // Total number of controls for all agents
        int n_agents;

        // Backward pass stuff
        std::vector<MatrixXd> P;                                   // value matrix at final step is only cost/state hessian
        std::vector<VectorXd> p;                        
        MatrixXd S(Nu, Nu);                                        // Same as S in the document
        MatrixXd F_k(Nx, Nx);                                      // Placeholder for eqn readibility
        MatrixXd YK(Nu, Nx);                                       // Right hand side of system of linear equations for K                              
        VectorXd Yd(Nu);                                           // Right hand side of system of linear equations for d

};