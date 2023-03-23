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
        // ALILQGames(NPlayerModel* ptr_model, std::vector<Cost*> ptr_costs)
        // {
        //     Nmodel.reset(ptr_model);                 // reset shared pointer to the model 
        //     Ncost = ptr_costs;
        //     dt = Nmodel->dt;
        // }

        // std::shared_ptr<NPlayerModel> Nmodel;                   // N player model;

        // std::vector<Cost*> Ncost;                   // Might make it a shared pointer or pass by reference
        // //std::shared_ptr<Cost> cost;
        ALILQGames(NPlayerModel* ptr_model, std::vector<std::shared_ptr<Cost>> ptr_costs) : pc(move(ptr_costs))
        {
            Nmodel.reset(ptr_model);                 // reset shared pointer to the model 
            dt = Nmodel->dt;
        }

        std::vector<std::shared_ptr<Cost>> pc;  
        std::shared_ptr<NPlayerModel> Nmodel;                   // N player model; // change this


        ALILQGames();                                     // Constructor

        void init(int Horizon);

        void initial_rollout(const VectorXd& x0);

        void forward_rollout(const VectorXd& x0);

        void backward_pass();

        void solve(const VectorXd& x0);

        void recedingHorizon(const VectorXd& x0);

        VectorXd getState(int k);

        VectorXd getControl(int k);

        double dt;                                   // delta t

        int H;                                      // Horizon length

        double iter_cost;

    private:

        std::vector<VectorXd> x_k;                  // States over the entire Horizon 

        std::vector<VectorXd> u_k;                  // Control inputs over the entire Horizon

        std::vector<VectorXd> lambda;               // Dual variable for ALL agents (here we are assuming no equality constraints)

        VectorXd c;                                 // inequality constraints

        MatrixXd cx, cu;                             // constraint jacobian

        std::vector<VectorXd> x_hat;                // previous state rollout
        std::vector<VectorXd> u_hat;                // previous controls

        std::vector<MatrixXd> K_k;                  // Proportional gain

        std::vector<VectorXd> d_k;                  // Feedforward Gain (AKA gradient)

        MatrixXd fx, fu;                            // Dynamics Jacobian

        std::vector<VectorXd> lx,lu;                // Cost Gradients

        std::vector<MatrixXd> lxx, luu, lux;        // Cost Hessians

        std::vector<VectorXd> Lx, Lu;               // Augmented Lagrangian gradients

        std::vector<MatrixXd> Lxx, Luu, Lux;        // Augmented Lagrangian hessian

        double max_grad;

        
        int nx;                                             // Number of states for agent i
        int nu;                                     // Number of controls for agent i
        int Nx;                                     // Total number of states for all agents
        int Nu;                                     // Total number of controls for all agents
        int n_agents;

        // Backward pass stuff
        std::vector<MatrixXd> P;                                   // value matrix at final step is only cost/state hessian
        std::vector<VectorXd> p;                        
        MatrixXd S;                                        // Same as S in the document
        MatrixXd F_k;                                      // Placeholder for eqn readibility
        VectorXd beta_k;
        MatrixXd YK;                                       // Right hand side of system of linear equations for K                              
        VectorXd Yd;                                           // Right hand side of system of linear equations for d

};