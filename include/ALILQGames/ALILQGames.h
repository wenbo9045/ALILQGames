#pragma once
#include "NPlayerModel.h"
#include <iostream>
#include <vector>
#include <memory>
#include "cost.h"
#include "SolverParams.h"

class ALILQGames
{
    public:
        // Pass a Nplayer Model system, and a vector of costs of each player

        ALILQGames(SolverParams& params, NPlayerModel* ptr_model, std::vector<std::shared_ptr<Cost>> ptr_costs) : pc(move(ptr_costs))
        {
            Nmodel.reset(ptr_model);                 // reset shared pointer to the model 
            dt = params.dt;
            H = params.H;

            // populate terms for entire horizon
            x_k.resize(H);
            u_k.resize(H-1);
            K_k.resize(H-1);
            d_k.resize(H-1);
            lambda.resize(H);

            alpha = params.alpha;
            tol = params.tol;
            nx = params.nx;                                      // Number of states for agent i
            nu = params.nu;                                      // Number of controls for agent i
            Nx = params.Nx;                                      // Total number of states for all agents
            Nu = params.Nu;                                      // Total number of controls for all agents
            n_agents = params.n_agents;

            // Initialize Jacobians
            fx = MatrixXd::Zero(Nx, Nx);                            // Feedback gain is m by n
            fu = MatrixXd::Zero(Nx, Nu);                            // Feedback gain is m by n

            // Costs
            lx.resize(n_agents);
            lu.resize(n_agents);
            lxx.resize(n_agents);
            luu.resize(n_agents);
            lux.resize(n_agents);

            // Backward pass stuff 
            DeltaV.resize(n_agents);
            p.resize(n_agents);
            P.resize(n_agents);
            S = MatrixXd::Zero(Nu, Nu);                             // Same as S in the document
            F_k = MatrixXd::Zero(Nx, Nx);                           // Placeholder for eqn readibility
            beta_k = VectorXd::Zero(Nx);                         // Placeholder for eqn readibility
            YK = MatrixXd::Zero(Nu, Nx);                            // Right hand side of system of linear equations for K                              
            Yd = VectorXd::Zero(Nu);                                // Right hand side of system of linear equations for d


            for(int i=0; i < n_agents; i++)
            {
                DeltaV[i] = 0.0;
                p[i] = VectorXd::Zero(Nx);
                P[i] = MatrixXd::Zero(Nx, Nx);
                lx[i] = VectorXd::Zero(Nx);                         // gradient of cost wrt x and is m by 1
                lu[i] = VectorXd::Zero(Nu);                         // gradient of cost wrt u and is n by 1
                lxx[i] = MatrixXd::Zero(Nx, Nx);                    // Hessian of cost wrt xx and is n by n
                luu[i] = MatrixXd::Zero(Nu, Nu);                    // Hessian of cost wrt uu and is m by m
                lux[i] = MatrixXd::Zero(Nu, Nx);                    // Hessian of cost wrt ux and is m by n
            }
            

            // Fill concatenated states, controls and policies for entire horizon
            for(int k=0; k < H; k++)                                     //Maybe do std::fill                 
            {
                x_k[k] = VectorXd::Zero(Nx);                                  // State vector is n dimensional
            }

            for(int k=0; k<H-1; k++)
            {
                u_k[k] = 0.0*VectorXd::Random(Nu);                           // Input vector is m dimensional
                K_k[k] = 0.1*MatrixXd::Random(Nu, Nx);                        // Feedback gain is m by n
                d_k[k] = 0.1*VectorXd::Random(Nu);                            // Feedforward is m dimensional
            }
        }

        std::vector<std::shared_ptr<Cost>> pc;  
        std::shared_ptr<NPlayerModel> Nmodel;                   // N player model; // change this


        ALILQGames();                                     // Constructor

        // void init(int Horizon);

        void initial_rollout(const VectorXd& x0);

        void forward_rollout(const VectorXd& x0);

        void backward_pass();

        void BackTrackingLineSearch(const VectorXd& x0);

        void ArmuijoLineSearch(const VectorXd& x0);

        void solve(const VectorXd& x0);

        void recedingHorizon(const VectorXd& x0);

        VectorXd getState(int k);

        VectorXd getControl(int k);

        double dt;                                   // delta t

        int H;                                      // Horizon length

        int iter_;

        double iter_cost, total_cost;

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

        double alpha, tol;

        
        int nx;                                             // Number of states for agent i
        int nu;                                     // Number of controls for agent i
        int Nx;                                     // Total number of states for all agents
        int Nu;                                     // Total number of controls for all agents
        int n_agents;

        // Backward pass stuff
        std::vector<MatrixXd> P;                                   // value matrix at final step is only cost/state hessian
        std::vector<VectorXd> p;
        std::vector<double> DeltaV, cost, cost_now;                  // Expected change in cost (used for linesearch)
        MatrixXd S;                                        // Same as S in the document
        MatrixXd F_k;                                      // Placeholder for eqn readibility
        VectorXd beta_k;
        MatrixXd YK;                                       // Right hand side of system of linear equations for K                              
        VectorXd Yd;                                           // Right hand side of system of linear equations for d

};