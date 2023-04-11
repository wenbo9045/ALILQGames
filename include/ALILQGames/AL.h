#pragma once
#include "GlobalConstraints.h"
#include <iostream>
#include <vector>
#include <memory>
#include "SolverParams.h"

using namespace std;

class AL
{
    public:
        
        AL(SolverParams& params, vector<shared_ptr<GlobalConstraints>> ptr_constr) : ptr_cons(move(ptr_constr))    
        {
            I_mu = MatrixXd::Zero(params.p_inq, params.p_inq);
            c = VectorXd::Zero(params.p_inq);
            cx = MatrixXd::Zero(params.p_inq, params.Nx);
            cu = MatrixXd::Zero(params.p_inq, params.Nu);
            penalty_scale = params.penalty_scale;

            mu_original = params.penalty;
            mu = params.penalty;
            
            lambda.resize(params.H);

            // Fill multiplier for each time step 
            for(int k=0; k < params.H; k++)                                     //Maybe do std::fill                 
            {
                lambda[k] = VectorXd::Zero(params.p_inq);                                  // State vector is n dimensional
            } 
        }

        vector<shared_ptr<GlobalConstraints>> ptr_cons;  // Shared pointer to the constraints

        AL();                                           // Constructor

        double Merit(const int k, const int i, const double l, 
                        const VectorXd& x, const VectorXd& u);

        void ALGradHess(const int k, MatrixXd& Lxx, MatrixXd& Luu, MatrixXd& Lux,
                VectorXd& Lx, VectorXd& Lu, const MatrixXd& lxx, const MatrixXd& luu,
                const MatrixXd& lux, const VectorXd& lx, 
                const VectorXd& lu, const VectorXd& x, const VectorXd& u);

        // void ALGrad(VectorXd& Lx, VectorXd& Lu, const VectorXd& lx, 
        //         const VectorXd& lu,const VectorXd& x, const VectorXd& u);

        // void ALHess(MatrixXd& Lxx, MatrixXd& Luu, MatrixXd& Lux, 
        //         const VectorXd& lxx, const VectorXd& luu, const VectorXd& lux);
        
        void ActiveConstraint(const int k);

        void ConcatConstraint(const VectorXd& x, const VectorXd& u);

        double MaxConstraintViolation(const VectorXd& x, const VectorXd& u);

        void DualUpdate(const vector<VectorXd>& x_k, const vector<VectorXd>& u_k);
        
        void PenaltySchedule();

        void ResetDual();

        void ResetPenalty();

        VectorXd GetDual(const int k);
        // void ConcatConstraintJacob(const VectorXd& x, const VectorXd& u);


    private:
        double mu_original;                             // is used for reseting the penalty
        double mu;
        MatrixXd I_mu;                                  // inq vs inq where inq is number of inequality constraints
        int inq;
        MatrixXd cx,cu;
        VectorXd c;
        vector<VectorXd> lambda;               // Dual variable for ALL agents (here we are assuming no equality constraints)
        double penalty_scale;
};