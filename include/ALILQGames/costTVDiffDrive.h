#pragma once

#include "cost.h"
#include "OracleParams.h"
#include "SolverParams.h"

/**
 * @brief Time Varying Cost for differential drive model
 * I'm only updating the parameter alpha, in setGoal()
 * For passing Rs: (i.e. for player i = 1): Rij = [R11 R12 R13 ... R1N] Nu x nplayers*Nu or diagm(Rij)
 */

class DiffDriveTVCost : public Cost {
    
    public:
        DiffDriveTVCost(SolverParams& params, MatrixXd Qi, MatrixXd QNi, MatrixXd Rij)
        {
        
            Q = Qi;
            QN = QNi;
            R = Rij;
            xgoal = params.xgoal;
            isGoalChanging = params.isGoalChanging;
            alpha = params.tv_cost_alpha;

            Nx = Q.rows();
            Nu = R.rows();
        }

        double TotalCost(const int i, const int H, const std::vector<VectorXd>& x, const std::vector<VectorXd>& u) override{
            double cost = 0.0;

            for (int k=0; k < H-1; k++)
            {
                cost += StageCost(i, x[k], u[k]);
            }
            return cost;
        }


        // TODO: Shouldn't be cost relative to goal
        double StageCost(const int i, const VectorXd &x, const VectorXd &u) override{
            // double input_cost;
            // for ()
            return (0.5*(x - xgoal).transpose()*Q*(x - xgoal) + 0.5*u.transpose()*R*u).sum();       // returns a 1x1 matrix?
        }

        double TerminalCost(const int i, const VectorXd &x) override{
            return (0.5*(x - xgoal).transpose()*alpha*QN*(x - xgoal)).sum();
        }

        void StageCostGradient(const int i, VectorXd &lx, VectorXd &lu, const VectorXd& x, const VectorXd& u) override{
            assert(lx.rows() == Nx);
            assert(lx.cols() == 1);
            assert(lu.rows() == Nu);
            assert(lu.cols() == 1); 

            lx = Q*(x - xgoal);
            lu = R*u;
        }

        void TerminalCostGradient(const int i, VectorXd &lx, const VectorXd& x) override{
            assert(lx.rows() == Nx);

            lx = alpha*QN*(x -xgoal);
        }

        void StageCostHessian(const int i, MatrixXd &lxx, MatrixXd &luu, const VectorXd& x, const VectorXd& u) override {
            assert(lxx.rows() == Nx);
            assert(lxx.cols() == Nx);
            assert(luu.rows() == Nu);
            assert(luu.cols() == Nu);

            lxx = Q;
            luu = R;         

        }

        void TerminalCostHessian(const int i, MatrixXd &lxx, const VectorXd& x) override {
            assert(lxx.rows() == Nx);
            lxx = alpha*QN;
        }

        bool setGoal(const VectorXd& xgoal_in) override{
            if (!isGoalChanging)
            {
                std::cout << " Parameter Goal Changing is set to false!\n";
                return false;
            }
            else 
            {
                xgoal = xgoal_in;
                alpha *= alpha;
                return true;
            }
        }

        VectorXd getGoal() override{
            return xgoal;
        }


    private:
        // Quadratic cost variables 
        MatrixXd Q;
        MatrixXd QN;
        MatrixXd R;
        float alpha;          // parameter for varying the quadratic costs over the horizon

        // Rotating Goal Parameters
        bool isGoalChanging;
        VectorXd xgoal;
        
        int Nx;
        int Nu;
}; 