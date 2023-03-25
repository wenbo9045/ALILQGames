#pragma once

#include "cost.h"

class DiffDriveCost : public Cost {
    
    public:
        // (i.e. for player i = 1): Rij = [R11 R12 R13 ... R1N] nu x nplayers*nu or diagm(Rij)
        DiffDriveCost(MatrixXd Qi, MatrixXd QNi, MatrixXd Rij, VectorXd xgoalin)
        {
        
            Q = Qi;
            QN = QNi;
            R = Rij;
            xgoal = xgoalin;
            n_dims = Q.rows();
            m_dims = R.rows();
        }


        // TODO: Shouldn't be cost relative to goal
        double StageCost(const int i, const VectorXd &x, const VectorXd &u) override{
            // double input_cost;
            // for ()
            return (0.5*(x - xgoal).transpose()*Q*(x - xgoal) + 0.5*u.transpose()*R*u).sum();       // returns a 1x1 matrix?
        }

        double TerminalCost(const int i, const VectorXd &x) override{
            return (0.5*(x - xgoal).transpose()*QN*(x - xgoal)).sum();
        }

        void StageCostGradient(const int i, VectorXd &lx, VectorXd &lu, const VectorXd& x, const VectorXd& u) override{
            assert(lx.rows() == n_dims);
            assert(lx.cols() == 1);
            assert(lu.rows() == m_dims);
            assert(lu.cols() == 1); 

            lx = Q*(x - xgoal);
            lu = R*u;
        }

        void TerminalCostGradient(const int i, VectorXd &lx, const VectorXd& x) override{
            assert(lx.rows() == n_dims);

            lx = QN*(x -xgoal);
        }

        void StageCostHessian(const int i, MatrixXd &lxx, MatrixXd &luu, const VectorXd& x, const VectorXd& u) override {
            assert(lxx.rows() == n_dims);
            assert(lxx.cols() == n_dims);
            assert(luu.rows() == m_dims);
            assert(luu.cols() == m_dims);

            lxx = Q;
            luu = R;         

        }

        void TerminalCostHessian(const int i, MatrixXd &lxx, const VectorXd& x) override {
            assert(lxx.rows() == n_dims);
            lxx = QN;
        }

    private:
        MatrixXd Q;
        MatrixXd QN;
        MatrixXd R;
        VectorXd xgoal;
        int n_dims;
        int m_dims;

}; 