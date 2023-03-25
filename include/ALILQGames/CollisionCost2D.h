#pragma once

#include "cost.h"

/*
    (r_avoid - distance_between_agents)^2
*/

class CollisionCost2D : public Cost {
    
    public:
        // (i.e. for player i = 1): Rij = [R11 R12 R13 ... R1N] nu x nplayers*nu or diagm(Rij)
        CollisionCost2D(int n_agents, MatrixXd& Qi, MatrixXd& QNi, MatrixXd& Rij, VectorXd& xgoalin, VectorXd& r_avoid, double rho_obs)
        {
            n_ag = n_agents;
            r = r_avoid;
            rho = rho_obs;
            Q = Qi;
            QN = QNi;
            R = Rij;
            xgoal = xgoalin;
            n_dims = Q.rows();
            m_dims = R.rows();
            dist_agents.resize(n_ag);
            dx.resize(n_ag);
            dy.resize(n_ag);
        }


        // TODO: Shouldn't be cost relative to goal
        double StageCost(const int i, const VectorXd &x, const VectorXd &u) override{
            // double input_cost;
            const int nx = x.rows()/n_ag;
            double distance;
            double penalty_coll;
            double dx, dy;
            for (int j=0; j < n_ag; j++)
            {
                if (j != i){ 
                    dx = x(nx*i) - x(nx*j);
                    dy = x(nx*i+1) - x(nx*j+1);

                    distance = std::sqrt(dx*dx + dy*dy);

                    // Check if the distance is within the keepout zone
                    if (distance <= r(j))
                    {
                        penalty_coll += 0.5*rho*(r(j) - distance)*(r(j) - distance);                   // if the agents are closer than the radius, add a penalty
                    }
                }
            }

            return (0.5*(x - xgoal).transpose()*Q*(x - xgoal) + 0.5*u.transpose()*R*u).sum() + penalty_coll;       // returns a 1x1 matrix?
        }

        double TerminalCost(const int i, const VectorXd &x) override{
            return (0.5*(x - xgoal).transpose()*QN*(x - xgoal)).sum();
        }

        void StageCostGradient(const int i, VectorXd &lx, VectorXd &lu, const VectorXd& x, const VectorXd& u) override{
            // assert(lx.rows() == n_dims);
            // assert(lx.cols() == 1);
            // assert(lu.rows() == m_dims);
            // assert(lu.cols() == 1); 

            const int nx = x.rows()/n_ag;
            double distance;
            double penalty_coll;
            double dx, dy;

            // lx = Q*(x - xgoal);

            for (int j=0; j < n_ag; j++)
            {
                if (j != i)
                {
                    dx = (x(nx*i) - x(nx*j));
                    dy = (x(nx*i+1) - x(nx*j+1));

                    distance = std::sqrt(dx*dx + dy*dy);
                    if (distance <= r(j))
                    {
                        lx(nx*i) -= rho * dx * (r(j) - distance)/distance;
                        lx(nx*i + 1) -= rho * dy * (r(j) - distance)/distance;
                        lx(nx*j) += rho * dx * (r(j) - distance)/distance; 
                        lx(nx*j + 1) += rho * dy * (r(j) - distance)/distance;
                    }
                }
            }

            // lu = R*u;
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
        VectorXd r;
        VectorXd xgoal;
        std::vector<double> dx;
        std::vector<double> dy;
        std::vector<double> dist_agents;

        double rho;
        int n_dims;
        int m_dims;
        int n_ag;
}; 