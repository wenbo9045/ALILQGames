#pragma once

#include "cost.h"

/*
    (r_avoid - distance_between_agents)^2
*/

class CollisionCost2D : public Cost {
    
    public:
        // (i.e. for player i = 1): Rij = [R11 R12 R13 ... R1N] nu x nplayers*nu or diagm(Rij)
        CollisionCost2D(SolverParams& params, MatrixXd& Qi, MatrixXd& QNi, MatrixXd& Rij, VectorXd& xgoalin, VectorXd& r_avoid)
        {
            n_ag = params.n_agents;
            r = r_avoid;
            rho = params.rho_obs;
            Q = Qi;
            QN = QNi;
            R = Rij;
            xgoal = xgoalin;
            nx = params.nx;
            nu = params.nu;
            // n_dims = Q.rows();
            // m_dims = R.rows();
            dist_agents.resize(n_ag);
            dx.resize(n_ag);
            dy.resize(n_ag);
        }

        double TotalCost(const int i, const int H, const std::vector<VectorXd>& x, const std::vector<VectorXd>& u) override{
            double cost = 0.0;

            // Add up stage cost
            for (int k=0; k < H-2; k++)
            {
                cost += StageCost(i, x[k], u[k]);
            }

            // if (length(x) )
            cost += TerminalCost(i, x[H-1]);
            return cost;
        }


        // TODO: Shouldn't be cost relative to goal
        double StageCost(const int i, const VectorXd &x, const VectorXd &u) override{
            // double input_cost;

            double distance;
            double penalty_coll = 0.0;
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

            double distance;
            double dx, dy;

            lx = Q*(x - xgoal);
            lu = R*u;

            for (int j=0; j < n_ag; j++)
            {
                if (j != i)
                {
                    dx = (x(nx*i) - x(nx*j));
                    dy = (x(nx*i+1) - x(nx*j+1));

                    distance = std::sqrt(dx*dx + dy*dy);
                    if (distance <= r(j))
                    {
                        if (distance < 0.001)
                        {
                            dx += 0.01;
                            dy += 0.01;
                            distance = std::sqrt(dx*dx + dy*dy);
                        }   
                        lx(nx*i) -= rho * dx * (r(j) - distance)/distance;
                        lx(nx*i + 1) -= rho * dy * (r(j) - distance)/distance;
                        lx(nx*j) += rho * dx * (r(j) - distance)/distance; 
                        lx(nx*j + 1) += rho * dy * (r(j) - distance)/distance;
                    }
                }
            }
        }

        void TerminalCostGradient(const int i, VectorXd &lx, const VectorXd& x) override{
            assert(lx.rows() == x.rows());

            lx = QN*(x -xgoal);
        }

        void StageCostHessian(const int i, MatrixXd &lxx, MatrixXd &luu, const VectorXd& x, const VectorXd& u) override {
            // assert(lxx.rows() == n_dims);
            // assert(lxx.cols() == n_dims);
            // assert(luu.rows() == m_dims);
            // assert(luu.cols() == m_dims);

            double distance;
            double dx, dy;

            lxx = Q;
            luu = R;  

            for (int j=0; j < n_ag; j++)
            {
                if (j != i)
                {
                    dx = (x(nx*i) - x(nx*j));
                    dy = (x(nx*i+1) - x(nx*j+1));

                    distance = std::sqrt(dx*dx + dy*dy);
                    if (distance <= r(j))
                    {

                        if (distance < 0.001)
                        {
                            dx += 0.01;
                            dy += 0.01;
                            distance = std::sqrt(dx*dx + dy*dy);
                        }
                        int xi = nx*i;
                        int yi = nx*i +1;
                        int xj = nx*j;
                        int yj = nx*j + 1;
                        float dxdx = dx*dx;
                        float dydy = dy*dy;
                        float dxdy = dx*dy;
                        float denom = dxdx + dydy;

                        lxx(xi, xi) -= rho * (r(j) * (distance - dxdx/distance)/(denom) - 1);                   // ∂²c/∂xᵢ∂xᵢ 
                        lxx(yi, yi) -= rho * (r(j) * (distance - dydy/distance)/(denom) - 1);                   // ∂²c/∂yᵢ∂yᵢ
                        lxx(xi, yi) -= rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂xᵢ∂yᵢ
                        lxx(yi, xi) = lxx(xi, yi);
                        // lxx(yi, xi) -= rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂yᵢ∂xᵢ, lxx(nx*i, nx*i+1);
                        
                        lxx(xi, xj) += rho * (r(j) * (distance - dxdx/distance)/(denom) - 1);                   // ∂²c/∂xᵢ∂xⱼ 
                        lxx(xj, xi) += rho * (r(j) * (distance - dxdx/distance)/(denom) - 1);                   // ∂²c/∂xⱼ∂xᵢ, lxx(nx*i, nx*j);
                        lxx(xi, yj) += rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂xᵢ∂yⱼ
                        lxx(yj, xi) = lxx(xi, yj);
                        // lxx(yj, xi) += rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂yⱼ∂xᵢ, lxx(nx*i, nx*j + 1) 

                        lxx(yi, xj) += rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂yᵢ∂xⱼ       
                        lxx(xj, yi) = lxx(yi, xj);
                        // lxx(xj, yi) += rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂xⱼ∂yᵢ, lxx(nx*i+1,nx*j)
                        
                        lxx(xj, xj) -= rho * (r(j) * (distance - dxdx/distance)/(denom) - 1);                   // ∂²c/∂xⱼ∂xⱼ    
                        lxx(yj, yj) -= rho * (r(j) * (distance - dydy/distance)/(denom) - 1);                   // ∂²c/∂yⱼ∂yⱼ
                        
                        lxx(yj, xj) -= rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂yⱼ∂xⱼ
                        lxx(xj, yj) = lxx(yj, xj);
                        // lxx(xj, yj) -= rho * (r(j) * (-dxdy/distance)/(denom));                                 // ∂²c/∂xⱼ∂yⱼ, lxx(nx*j + 1, nx*j)
                                                                                                                        
                        lxx(yi, yj) -= rho * (r(j) * (-distance + dydy/distance)/(denom) + 1);                  // ∂²c/∂yⱼ∂yᵢ
                        lxx(yj, yi) = lxx(yi, yj);
                        // lxx(yj, yi) -= rho * (r(j) * (-distance + dydy/distance)/(denom) + 1);                  // ∂²c/∂yᵢ∂yⱼ,  lxx(nx*i + 1, nx*j + 1)


                        // lxx(nx*i, nx*i) -= rho * (r(j) * (distance - dx*dx/distance)/(dx*dx + dy*dy) - 1);              // ∂²c/∂xᵢ∂xᵢ 
                        // lxx(nx*i + 1, nx*i + 1) -= rho * (r(j) * (distance - dy*dy/distance)/(dx*dx + dy*dy) - 1);      // ∂²c/∂yᵢ∂yᵢ
                        // lxx(nx*i, nx*i+1) -= rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                          // ∂²c/∂xᵢ∂yᵢ
                        // lxx(nx*i+1, nx*i) -= rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                          // ∂²c/∂yᵢ∂xᵢ, lxx(nx*i, nx*i+1);
                        
                        // lxx(nx*i, nx*j) += rho * (r(j) * (distance - dx*dx/distance)/(dx*dx + dy*dy) - 1);              // ∂²c/∂xᵢ∂xⱼ 
                        // lxx(nx*j,nx*i) += rho * (r(j) * (distance - dx*dx/distance)/(dx*dx + dy*dy) - 1);               // ∂²c/∂xⱼ∂xᵢ, lxx(nx*i, nx*j);
                        // lxx(nx*i, nx*j + 1) += rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                        // ∂²c/∂xᵢ∂yⱼ
                        // lxx(nx*j + 1, nx*i) += rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                        // ∂²c/∂yⱼ∂xᵢ, lxx(nx*i, nx*j + 1) 

                        // lxx(nx*i+1,nx*j) += rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                           // ∂²c/∂yᵢ∂xⱼ       
                        // lxx(nx*j,nx*i+1) += rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                           // ∂²c/∂xⱼ∂yᵢ, lxx(nx*i+1,nx*j)
                        
                        // lxx(nx*j, nx*j) -= rho * (r(j) * (distance - dx*dx/distance)/(dx*dx + dy*dy) - 1);              // ∂²c/∂xⱼ∂xⱼ    
                        // lxx(nx*j + 1, nx*j + 1) -= rho * (r(j) * (distance - dy*dy/distance)/(dx*dx + dy*dy) - 1);      // ∂²c/∂yⱼ∂yⱼ
                        
                        // lxx(nx*j + 1, nx*j) -= rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                        // ∂²c/∂yⱼ∂xⱼ
                        // lxx(nx*j, nx*j+1) -= rho * (r(j) * (-dx*dy/distance)/(dx*dx + dy*dy));                          // ∂²c/∂xⱼ∂yⱼ, lxx(nx*j + 1, nx*j)
                                                                                                                        
                        // lxx(nx*i + 1, nx*j + 1) -= rho * (r(j) * (-distance + dy*dy/distance)/(dx*dx + dy*dy) + 1);     // ∂²c/∂yⱼ∂yᵢ
                        // lxx(nx*j + 1, nx*i + 1) -= rho * (r(j) * (-distance + dy*dy/distance)/(dx*dx + dy*dy) + 1);     // ∂²c/∂yᵢ∂yⱼ,  lxx(nx*i + 1, nx*j + 1)

                                                                                                                        // ∂²c/∂yᵢ∂xⱼ              
                    }
                }   
            }
        }

        void TerminalCostHessian(const int i, MatrixXd &lxx, const VectorXd& x) override {
            assert(lxx.rows() == x.rows());
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
        int nx;
        int nu;
        int n_ag;
}; 