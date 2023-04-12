#pragma once

#include "cost.h"
#include "OracleParams.h"

class DiffDriveCost : public Cost {
    
    public:
        // (i.e. for player i = 1): Rij = [R11 R12 R13 ... R1N] Nu x nplayers*Nu or diagm(Rij)
        DiffDriveCost(OracleParams& oracleparams, MatrixXd Qi, MatrixXd QNi, MatrixXd Rij)
        {
        
            Q = Qi;
            QN = QNi;
            R = Rij;
            xgoal = oracleparams.x0goal;

            if (oracleparams.GoalisChanging){
                xfgoal = oracleparams.xfgoal;
                x0goal = oracleparams.x0goal;
                GoalOrigin = oracleparams.RotGoalOrigin;
                const float dx = GoalOrigin(0) - x0goal(0);
                const float dy = GoalOrigin(1) - x0goal(1);
                Radius = std::sqrt(dx*dx + dy*dy);

                nx = xfgoal.rows()/oracleparams.n_agents;

                phi.resize(oracleparams.n_agents);
            }
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
            return (0.5*(x - xgoal).transpose()*QN*(x - xgoal)).sum();
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

            lx = QN*(x -xgoal);
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
            lxx = QN;
        }

        VectorXd NAgentGoalChange(int k) override
        {
            const float kf = 450.0;                         // where the final goal will be at time tf

            if (k > 450.0)
                k = 450.0;  
            
            // x1(t) = Rcos(omega1*t + phi1)
            // y1(t) = Rsin(omega1*t + phi2)
            // Initial and Final Conditions x1(0) = x0goal[1], y1(0) = x0goal[2]
        
            for (std::size_t i = 0; i < phi.size(); i++)
            {
                const int inx = i*nx;

                // std::cout << GoalOrigin << "\n";

                phi[i] = acos((x0goal(inx) - GoalOrigin(0))/Radius);
                const double omega = (acos((xfgoal(0) - GoalOrigin(0))/Radius) - phi[i])/kf;

                // X coordinate of goal
                xgoal(inx) = Radius*cos(omega*k + phi[i]) + GoalOrigin(0);
                // Y coordinate of goal
                xgoal(1+ inx) = Radius*sin(omega*k + phi[i]) + GoalOrigin(1); 

            }

            return xgoal;

        }


    private:
        // Quadratic cost variables 
        MatrixXd Q;
        MatrixXd QN;
        MatrixXd R;
        float alpha = 0.2;          // parameter for varying the quadratic costs over the horizon

        // Rotating Goal Parameters
        vector<double> phi;
        VectorXd x0goal;
        VectorXd xgoal;
        VectorXd xfgoal;
        VectorXd GoalOrigin;
        double Radius;     
        
        int Nx;
        int Nu;

        int nx;

}; 