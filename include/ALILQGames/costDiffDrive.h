#pragma once

#include "cost.h"
#include "OracleParams.h"
#include "SolverParams.h"

class DiffDriveCost : public Cost {
    
    public:
        // (i.e. for player i = 1): Rij = [R11 R12 R13 ... R1N] Nu x nplayers*Nu or diagm(Rij)
        DiffDriveCost(OracleParams& oracleparams, MatrixXd Qi, MatrixXd QNi, MatrixXd Rij)
        {
        
            Q = Qi;
            QN = QNi;
            R = Rij;
            xgoal = oracleparams.x0goal;
            isGoalChanging = oracleparams.GoalisChanging;


            if (isGoalChanging){
                
                // x0goal = oracleparams.x0goal;
                // xfgoal = oracleparams.xfgoal;
                // GoalOrigin = oracleparams.RotGoalOrigin;
                // const float dx = GoalOrigin(0) - x0goal(0);
                // const float dy = GoalOrigin(1) - x0goal(1);
                // Radius = std::sqrt(dx*dx + dy*dy);
                n_agents = oracleparams.n_agents;
                agent_pts_.reserve(n_agents);

                nx = xgoal.rows()/n_agents;

                // phi.resize(oracleparams.n_agents);
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


        void setCtrlPts(std::vector<Agent>& agent_pts) override{

            agent_pts_ = agent_pts;
        }

        void BezierCurveGoal(const std::vector<Agent>& agent_pts, const int k, const int H) override{
            
            const float t = (float)k / (float)H;
            switch (agent_pts[1].control_pts.size())
            {
            case 1:
                for (int i = 0; i < n_agents; i++)
                {
                    xgoal(i*nx) = agent_pts[i].control_pts[0].x;
                    xgoal(i*nx + 1) = agent_pts[i].control_pts[0].y;
                }
                break;
            
            case 2:
                for (int i = 0; i < n_agents; i++)
                {
                    const ImVec2 P0 = agent_pts[i].control_pts[0];
                    const ImVec2 P1 = agent_pts[i].control_pts[1];
                    xgoal(i*nx) = (1 - t)*P0.x + t*P1.x;
                    xgoal(i*nx + 1) = (1 - t)*P0.y + t*P1.y;
                }
                break;
            case 3:
                for (int i = 0; i < n_agents; i++)
                {
                    const ImVec2 P0 = agent_pts[i].control_pts[0];
                    const ImVec2 P1 = agent_pts[i].control_pts[1];
                    const ImVec2 P2 = agent_pts[i].control_pts[2];

                    xgoal(i*nx) = P1.x + (1 - t)*(1 - t)*(P0.x - P1.x) + t*t*(P2.x - P1.x); 
                    xgoal(i*nx + 1) = P1.y + (1 - t)*(1 - t)*(P0.y - P1.y) + t*t*(P2.y - P1.y);
                }
                break;
            case 4:
                for (int i = 0; i < n_agents; i++)
                {
                    const ImVec2 P0 = agent_pts[i].control_pts[0];
                    const ImVec2 P1 = agent_pts[i].control_pts[1];
                    const ImVec2 P2 = agent_pts[i].control_pts[2];
                    const ImVec2 P3 = agent_pts[i].control_pts[3];
                    const float t1 = (1 - t);
                    const float t2 = t1*t1;
                    const float t3 = t1*t2;

                    xgoal(i*nx) = t3*P0.x + 3*t2*t*P1.x + 3*t1*t*t*P2.x + t*t*t*P3.x;
                    xgoal(i*nx + 1) = t3*P0.y + 3*t2*t*P1.y + 3*t1*t*t*P2.y + t*t*t*P3.y;
                }
                break;
            }


            
        }

        void BezierCurveGoal(const int k, const int H) override{
            
            const float t = (float)k / (float)H;
            switch (agent_pts_[1].control_pts.size())
            {
            case 1:
                for (int i = 0; i < n_agents; i++)
                {
                    xgoal(i*nx) = agent_pts_[i].control_pts[0].x;
                    xgoal(i*nx + 1) = agent_pts_[i].control_pts[0].y;
                }
                break;
            
            case 2:
                for (int i = 0; i < n_agents; i++)
                {
                    const ImVec2 P0 = agent_pts_[i].control_pts[0];
                    const ImVec2 P1 = agent_pts_[i].control_pts[1];
                    xgoal(i*nx) = (1 - t)*P0.x + t*P1.x;
                    xgoal(i*nx + 1) = (1 - t)*P0.y + t*P1.y;
                }
                break;
            case 3:
                for (int i = 0; i < n_agents; i++)
                {
                    const ImVec2 P0 = agent_pts_[i].control_pts[0];
                    const ImVec2 P1 = agent_pts_[i].control_pts[1];
                    const ImVec2 P2 = agent_pts_[i].control_pts[2];

                    xgoal(i*nx) = (1 -t)*(1 -t)*P0.x + 2*(1 - t)*t*P1.x + t*t*P2.x;   
                    xgoal(i*nx + 1) = (1 -t)*(1 -t)*P0.y + 2*(1 - t)*t*P1.y + t*t*P2.y;
                }
                break;
            case 4:
                for (int i = 0; i < n_agents; i++)
                {
                    const ImVec2 P0 = agent_pts_[i].control_pts[0];
                    const ImVec2 P1 = agent_pts_[i].control_pts[1];
                    const ImVec2 P2 = agent_pts_[i].control_pts[2];
                    const ImVec2 P3 = agent_pts_[i].control_pts[3];
                    const float t1 = (1 - t);
                    const float t2 = t1*t1;
                    const float t3 = t1*t2;

                    xgoal(i*nx) = t3*P0.x + 3*t2*t*P1.x + 3*t1*t*t*P2.x + t*t*t*P3.x;
                    xgoal(i*nx + 1) = t3*P0.y + 3*t2*t*P1.y + 3*t1*t*t*P2.y + t*t*t*P3.y;
                }
                break;
            }
        }

        bool setGoal(const int k) override{
            if (!isGoalChanging)
            {
                return false;
            }
            else 
            {
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
        float alpha = 0.2;          // parameter for varying the quadratic costs over the horizon

        std::vector<Agent> agent_pts_;

        // Rotating Goal Parameters
        bool isGoalChanging;
        VectorXd xgoal;
        
        int Nx;
        int Nu;

        int nx, n_agents;

}; 