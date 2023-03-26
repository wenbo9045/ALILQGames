#include "ALILQGames/NPlayerModel.h"
#include "ALILQGames/model.h"

        // ~PointMass() {}
        // For heterogenous agents, pass std::vector<Model>& playerModels
        VectorXd NPlayerModel::dynamics_concat(const VectorXd &x, const VectorXd &u)
        {
            VectorXd xdot(Nx);              // init this

            // Concatenated state
            for (int i=0; i<MPlayers; i++)
            {
                xdot.segment(i*nx, nx) = model->dynamics(x.segment(i*nx, nx), u.segment(i*nu, nu));      
            }
            return xdot;
        }

        // Dynamics Jacobian is a bit different from the documentation.
        // Here the state and the controls are concatenated 
        // x_{k+1} = [A1  0  0; x_{k} + [B1  0   0; [u_1; u_2; u_3]
        //            0  A2  0;          0  B2   0;
        //            0   0 A3]          0   0  B3]


        void NPlayerModel::dynamicsJacobConcat(MatrixXd &fx, MatrixXd &fu, const VectorXd& x, const VectorXd& u)
        {

            assert(fx.rows() == Nx);
            assert(fx.cols() == Nx);
            assert(fu.rows() == Nx);
            assert(fu.cols() == Nu);
            
            for (int i=0; i<MPlayers; i++)
            {
                model->stateJacob(fx.block(i*nx, i*nx, nx, nx), x.segment(i*nx, nx), u.segment(i*nu, nu));
                model->controlJacob(fu.block(i*nx, i*nu, nx, nu), x.segment(i*nx, nx), u.segment(i*nu, nu));      
            }
        }