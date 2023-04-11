#include "ALILQGames/NPlayerModel.h"
#include "ALILQGames/model.h"

    /*
    Base Class for a N agent dynamics model (Now Homogenous)

    dynamics_concat(): base function for concatenating all agents' states
                into one big vector ẋ. 
                It calls model->dynamics function (check model.h)
    
    dynamicsJacobConcat(): base function for concatenating all agents' state and
                and control Jacobian into the following:
                It is a bit different from the documentation.
    
                xₖ₊₁ = [A1  0  0; xₖ + [B1  0   0; [u_1; u_2; u_3]
                       0  A2  0;       0  B2   0;
                       0   0 A3]       0   0  B3]

                It calls model->stateJacob and model->controlJacob function (check model.h)

    RK4(): Integrates dynamics with explicit RK4
            Returns vector of state
    */


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

    void NPlayerModel::dynamicsJacobConcat(MatrixXd &fx, MatrixXd &fu, const VectorXd& x, const VectorXd& u)
    {

        assert(fx.rows() == Nx);
        assert(fx.cols() == Nx);
        assert(fu.rows() == Nx);
        assert(fu.cols() == Nu);
        
        for (int i=0; i<MPlayers; i++)
        {
            const int inx = i*nx;
            const int inu = i*nu;
            model->stateJacob(fx.block(inx, inx, nx, nx), x.segment(inx, nx), u.segment(inu, nu));
            model->controlJacob(fu.block(inx, inu, nx, nu), x.segment(inx, nx), u.segment(inu, nu));      
        }
    }