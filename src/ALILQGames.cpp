#include "ALILQGames/ALILQGames.h"


void ALILQGames::init(int Horizon)
{
    H = Horizon;
    x_k.resize(H);
    u_k.resize(H);
    K_k.resize(H);
    d_k.resize(H);
    
    nx = Nmodel -> nx;                                      // Number of states for agent i
    nu = Nmodel -> nu;                                      // Number of controls for agent i
    Nx = Nmodel -> Nx;                                      // Total number of states for all agents
    Nu = Nmodel -> Nu;                                      // Total number of controls for all agents
    n_agents = Nmodel ->MPlayers;

    // Initialize Jacobians
    fx = MatrixXd::Zero(Nx, Nx);                            // Feedback gain is m by n
    fu = MatrixXd::Zero(Nx, Nu);                            // Feedback gain is m by n


    // Backward pass stuff 
    P = lxx;                                                // value matrix at final step is only cost/state hessian
    p = lx;                    
    S = MatrixXd::Zero(Nu, Nu);                             // Same as S in the document
    F_k = MatrixXd::Zero(Nx, Nx);                           // Placeholder for eqn readibility
    YK = MatrixXd::Zero(Nu, Nx);                            // Right hand side of system of linear equations for K                              
    Yd = VectorXd::Zero(Nu);                                // Right hand side of system of linear equations for d


    for(int i=0; i < n_agents; i++)
    {
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
        u_k[k] = 0.01*VectorXd::Random(Nu);                           // Input vector is m dimensional
        K_k[k] = 0.0*MatrixXd::Random(Nu, Nx);                        // Feedback gain is m by n
        d_k[k] = 0.0*VectorXd::Random(Nu);                            // Feedforward is m dimensional
    }
}

void ALILQGames::initial_rollout(const VectorXd& x0)
{
    x_k[0] = x0;                                // x0 
    
    for(int k=0; k < H; k++)                  
    {
        //u_t[k] = - K_t[k]*x_t[k] - k_t[k];                  // This is not neccasry (just an initial random rollout)
        x_k[k+1] = Nmodel->RK4(x_k[k], u_k[k] , Nmodel->dt);         // rollout nonlinear dynmaics with policy
    }

}


// pass by reference because might implement mpc
void ALILQGames::forward_rollout(const VectorXd& x0)
{
    x_hat = x_k;                                                 // previous states
    u_hat = u_k;                                                 // previous controls

    x_k[0] = x0;                                                 // Inital State (will change for MPC) 
    max_grad = d_k[0].norm();                                    // Want to store infinity norm of feedforward term (for convergence)


    double norm_grad;

    for(int k=0; k < H; k++)                  
    {
        u_k[k] = u_hat[k] - K_k[k]*(x_t[k] - x_hat[k]) - d_k[k];        // optimal affine policy
        // std::cout << "xhat " << x_hat[k] << "\n";
        // std::cout << "x_t " << x_t[k] << "\n";
        x_k[k+1] = Nmodel->RK4(x_k[k], u_k[k] , Nmodel->dt);                      // rollout nonlinear dynmaics with policy

        // Store Maximum gradient (to check for convergence)
        norm_grad = d_k[k].norm();
        if (norm_grad > max_grad)
        {
            max_grad = norm_grad;
        }
    }

}

void ALILQGames::backward_pass()
{
    // Terminal State cost to go
    //->TerminalCostGradient(lx, x_k[H-1]);
    for (int i=0; i < n_agents; i++)            // For each agent
    {
        Ncost[i]->TerminalCostGradient(lx[i], x_k[H-1]);
        Ncost[i]->TerminalCostHessian(lxx[i], x_k[H-1]);
    }
    cost->TerminalCostHessian(lxx, x_k[H-1]);

    // Check if I should do copy or not
    P = lxx;                                          // value matrix at final step is only cost/state hessian
    p = lx;                    

    // iter_cost += cost->TerminalCost(x_t[H-1]);

    for(int k=H-1; k>=0; k-- )     
    {
        // x_{k+1} = [A1  0  0; x_{k} + [B1  0   0; [u_1; u_2; u_3]
        //            0  A2  0;          0  B2   0;
        //            0   0 A3]          0   0  B3]

        Nmodel->dynamicsJacobConcat(fx, fu, x_k[k], u_k[k]);  // get the concatenated dynamics


        for (int i=0; i < n_agents; i++)            // For each agent
        {
            cost->StageCostGradient(lx[i], lu[i], x_k[k], u_k[k]);
            cost->StageCostHessian(lxx[i], luu[i], x_k[k], u_k[k]); 


            // S = [(R¹¹ + B¹ᵀ P¹ B¹)     (B¹ᵀ P¹ B²)     ⋅⋅⋅      (B¹ᵀ P¹ Bᴺ)   ;
            //         (B²ᵀ P² B¹)     (R²² + B²ᵀ P² B²)  ⋅⋅⋅      (B²ᵀ P² Bᴺ)   ;
            //              ⋅                  ⋅        ⋅                ⋅       ;
            //              ⋅                  ⋅           ⋅             ⋅       ;
            //              ⋅                  ⋅              ⋅          ⋅       ;
            //         (Bᴺᵀ Pᴺ B¹)        (Bᴺᵀ Pᴺ B²)     ⋅⋅⋅   (Rᴺᴺ + Bᴺᵀ Pᴺ Bᴺ)], Nu × Nu

            // This naively fills the entire row            #P.middleRows(i*nx, nx) change this
            S.block(i*nu, 0, nu, Nu) 
                = fu.middleCols(i*nu, nu).transpose() * P[i] * fu;
            
            // The diagonals are overritten here
            S.block(i*nu, i*nu, nu, nu) 
                = luu[i].block(i*nu, i*nu, nu, nu)
                + (fu.middleCols(i*nu, nu).transpose() * P[i] * fu.middleCols(i*nu, nu));
            
            YK.block(i*nu, i*nu, nu, nu) 
                = fu.middleCols(i*nu, nu).transpose() * P[i] * fx;
            
            Yd.segment(i*nu, nu) = (fu.middleCols(i*nu, nu).transpose() * p[i] + lu[i].segment(i*nu, nu);

        }
        // Do a least squares like \ in matlab??
        S = S.inverse();
        K_k[k] = S*YK; 
        d_k[k] = S*Yd;

        F_k = fx - fu * K_k[k];
        beta_k = - fu * d_k[k]

        for (int i=0; i < n_agents; i++)            // For each agent
        {

            // Update recursive variables P and p
            p[i] = lx[i] + (K_k[k].transpose() * (luu[i] * d_k[k] - lu[i]) + F_k.transpose() * (p[i] + P[i] * beta_k);

            P[i] = lxx[i] + (K_k[k].transpose() * luu[i] * K_k[k]) + F_k.transpose() * P[i] * F_k;

        }

        // iter_cost += cost->StageCost(x_t[k], u_t[k]);

    }  
}

void ALILQGames::solve(const VectorXd& x0)
{

    initial_rollout(x0);

    for(int iter=0; iter < 40; iter++)
    {

        iter_cost = 0.0;

        backward_pass();

        std::cout << iter_cost << "\n";

        forward_rollout(x0);


        if ( max_grad < 0.00001 ) // If the infinity norm of gradient term is less than some tolerance, converged
        {
            std::cout << "Converged!" << "\n"; 
            break;
        }
    }

    std::cout << "Solution x[end]: " << x_t[0] << "\n";
    std::cout << "Solution x[end]: " << x_t[99] << "\n";
    //std::cout << "Solution u[end]: " << u_t[98] << "\n";

}

// void recedingHorizon(const VectorXd& x0)
// {

// }


VectorXd ALILQGames::getState(int k)
{
    return x_t[k];
}