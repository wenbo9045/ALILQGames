#include "ALILQGames/ILQGames.h"
#include "ALILQGames/Timer.h"


double ILQGames::initial_rollout(const VectorXd& x0)
{
    x_k[0] = x0;                                // x0 
    
    for(int k=0; k < H-1; k++)                  
    {
        //u_t[k] = - K_t[k]*x_t[k] - k_t[k];                  // This is not neccasry (just an initial random rollout)
        x_k[k+1] = Nmodel->RK4(x_k[k], u_k[k] , Nmodel->dt);         // rollout nonlinear dynmaics with policy
    }

    total_cost = 0.0;
    
    for (int i=0; i < n_agents; i++)            // For each agent
    {
        // cost[i] = pc[i]->TotalCost(i, H, x_k, u_k);
        total_cost += pc[i]->TotalCost(i, H, x_k, u_k);
    }

    return 0.0;
}


// TODO: implement mpc
double ILQGames::forward_rollout(const VectorXd& x0)
{
    x_hat = x_k;                                                 // previous states
    u_hat = u_k;                                                 // previous controls

    x_k[0] = x0;                                                 // Inital State (will change for MPC) 
    max_grad = d_k[0].lpNorm<Eigen::Infinity>();                 // Want to store infinity norm of feedforward term (for convergence)


    double norm_grad;

    for(int k=0; k < H-1; k++)                  
    {
        u_k[k] = u_hat[k] - K_k[k]*(x_k[k] - x_hat[k]) - alpha*d_k[k];        // optimal affine policy
        // std::cout << "xhat " << x_hat[k] << "\n";
        // std::cout << "x_t " << x_t[k] << "\n";
        x_k[k+1] = Nmodel->RK4(x_k[k], u_k[k] , Nmodel->dt);                      // rollout nonlinear dynmaics with policy

        // Store Maximum gradient (to check for convergence)
        norm_grad = d_k[k].lpNorm<Eigen::Infinity>();
        if (norm_grad > max_grad)
        {
            max_grad = norm_grad;
        }
    }
    return 0.0;
}

double ILQGames::backward_pass(const int k_now)
{
    // Terminal State cost to go
    for (int i=0; i < n_agents; i++)            // For each agent
    {
        pc[i]->TerminalCostGradient(i, lx[i], x_k[H-1]);
        pc[i]->TerminalCostHessian(i, lxx[i], x_k[H-1]);

        P[i] = lxx[i];
        p[i] = lx[i];
        DeltaV[i] = 0.0;

    }                 

    // iter_cost += cost->TerminalCost(x_t[H-1]);

    for(int k=H-2; k >= 0; k-- )     
    {
        // x_{k+1} = [A1  0  0; x_{k} + [B1  0   0; [u_1; u_2; u_3]
        //            0  A2  0;          0  B2   0;
        //            0   0 A3]          0   0  B3]

        Nmodel->dynamicsJacobConcat(fx, fu, x_k[k], u_k[k]);  // get the concatenated dynamics


        for (int i=0; i < n_agents; i++)            // For each agent
        {

            pc[i]->StageCostGradient(i, lx[i], lu[i], x_k[k], u_k[k]);
            pc[i]->StageCostHessian(i, lxx[i], luu[i], x_k[k], u_k[k]); 


            // std::cout << "Pnow " << k << " " << P[i] <<"\n";


            // Cheap hack to regularize the lxx matrix (this happens because of the collision constraint, which is non-convex)
            // Therefore, the hessian has negative eigen-values
            // Without regularization, the agents want to "collide"
            bool NotPD = false;
            lxx[i] += 0.000001*MatrixXd::Identity(Nx, Nx);

            Eigen::LLT<MatrixXd> lltOflxx(lxx[i].block(i*nx, i*nx, nx, nx)); // compute the Cholesky decomposition of lxx

            if (lltOflxx.info() == Eigen::NumericalIssue)
            {
                NotPD = true;
                // std::cout << "lxx is not PD\n " << lxx[i] << "\n";
            }

            // cout << !lxx[i].isZero(0) << "\n";

            while(NotPD && !lxx[i].isZero(0))
            {
                // std::cout << "lxx is not PD\n " << lxx[i] << "\n";
                NotPD = false;
                double max_lxx = lxx[i].block(i*nx, i*nx, nx, nx).maxCoeff();
                // double max_lxx = lxx[i].block(i*nx, i*nx, nx, nx).lpNorm<Eigen::Infinity>();
                lxx[i].block(i*nx, i*nx, nx, nx) += max_lxx*MatrixXd::Identity(nx, nx);

                Eigen::LLT<MatrixXd> lltOflxx(lxx[i].block(i*nx, i*nx, nx, nx));

                if (lltOflxx.info() == Eigen::NumericalIssue)
                {
                    NotPD = true;
                }
            }

            // cout << "working " << "\n";


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
                

            YK.middleRows(i*nu, nu) 
                = fu.middleCols(i*nu, nu).transpose() * P[i] * fx;
            
            Yd.segment(i*nu, nu) = (fu.middleCols(i*nu, nu).transpose() * p[i]) + lu[i].segment(i*nu, nu);

        }

        // for (size_t ii = 0; ii < S.cols(); ii++) {
        //     const float radius = S.col(ii).lpNorm<1>() - std::abs(S(ii, ii));
        //     const float eval_lo = S(ii, ii) - radius;

        //     constexpr float min_eval = 1e-3;
        //     if (eval_lo < min_eval){ 
        //         S(ii, ii) += radius + min_eval;
        //     }
        // }

        // Do a least squares like \ in matlab??
        S = S.inverse();
        K_k[k] = S*YK; 
        d_k[k] = S*Yd;

        F_k = fx - fu * K_k[k];
        beta_k = - fu * d_k[k];                     // n x m x m x 1 = n x 1

        for (int i=0; i < n_agents; i++)            // For each agent
        {

            // Update recursive variables P, p, and delta v

            // DeltaV seems to blow up, check for this
            DeltaV[i] += 0.5 * ((d_k[k].transpose() * luu[i] - 2 * lu[i].transpose()) * d_k[k]  + DeltaV[i]
            - ((P[i] * beta_k + 2 * p[i]).transpose() * beta_k).sum()) + DeltaV[i];

            // std::cout << "V[i] " << DeltaV[i] << "\n";

            p[i] = lx[i] + (K_k[k].transpose() * (luu[i] * d_k[k] - lu[i])) + F_k.transpose() * (p[i] + P[i] * beta_k);

            P[i] = lxx[i] + (K_k[k].transpose() * luu[i] * K_k[k]) + F_k.transpose() * P[i] * F_k;

        }
    }  
    return 0.0;
}

void ILQGames::BackTrackingLineSearch(const VectorXd& x0)
{
    x_hat = x_k;                                                 // previous states
    u_hat = u_k;  
    double alphas[6] = {1.0, 0.5, 0.25, 0.125, 0.0625, 0.03125};
    double total_cost_now = 0.0;

    for (int i=0; i<6; i++)
    {
        total_cost_now = 0.0;
        alpha = alphas[i];
        forward_rollout(x0);
        for (int i=0; i < n_agents; i++)            // For each agent
        {
            total_cost_now += pc[i]->TotalCost(i, H, x_k, u_k);
        }

        if (iter_ == 0)          // first iteration is random so we dont care about it (cost should increase)
        {
            break;
        }

        // if (abs(total_cost_now - total_cost) < 10000.0)
        if (total_cost_now < total_cost)
        {
            break;
        }
    }

    total_cost = total_cost_now;
}

void ILQGames::ArmuijoLineSearch(const VectorXd& x0)
{
    x_hat = x_k;                                                 // previous states
    u_hat = u_k;                                                 // previous controls
    double total_cost_now = 0.0;
    double DeltaV_total = 0.0;

    forward_rollout(x0);                                       // rollout with alpha = 1
    for (int i=0; i < n_agents; i++)            // For each agent
    {
        // Store entire previous trajectory
        total_cost_now += pc[i]->TotalCost(i, H, x_k, u_k);
        DeltaV_total += DeltaV[i];
        // cost_now[i] = pc[i]->TotalCost(i, H, x_k, u_k);
    }
    // while (cost_now[i] > cost[i] - 1e-2*alpha*DeltaV[i])
    while (total_cost_now > total_cost - (1e-3*alpha * DeltaV_total))
    {
        // std::cout << "Hello !" << "\n";
        alpha *= 0.5;

        forward_rollout(x0);            // rollout with reduced alpha

        total_cost_now = 0.0;

        // cost_now[i] = pc[i]->TotalCost(i, H, x_k, u_k);
        for (int i=0; i < n_agents; i++)            // For each agent
        {
            total_cost_now += pc[i]->TotalCost(i, H, x_k, u_k);
        }
    }
    alpha = 1.0;                            // reset alpha
    total_cost = total_cost_now;                  // previous cost is now current cost
    // cost[i] = cost_now[i];                  // previous cost is now current cost
}

void ILQGames::solve(SolverParams& params, const VectorXd& x0) 
{ 
    {
    Timer timer;
    initial_rollout(x0);

    iter_ = 0;

    double total_cost_prev = 0.0;

    for(int iter=0; iter < params.max_iter_ilq; iter++)
    {

        iter_cost = 0.0;

        backward_pass(iter_);

        double total_cost_now = 0.0;

        for (int i=0; i < n_agents; i++)            // For each agent
        {
            total_cost_now += pc[i]->TotalCost(i, H, x_k, u_k);
        }

        std::cout << "Total Cost Now: " << total_cost_now << "\n";

        // BackTrackingLineSearch(x0);
        // lineSearch(x0);
        forward_rollout(x0);

        if ( max_grad < params.grad_tol || abs(total_cost_now -  total_cost_prev) < params.cost_tol) // If the infinity norm of gradient term is less than some tolerance, converged
        {
            std::cout << "Converged!" << "\n"; 
            break;
        }

        total_cost_prev = total_cost_now; 
        iter_ += 1;
    }
    }
    std::cout << "Solution x[0]: " << x_k[0] << "\n";
    std::cout << "Solution x[end]: " << x_k[H-1] << "\n";
    //std::cout << "Solution u[end]: " << u_t[98] << "\n";

}

// // void recedingHorizon(const VectorXd& x0)
// // {

// // }


VectorXd ILQGames::getState(const int k)
{
    return x_k[k];
}

VectorXd ILQGames::getControl(const int k)
{
    return u_k[k];
}

VectorXd ILQGames::getMPCState(const int k) 
{

    cout << "Your'e not implementing MPC! Error\n";
    return x_k[k];
}

VectorXd ILQGames::getMPCControl(const int k) 
{
    cout << "Your'e not implementing MPC! Error\n";
    return u_k[k];
}

double ILQGames::getStageCost(const int i, const int k)
{

    return pc[i]->StageCost(i, x_k[k], u_k[k]);
}

double ILQGames::getTerminalCost(const int i)
{
    return pc[i]->TerminalCost(i, x_k[H-1]);
}