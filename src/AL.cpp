#include "ALILQGames/AL.h"


double AL::Merit(const int k, const int i, const double l, 
                    const VectorXd& x, const VectorXd& u)
{
    ConcatConstraint(x, u);

    ActiveConstraint(k);

    return l + (lambda[k] + 0.5 * I_mu * c).transpose() * c;
}

void AL::ALGradHess(const int k, MatrixXd& Lxx, MatrixXd& Luu, MatrixXd& Lux,
                VectorXd& Lx, VectorXd& Lu, const MatrixXd& lxx, const MatrixXd& luu,
                const MatrixXd& lux, const VectorXd& lx, 
                const VectorXd& lu, const VectorXd& x, const VectorXd& u)
{
    ConcatConstraint(x, u);                             // call the constraint concatenation function

    // cout << "c \n" << k << " " << c << "\n";
    
    ActiveConstraint(k);                                // check the active constraints

    Lx = lx + cx.transpose() * (lambda[k] + I_mu*c);    // Derivative of AL wrt x
    Lu = lu + cu.transpose() * (lambda[k] + I_mu*c);    // Derivative of AL wrt u
    Lxx = lxx + (I_mu * cx).transpose() * cx;
    Luu = luu + (I_mu * cu).transpose() * cu;
    Lux = lux + (I_mu * cu).transpose() * cx;
}

// void AL::ALHess(MatrixXd& Lxx, MatrixXd& Luu, MatrixXd& Lux, 
//                 const VectorXd& lxx, const VectorXd& luu,
//                 const VectorXd& lux)
// {
//     Lxx = lxx + (I_mu * cx).transpose() * cx;
//     Luu = luu + (I_mu * cu).transpose() * cu;
//     Lux = lux + (I_mu * cu).transpose() * cx;
// } 

void AL::ActiveConstraint(const int k)
{
    I_mu.setZero();
    // Check for the active contraints
    // #pragma omp parallel for num_threads(4)
    for(int ii=0; ii< I_mu.rows(); ii++)
    {
        // if constraint is active or the dual variable associated with it is not 0
        if (c(ii) >= 0 || (abs(lambda[k](ii))) > 0.000001)     // if the inequality constraint is violated and the dual variable associated with it is not 0
        {
            I_mu(ii, ii) = mu;
        }
    }

}

// Todo: Find a better way to combine them
void AL::ConcatConstraint(const VectorXd& x, const VectorXd& u)
{
    c.setZero();
    cx.setZero();
    cu.setZero();

    int p = 0;                                              // number of constraints for each constraint class
    int p_prev = 0;
    for (int i=0; i < ptr_cons.size(); i++)
    {
        p = ptr_cons[i]->n_constr;
        ptr_cons[i]->StateAndInputConstraint(c.segment(p_prev, p), x, u);
        ptr_cons[i]->StateConstraintJacob(cx.middleRows(p_prev, p), x);
        ptr_cons[i]->ControlConstraintJacob(cu.middleRows(p_prev, p), u);

        p_prev += p;
    }
    // cout << "c(0) " << c(1) << "\n";

}

double AL::MaxConstraintViolation(const VectorXd& x, const VectorXd& u)
{
    c.setZero();
    int p = 0;                                              // number of constraints for each constraint class
    int p_prev = 0;
    // double max_c = 0.0;
    for (int i=0; i < ptr_cons.size(); i++)
    {
        p = ptr_cons[i]->n_constr;
        ptr_cons[i]->StateAndInputConstraint(c.segment(p_prev, p), x, u);
        p_prev += p;
    }
    return c.maxCoeff();
}

void AL::DualUpdate(const vector<VectorXd>& x_k, const vector<VectorXd>& u_k)
{

    for(int k=0; k < x_k.size() - 1; k++)
    {
        ConcatConstraint(x_k[k], u_k[k]);
        // cout << "c(0) " << c(1) << "\n";
        // cout << "C \n" << k << "\n";

        lambda[k] = (lambda[k] + mu*c).cwiseMax(0);
        // lambda[k] += (mu*c).cwiseMax(0);
    }
}

void AL::PenaltySchedule()
{
    mu *= penalty_scale;
}

void AL::ResetDual()
{
    // #pragma omp parallel for num_threads(4)
    for(int k=0; k < lambda.size() - 1; k++)
    {
        lambda[k].setZero();
    }
}

void AL::ResetPenalty()
{
    mu = mu_original;
}

VectorXd AL::GetDual(const int k)
{
    return lambda[k];
}