#pragma once
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class GlobalIneqConstraints
{
    public:
        GlobalIneqConstraints(int p_constraints)
        {
            ineq = p_constraints;
        }

        int ineq;



};