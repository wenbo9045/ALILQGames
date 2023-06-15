#pragma once

#include "FeedbackLinearization.h"
#include "SolverParams.h"


// Still need to implement it
// State x: [x, y, theta, v]
// Input u: [v\dot, omega]

class DiffDrive4dFeedbackLinearization : public FeedbackLinearization {

    public:

        DiffDrive4dFeedbackLinearization(SolverParams& params)
        {

        }
    
        void decoupling_matrix(MatrixXd& M, const VectorXd& x) override {
            M << -x(3)*std::sin(x(2)), std::cos(x(2)),
                  x(3)*std::cos(x(2)), std::sin(x(2));
        }

        void inverse_decoupling_matrix(MatrixXd& Minv, const VectorXd& x) override {
            Minv << -std::sin(x(2))/x(3), std::cos(x(2))/x(3),
                    std::cos(x(2)), std::sin(x(2));

        }

        // State conversion map x = lambda(zeta); zeta to x
        void conversion_map(VectorXd& x, const VectorXd& zeta) override {
            const float v_x = zeta(1);
            const float v_y = zeta(3);
            // lambda(zeta) = p_x, p_y, /sqrt(v_x^2 + v_y^2), atan2(v_y,v_x)

            x << zeta(0), zeta(2), std::sqrt(v_x*v_x + v_y*v_y), atan2(v_y, v_x); 
        }

        // x to zeta
        void inv_conversion_map(VectorXd& zeta, const VectorXd& x) override {
            zeta << x(0), x(3)*std::cos(x(2)), x(1), x(3)*std::sin(x(2));
        }
    
};