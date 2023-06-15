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
            nx = params.nx;
            nu = params.nu;
        }
        
        // from angular velocity and acceleration to acceleration along x and y
        MatrixXd decoupling_matrix(const VectorXd& x) override {
            MatrixXd M(nu, nu);
            M << -x(3)*std::sin(x(2)), std::cos(x(2)),
                  x(3)*std::cos(x(2)), std::sin(x(2));
            
            return M;
        }

        // from acceleration along x and y to angular velocity and acceleration
        MatrixXd inverse_decoupling_matrix(const VectorXd& x) override {
            MatrixXd Minv(nu, nu);

            Minv << -std::sin(x(2))/x(3), std::cos(x(2))/x(3),
                    std::cos(x(2)), std::sin(x(2));
            return Minv;

        }

        // State conversion map x = lambda(zeta); zeta to x
        VectorXd conversion_map(const VectorXd& zeta) override {
            VectorXd x(nx);
            const float v_x = zeta(2);
            const float v_y = zeta(3);
            // lambda(zeta) = x, y, theta, v

            x << zeta(0), zeta(1), atan2(v_y, v_x), std::sqrt(v_x*v_x + v_y*v_y); 
            return x;
        }

        // x to zeta
        VectorXd inv_conversion_map(const VectorXd& x) override {
            VectorXd zeta(nx);
            // x, y, x_dot, y_dot
            zeta << x(0), x(1), x(3)*std::cos(x(2)), x(3)*std::sin(x(2));
            return zeta;
        }
    
    private:
        int nx;
        int nu;
    
};