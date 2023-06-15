#pragma once

#include "FeedbackLinearization.h"


// Still need to implement it
// State x: [x, y, theta, v]
// Input u: [v\dot, omega]

class DiffDrive4dFeedbackLinearization : public FeedbackLinearization {

    public:

        DiffDrive4dFeedbackLinearization(double dtin)
        {
            nx = 4;
            nu = 2;
            dt = dtin;
            xdot = VectorXd::Zero(nx);

        }
    
        VectorXd dynamics(const VectorXd &x, const VectorXd &u) override {
            
            // VectorXd xdot(nx);
            xdot(0) = x(3)*std::cos(x(2));
            xdot(1) = x(3)*std::sin(x(2));
            xdot(2) = u(1);
            xdot(3) = u(0);
            return xdot;
        }

        void stateJacob(Eigen::Ref<MatrixXd> fx, const VectorXd& x, const VectorXd& u) override {
            assert(fx.rows() == nx);
            assert(fx.cols() == nx);

            fx <<  
                0.0, 0,  -x(3)*std::sin(x(2)), std::cos(x(2)), 
                0.0, 0,   x(3)*std::cos(x(2)), std::sin(x(2)),
                0.0, 0,                     0,              0,
                0.0, 0,                     0,              0;
            
            // Discretize Jacobian
            fx = fx*dt + MatrixXd::Identity(nx, nx);
        }

        void controlJacob(Eigen::Ref<MatrixXd> fu, const VectorXd& x, const VectorXd& u) override {
            assert(fu.rows() == nx);
            assert(fu.cols() == nu);

            fu <<  
                0.0,   0.0, 
                0.0,   0.0,
                0.0,   1.0,
                1.0,   0.0;
            
            // Discretize Jacobian
            fu = fu*dt;
        }
    private:
        VectorXd xdot;
};