#pragma once

#include "model.h"


// State x: [x, y, theta]
// Input u: [v, omega]

class DiffDriveModel3D : public Model {

    public:

        DiffDriveModel3D(double dtin)
        {
            nx = 3;
            nu = 2;
            dt = dtin;
            xdot = VectorXd::Zero(nx);
        }
    
        VectorXd dynamics(const VectorXd &x, const VectorXd &u) override {

            xdot(0) = u(0)*std::cos(x[2]);
            xdot(1) = u(0)*std::sin(x[2]);
            xdot(2) = u(1);
            return xdot;
        }

        void stateJacob(Eigen::Ref<MatrixXd> fx, const VectorXd& x, const VectorXd& u) override {            
            assert(fx.rows() == nx);
            assert(fx.cols() == nx);

            fx <<  
                0, 0,  -u(0)*std::sin(x[2]),
                0, 0,   u(0)*std::cos(x[2]),
                0, 0,                      0;
            
            // Discretize Jacobian
            fx = fx*dt + MatrixXd::Identity(nx, nx);
        }

        void controlJacob(Eigen::Ref<MatrixXd> fu, const VectorXd& x, const VectorXd& u) override {
            assert(fu.rows() == nx);
            assert(fu.cols() == nu);

            fu <<  
                std::cos(x[2]),     0, 
                std::sin(x[2]),     0,
                             0,   1.0;
            
            // Discretize Jacobian
            fu = fu*dt;
        }

    private:
        VectorXd xdot;
};