#pragma once

#include "model.h"


// State x: [x, y, ẋ, ẏ]
// Input u: [FX, Fy]

class PointMass : public Model {

    public:

        // ~PointMass() {}

        PointMass(double dtin)
        {
            nx = 4;
            nu = 2;
            dt = dtin;
        }
    
        VectorXd dynamics(const VectorXd &x, const VectorXd &u) override {
            VectorXd xdot(nx);
            // ẋ₁ = x[3]
            // ẍ₁ = -(c/m)*ẋ₁ + u[1]/(m)   
            // ẏ₁ = x[4]
            // ÿ₁ = -(c/m)*ẏ₁ + u[2]/(m)  
            xdot(0) = x(2);
            xdot(1) = x(3);
            xdot(2) = -(damp/mass)*x(2) + u(0)/mass;
            xdot(3) = -(damp/mass)*x(3) + u(1)/mass;
            return xdot;
        }


        void stateJacob(MatrixXd &fx, const VectorXd& x, const VectorXd& u) override {
            assert(fx.rows() == nx);
            assert(fx.cols() == nx);

            fx <<  
                0.0, 0.0,          1.0,          0.0,
                0.0, 0.0,          0.0,          1.0,
                0.0, 0.0, (-damp/mass),          0.0,
                0.0, 0.0,          0.0, (-damp/mass);
            
            // Discretize Jacobian
            fx = fx*dt + MatrixXd::Identity(nx, nx);
        }

        void controlJacob(MatrixXd &fu, const VectorXd& x, const VectorXd& u) override {
            assert(fu.rows() == nx);
            assert(fu.cols() == nu);

            fu <<  
                    0.0,      0.0, 
                    0.0,      0.0,
               (1/mass),      0.0, 
                    0.0, (1/mass);
            
            // Discretize Jacobian
            fu = fu*dt;
        }
    
    private:
        float mass = 1.0;
        float damp = 0.1;

};