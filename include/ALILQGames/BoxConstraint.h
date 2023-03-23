#pragma once
#include "constraints.h"

class BoxConstraint : public Constraints {

    public:
        BoxConstraint(VectorXd& umin_in, VectorXd& umax_in, VectorXd& xmin_in, VectorXd& xmax_in)
        {
            umin = umin_in;
            umax = umax_in;
            xmin = xmin_in;
            xmax = xmax_in;
            nx = xmax_in.rows();
            nu = umax_in.rows();

            cx = MatrixXd::Zero(2*(nx + nu), nx);
            cx.block(0,0,nx,nx) = MatrixXd::Identity(nx, nx);
            cx.block(nx+nu,0,nx,nx) = -MatrixXd::Identity(nx, nx);

            cu = MatrixXd::Zero(2*(nx + nu), nu);
            cu.block(nx,0,nu,nu) = MatrixXd::Identity(nu, nu);
            cu.block(2*nx + nu,0, nu , nu) = -MatrixXd::Identity(nu, nu);

            // asserting that you need to provide constraints for all states and inputs 
            assert(xmin_in.rows() == nx);
            assert(umin_in.rows() == nu);

            // Z = VectorXd::Zero(nx + nu);
            // d = VectorXd::Zero(2*(nx + nu));
            // d << xmax, 
            //      umax, 
            //     -xmin, 
            //     -umin;

            // C = MatrixXd::Zero(2*(nx + nu), (nx + nu));
            // C << cx, cu;

        }

        void StateAndInputConstraint(VectorXd& c,const VectorXd& x, const VectorXd& u) override {
            assert(nx == x.rows());
            assert(nu == u.rows());
            // Z << x, 
            //      u;                                      // concatenate state and inputs

            // c = C*Z - d;

            // std::cout << "\nc1\n" << c << "\n";


            // TODO: Check which is faster
            for(int i=0; i < c.rows(); i++)
            {
                if (i < nx)
                {
                    c(i) = x(i) - xmax(i);
                }
                if (i >= nx && i < (nx + nu))
                {
                    c(i) = u(i - nx) - umax(i - nx);
                } 
                if (i >= (nx + nu) && i < (2*nx + nu))
                {
                    c(i) = -x(i - nx - nu) + xmin(i - nx - nu);
                }
                if (i >= (2*nx + nu) && i < (2*nx + 2*nu))
                {
                    c(i) = -u(i- 2*nx - nu) + umin(i - 2*nx - nu);
                }      
            }

            // std::cout << "\nc2\n" << c << "\n";

        }

    private:
        VectorXd umin, umax, xmin, xmax;
        // VectorXd Z;
        // VectorXd d;
        MatrixXd cx,cu;
        // MatrixXd C;
        int nx, nu;

};
