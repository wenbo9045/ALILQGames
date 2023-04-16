#pragma once
#include "GlobalConstraints.h"

class BoxConstraint : public GlobalConstraints {

    public:
        BoxConstraint(VectorXd& umin_in, VectorXd& umax_in, VectorXd& xmin_in, VectorXd& xmax_in)
        {
            umin = umin_in;
            umax = umax_in;
            xmin = xmin_in;
            xmax = xmax_in;
            nx = xmax_in.rows();
            nu = umax_in.rows();

            n_constr = 2*(nx + nu);

            Cx = MatrixXd::Zero(n_constr, nx);
            Cx.block(0,0,nx,nx) = MatrixXd::Identity(nx, nx);
            Cx.block(nx+nu,0,nx,nx) = -MatrixXd::Identity(nx, nx);

            Cu = MatrixXd::Zero(n_constr, nu);
            Cu.block(nx,0,nu,nu) = MatrixXd::Identity(nu, nu);
            Cu.block(2*nx + nu,0, nu , nu) = -MatrixXd::Identity(nu, nu);

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

        void StateAndInputConstraint(Eigen::Ref<VectorXd> c,const VectorXd& x, const VectorXd& u) override {
            assert(nx == x.rows());
            assert(nu == u.rows());
            // Z << x, 
            //      u;                                      // concatenate state and inputs

            // c = C*Z - d;

            // std::cout << "\nc1\n" << c << "\n";


            // TODO: Check which is faster
            // #pragma omp parallel for num_threads(2)
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
        
        void StateConstraintJacob(Eigen::Ref<MatrixXd> cx, const VectorXd& x) override {cx = Cx;}

        void ControlConstraintJacob(Eigen::Ref<MatrixXd> cu,const VectorXd& u) override {cu = Cu;} 

    private:
        VectorXd umin, umax, xmin, xmax;
        // VectorXd Z;
        // VectorXd d;
        MatrixXd Cx,Cu;
        // MatrixXd C;
        int nx, nu;

};
