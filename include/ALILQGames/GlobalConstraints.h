#pragma once
// #define EIGEN_DONT_PARALLELIZE

#include <Eigen/Dense>
#include <cassert>  


using Eigen::VectorXd;
using Eigen::MatrixXd;

struct GlobalConstraints
{
    int n_constr;                                   // number of constraints

    virtual ~GlobalConstraints() = default;

    virtual void StateConstraint(VectorXd& c, const VectorXd& x) {}

    virtual void InputConstraint(VectorXd& c, const VectorXd& u) {}

    virtual void StateAndInputConstraint(Eigen::Ref<VectorXd> c,const VectorXd& x, const VectorXd& u) {}


    virtual void StateConstraintJacob(Eigen::Ref<MatrixXd> cx, const VectorXd& x) {}

    virtual void ControlConstraintJacob(Eigen::Ref<MatrixXd> cu,const VectorXd& u) {}

};
