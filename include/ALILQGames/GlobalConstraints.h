#pragma once
#include <Eigen/Dense>
#include <cassert>  


using Eigen::VectorXd;
using Eigen::MatrixXd;

struct GlobalConstraints
{
    virtual ~GlobalConstraints() = default;

    virtual void StateConstraint(VectorXd& c, const VectorXd& x) {}

    virtual void InputConstraint(VectorXd& c, const VectorXd& u) {}

    virtual void StateAndInputConstraint(VectorXd& c,const VectorXd& x, const VectorXd& u) {}


    virtual void StateConstraintJacob(MatrixXd& cx, const VectorXd& x) {}

    virtual void ControlConstraintJacob(MatrixXd& cu,const VectorXd& u) {}

};
