#include "ALILQGames/ALILQGames.h"
#include "ALILQGames/pointmass.h"
#include "ALILQGames/costDiffDrive.h"
#include <iostream>
// #include "matplotlibcpp.h"



// namespace plt = matplotlibcpp;

int main(){


    Eigen::MatrixXd Q(4,4);
    Eigen::MatrixXd QN(4,4);

    Eigen::MatrixXd R(2,2);
    Q << 
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;

    R << 
        1.0, 0.0,
        0.0, 1.0;

    QN = 10.0*Q;

    Eigen::VectorXd x = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd xgoal(4);
    Eigen::VectorXd u(2);

    xgoal << 1.5, 1.5, 0.0, 0.0;

    u << 0.0, 0.0;

    double dt = 0.1;

    // ILQR* ilqr;                 // Declare pointer to the ILQR class.
    // // ilqr->forward_rollout(x, u);        // access a member using member access operator

    // // It creates a new variable pm, with the type Model*, that is, a pointer to an object of type Model, and then
    // // It allocates a new Model object on the heap, and
    // // It sets the pm variable to point to the new Model object.
    // Model* pm = new PointMass(dt);                    // heap allocation 
    // //std::shared_ptr<Model> model;
    // //model.reset(pm);

    // Cost* pc = new DiffDriveCost(Q, QN, R, xgoal);
    
    // ilqr = new ILQR(pm, pc);                                    // heap allocation 

    // // // pm->costGoal(x,u);

    // int H = 100;

    // // int N = 1000;

    // ilqr->init(H);

    // ilqr->solve(x);

    // Eigen::VectorXd X_t(4);

    // std::vector<double> X, Y;




    // for (int k = 0; k < 100; k++) {
    //     X_t = ilqr->getState(k);
    //     X.push_back(X_t[0]);
    //     Y.push_back(X_t[1]);

    //     // plt::xlim(0, 5.0);



    //     if (k % 10 == 0) 
    //     {
    //         // Clear previous plot
    //         plt::clf();
    //         // Plot line from given x and y data. Color is selected automatically.
    //         // plt::scatter(X, Y, 3, {{"s", "1"}{"c", "red"}, {"label", "Bot"}});

    //         plt::scatter(X, Y, {30});

    //         plt::xlim(-2.5, 2.5);

    //         plt::ylim(-2.5, 2.5);

    //         // // Plot a line whose name will show up as "log(x)" in the legend.
    //         // plt::plot(x, z, {{"label", "log(x)"}});

    //         // Set x-axis to interval [0,1000000]
    //         //plt::xlim(0, 5.0);

    //         // Add graph title
    //         plt::title("Sample figure");
    //         // Enable legend.
    //         plt::legend();
    //         // Display plot continuously
    //         plt::pause(0.1);
    //     }

    //     X.pop_back();
    //     Y.pop_back();

    // }

    // ilqr->solve(x);
    //model->dynamics(x,u);
    // pm->RK4(x,u, dt);

    return 0;
}