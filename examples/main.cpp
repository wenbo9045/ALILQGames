#include "ALILQGames/ALILQGames.h"
#include "ALILQGames/pointmass.h"
#include "ALILQGames/NPlayerModel.h"
#include "ALILQGames/costDiffDrive.h"
#include <iostream>
// #include "matplotlibcpp.h"


void* operator new(size_t size)
{
    std::cout << "Allocating " << size << " bytes\n";
    return malloc(size);
}
// namespace plt = matplotlibcpp;

int main(){

    // Player 1 costs

    MatrixXd Q1 = MatrixXd::Zero(8,8);
    Q1(0,0) = 0.0;
    Q1(1,1) = 0.0;
    Q1(2,2) = 0.0;
    Q1(3,3) = 0.0;

    std::cout << "\nQ1 "<< Q1 << "\n";

    MatrixXd QN1 = MatrixXd::Zero(8,8);


    QN1(0,0) = 10.0;
    QN1(1,1) = 10.0;
    QN1(2,2) = 10.0;
    QN1(3,3) = 10.0;

    std::cout << "\nQN1 "<< QN1 << "\n";
    
    MatrixXd R1 = MatrixXd::Zero(4,4);

    R1(0,0) = 1.0;
    R1(1,1) = 1.0;

    std::cout << "\nR1 "<< R1 << "\n";
    // Player 2 costs

    MatrixXd Q2 = MatrixXd::Zero(8,8);
    Q2(4,4) = 0.0;
    Q2(5,5) = 0.0;
    Q2(6,6) = 0.0;
    Q2(7,7) = 0.0;

    std::cout << "\nQ1 "<< Q1 << "\n";

    MatrixXd QN2 = MatrixXd::Zero(8,8);

    QN2(4,4) = 10.0;
    QN2(5,5) = 10.0;
    QN2(6,6) = 10.0;
    QN2(7,7) = 10.0;
    
    MatrixXd R2 = MatrixXd::Zero(4,4);

    R2(2,2) = 1.0;
    R2(3,3) = 1.0;

    // Initialize a 2 player point mass
    VectorXd x0(8);

    VectorXd xgoal(8);
    VectorXd u(4);

    x0 << 0.0, 1.5, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0;
    xgoal << 5.0, 1.5, 0.0, 0.0, 1.5, 5.0, 0.0, 0.0;

    u << 0.0, 0.0, 0.0, 0.0;

    double dt = 0.1;

    // Model& PointMass(dt);
    Model* pm = new PointMass(dt);                    // heap allocation

    NPlayerModel* Npm = new NPlayerModel(pm, 2);
    // Npm->dynamicsJacobConcat(fx, fu, x, u);
    // Npm->dynamics_concat(x, u);

    // ###################################################################################################################################
    // VIRTUAL FUNCTIONS USE ONLY POINTERS OR REFERENCE
    // DiffDriveCost p1(Q1, QN1, R1, xgoal);            // Create object directly or pointer to it?
    // DiffDriveCost p2(Q2, QN2, R2, xgoal);            // Create object directly or pointer to it?

    // std::vector<Cost> pcosts;

    // pcosts.emplace_back(p1);
    // pcosts.emplace_back(p2);
    // std::cout << pcosts[0].TerminalCost(xgoal) << "\n";
    // ###################################################################################################################################

    // Cost* pc1 = new DiffDriveCost(Q1, QN1, R1, xgoal);
    // Cost* pc2 = new DiffDriveCost(Q2, QN2, R2, xgoal);

    // std::vector<std::shared_ptr<Cost>> pc;  
    // pc.push_back( std::shared_ptr<Cost> (new DiffDriveCost(Q1, QN1, R1, xgoal)) );   
    // pc.push_back( std::shared_ptr<Cost> (new DiffDriveCost(Q2, QN2, R2, xgoal)) );


    // ALILQGames* alilqgame = new ALILQGames(Npm, pc);                 // Declare pointer to the ILQR class.

    // int H = 100;

    // alilqgame->init(H);

    // // alilqgame->initial_rollout(x);

    // // alilqgame->forward_rollout(x);

    // // alilqgame->backward_pass();
    // // std::cout << alilqgame->getState(88) << "\n";
    // alilqgame -> solve(x0);

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