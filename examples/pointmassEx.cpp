#include "ALILQGames/ALILQGames.h"
#include "ALILQGames/pointmass.h"
#include "ALILQGames/NPlayerModel.h"
#include "ALILQGames/BoxConstraint.h"
#include "ALILQGames/costDiffDrive.h"
#include <iostream>
#include <matplotlibcpp17/pyplot.h>
#include <matplotlibcpp17/animation.h>
#include <vector>
#include <chrono>

using namespace std;
using namespace matplotlibcpp17;
using matplotlibcpp17::animation::ArtistAnimation;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

// void* operator new(size_t size)
// {
//     std::cout << "Allocating " << size << " bytes\n";
//     return malloc(size);
// }

int main(){

    // Player 1 costs

    MatrixXd Q1 = MatrixXd::Zero(8,8);
    Q1.block(0,0,4,4) = 0.1*MatrixXd::Identity(4,4);

    MatrixXd QN1 = MatrixXd::Zero(8,8);
    QN1.block(0,0,4,4) = 30.0*MatrixXd::Identity(4,4);
    
    MatrixXd R1 = MatrixXd::Zero(4,4);
    R1.block(0,0,2,2) = 2.0*MatrixXd::Identity(2,2);

    // Player 2 costs

    MatrixXd Q2 = MatrixXd::Zero(8,8);
    Q2.block(4,4,4,4) = 0.1*MatrixXd::Identity(4,4);

    MatrixXd QN2 = MatrixXd::Zero(8,8);
    QN2.block(4,4,4,4) = 30.0*MatrixXd::Identity(4,4);
    
    MatrixXd R2 = MatrixXd::Zero(4,4);
    R2.block(2,2,2,2) = 2.0*MatrixXd::Identity(2,2);

    // Initialize a 2 player point mass
    VectorXd x0(8);

    VectorXd xgoal(8);
    VectorXd u(4);

    x0 << 0.0, 1.5, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0;
    xgoal << 5.0, 1.5, 0.0, 0.0, 1.5, 5.0, 0.0, 0.0;

    u << 0.0, 0.0, 0.0, 0.0;

    double dt = 0.1;

    // VectorXd umin(4);
    // umin << -5.0, -5.0, -5.0, -5.0;
    // VectorXd umax(4);
    // umax << 5.0, 5.0, 5.0, 5.0;

    // VectorXd xmin(8);
    // xmin << -100.0, -100.0, -10.0, -10.0, -100.0, -100.0, -10.0, -10.0;
    // VectorXd xmax(8);
    // xmax << 100.0, 100.0, 10.0, 10.0, 100.0, 100.0, 10.0, 10.0;
    int nx = 4;
    int nu = 2;
    int n_agents = 3;
    VectorXd umin = VectorXd::Random(n_agents*nu);
    VectorXd umax = VectorXd::Random(n_agents*nu);
    VectorXd utest = VectorXd::Random(n_agents*nu);

    VectorXd xmin = VectorXd::Random(n_agents*nx);
    VectorXd xmax = VectorXd::Random(n_agents*nx);
    VectorXd xtest = VectorXd::Random(n_agents*nx);


    VectorXd cinq = VectorXd::Zero(n_agents*(nx + nu));

    Constraints* cg = new BoxConstraint(umin, umax, xmin, xmax);

    auto t0 = high_resolution_clock::now();
    cg -> StateAndInputConstraint(cinq ,xtest ,utest);
    auto t1 = high_resolution_clock::now();

    /* Getting number of milliseconds as a double. */
    duration<double, std::milli> ms_double = t1 - t0;
    std::cout << ms_double.count() << "ms\n";

    //std::cout << 
    // // Model& PointMass(dt);
    // Model* pm = new PointMass(dt);                    // heap allocation

    // NPlayerModel* Npm = new NPlayerModel(pm, 2);

    // std::vector<std::shared_ptr<Cost>> pc;  
    // pc.push_back( std::shared_ptr<Cost> (new DiffDriveCost(Q1, QN1, R1, xgoal)) );   
    // pc.push_back( std::shared_ptr<Cost> (new DiffDriveCost(Q2, QN2, R2, xgoal)) );


    // ALILQGames* alilqgame = new ALILQGames(Npm, pc);                 // Declare pointer to the ILQR class.

    // int H = 100;

    // alilqgame->init(H);

    // auto t0 = high_resolution_clock::now();
    // alilqgame -> solve(x0);
    // auto t1 = high_resolution_clock::now();

    // /* Getting number of milliseconds as a double. */
    // duration<double, std::milli> ms_double = t1 - t0;
    // std::cout << ms_double.count() << "ms\n";

    // pybind11::scoped_interpreter guard{};
    // auto plt = matplotlibcpp17::pyplot::import();

    // VectorXd X_t(8);
    // VectorXd U_t(4);

    
    // std::vector<double> x1, y1, x2, y2, u11, u12, u21, u22;
    // std::vector<double> ts;

    // for (int k = 0; k < H-1; k++) {
    //     X_t = alilqgame->getState(k);
    //     U_t = alilqgame->getControl(k);
    //     u11.push_back(U_t[0]);
    //     //Y.push_back(X_t[1]);
    //     u12.push_back(U_t[1]);
    // }

    // plt.plot(Args(u11),
    //        Kwargs("color"_a = "blue", "linewidth"_a = 1.0));
    
    // plt.plot(Args(u12),
    //        Kwargs("color"_a = "red", "linewidth"_a = 1.0));

    // plt.show();



    return 0;
}