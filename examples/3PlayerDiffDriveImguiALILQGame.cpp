#include "ALILQGames/ALILQGames.h"
#include "ALILQGames/diffdrive4d.h"
#include "ALILQGames/NPlayerModel.h"
#include "ALILQGames/costDiffDrive.h"
#include "ALILQGames/CollisionCost2D.h"
#include "ALILQGames/BoxConstraint.h"
#include "ALILQGames/CollisionConstraint2D.h"
#include <iostream>
#include <vector>
#include <chrono>
#include "ALILQGames/TrajImGui.h"
#include "GLFW/glfw3.h"


using namespace std;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;

// void* operator new(size_t size)
// {
//     std::cout << "Allocating " << size << " bytes\n";
//     return malloc(size);
// }
void change_clear_color(float r, float g, float b) {
    glClearColor(r, g, b, 1.00f);
}

int main(){

    // ################################# Declaring paramters for solver ######################################
    int nx = 4;                     
    int nu = 2;                     
    int n_ag = 3;                   
    int Nx = n_ag*nx;              
    int Nu = n_ag*nu;               

    double dt = 0.1;               

    SolverParams params;                                // load param stuct (holds "most" solver paramters)
    params.H = 250;                                     // horizon length
    params.dt = 0.1;                                    // discretization time
    params.nx = nx;                                     // single agent number of states
    params.nu = nu;                                     // single agent number of controls
    params.n_agents = n_ag;                             // total number of homogenous agents
    params.Nx = Nx;                                     // total number of concatenated states
    params.Nu = Nu;                                     // total number of concatenated controls
    params.rho_obs = 500.0;                             // penalty parameter for collision cost 
    
    // AL Params
    params.penalty_scale = 1.2;

    // number of inequality constraints = state and input constraints + number of collisions constraints btn agents
    params.p_inq = 2*(Nx + Nu) + n_ag*(n_ag - 1);   

    // lower & upper saturation limits
    VectorXd umin = -2.0*VectorXd::Ones(nu*n_ag);
    VectorXd umax =  2.0*VectorXd::Ones(nu*n_ag);

    // lower & upper state limits
    VectorXd xmin = -100.0*VectorXd::Ones(nx*n_ag);
    VectorXd xmax =  100.0*VectorXd::Ones(nx*n_ag);
    xmin(3) = -0.3;                                     // min Velocity constraint for agent 1
    xmin(7) = -0.3;                                     // min Velocity constraint for agent 2
    xmin(11) = -0.3;                                    // min Velocity constraint for agent 3

    xmax(3) =  0.3;                                     // max Velocity constraint for agent 1
    xmax(7) =  0.3;                                     // max Velocity constraint for agent 2
    xmax(11) = 0.3;                                     // max Velocity constraint for agent 3


    // collision avoidance radius for each agent: [r1: 2.0, r2: 2.0], where 1 and 2 are the agents
    VectorXd r_avoid(n_ag);

    r_avoid(0) = 0.5;                                                   
    r_avoid(1) = 0.5;
    r_avoid(2) = 0.5;

    // Player 1 Quadratic costs

    MatrixXd Q1 = MatrixXd::Zero(Nx, Nx);
    Q1.block(0, 0, nx, nx) = 0.00*MatrixXd::Identity(nx, nx);
    // Q1(2,2) = 1.0;
    // Q1(3,3) = 1.0;

    MatrixXd QN1 = MatrixXd::Zero(Nx, Nx);
    QN1.block(0, 0, nx, nx) = 30.0*MatrixXd::Identity(nx, nx);
    
    MatrixXd R1 = MatrixXd::Zero(Nu, Nu);
    R1.block(0, 0, nu, nu) = 2.0*MatrixXd::Identity(nu, nu);

    // Player 2 Quadratic costs

    MatrixXd Q2 = MatrixXd::Zero(Nx, Nx);
    Q2.block(1*nx, 1*nx, nx, nx) = 0.00*MatrixXd::Identity(nx, nx);
    // Q2(6,6) = 1.0;
    // Q2(7,7) = 1.0;

    MatrixXd QN2 = MatrixXd::Zero(Nx, Nx);
    QN2.block(1*nx, 1*nx, nx, nx) = 30.0*MatrixXd::Identity(nx, nx);
    
    MatrixXd R2 = MatrixXd::Zero(Nu, Nu);
    R2.block(1*nu, 1*nu, nu, nu) = 2.0*MatrixXd::Identity(nu, nu);

    // Player 3 Quadratic costs

    MatrixXd Q3 = MatrixXd::Zero(Nx, Nx);
    Q3.block(2*nx, 2*nx, nx, nx) = 0.00*MatrixXd::Identity(nx, nx);
    // Q3(10,10) = 1.0;
    // Q3(11,11) = 1.0;

    MatrixXd QN3 = MatrixXd::Zero(Nx, Nx);
    QN3.block(2*nx, 2*nx, nx, nx) = 30.0*MatrixXd::Identity(nx, nx);
    
    MatrixXd R3 = MatrixXd::Zero(Nu, Nu);
    R3.block(2*nu, 2*nu, nu, nu) = 2.0*MatrixXd::Identity(nu, nu);


    // Players' initial state
    VectorXd x0(Nx);

    x0 << 
        1.5, 0.0, M_PI_2, 0.0,        // Agent 1
        3.0, 0.0, M_PI_2, 0.0,        // Agent 2
        4.5, 0.0, M_PI_2, 0.0;        // Agent 3

    params.x0 = x0;
    
    // Players' goal state
    VectorXd xgoal(Nx);
    xgoal <<
        4.5, 5.0, M_PI_2, 0.0,          // Agent 1
        3.0, 5.0, M_PI_2, 0.0,          // Agent 2   
        1.5, 5.0, M_PI_2, 0.0;          // Agent 3

    OracleParams oracleparams;
    oracleparams.GoalisChanging = false;
    oracleparams.x0goal = xgoal;

    // construct a Differential drive model 
    Model* ptr_model = new DiffDriveModel4D(dt);                                      // heap allocation

    // construct a concatenated Differential drive model
    NPlayerModel* Npm = new NPlayerModel(ptr_model, n_ag);

    // vector to store pointers to players' costs
    vector<shared_ptr<Cost>> ptr_cost;

    // Player 1 quadratic cost
    ptr_cost.push_back( shared_ptr<Cost> (new DiffDriveCost(oracleparams, Q1, QN1, R1)) );        
    // Player 2 quadratic cost
    ptr_cost.push_back( shared_ptr<Cost> (new DiffDriveCost(oracleparams, Q2, QN2, R2)) );
    // Player 3 quadratic cost
    ptr_cost.push_back( shared_ptr<Cost> (new DiffDriveCost(oracleparams, Q3, QN3, R3)) );
    // ptr_cost.push_back( shared_ptr<Cost> (new CollisionCost2D(params, Q1, QN1, R1, xgoal, r_avoid)) );   
    // ptr_cost.push_back( shared_ptr<Cost> (new CollisionCost2D(params, Q2, QN2, R2, xgoal, r_avoid)) );

    // vector to store pointers to players' constraints
    vector<shared_ptr<GlobalConstraints>> ptr_constr; 
    ptr_constr.push_back( shared_ptr<GlobalConstraints> (new CollisionConstraint2D(params, r_avoid)) );
    ptr_constr.push_back( shared_ptr<GlobalConstraints> (new BoxConstraint(umin, umax, xmin, xmax)) );   

    // construct an augmented lagrangian cost
    AL* al = new AL(params, ptr_constr);

    // construct the main solver
    ALILQGames* alilqgame = new ALILQGames(params, Npm, ptr_cost, al);                    // Declare pointer to the ILQR class.


    // solve the problem
    // auto t0 = high_resolution_clock::now();
    // alilqgame -> solve(params, x0);
    // auto t1 = high_resolution_clock::now();

    // /* Getting number of milliseconds as a double. */
    // duration<double, std::milli> ms_double = t1 - t0;
    // cout << ms_double.count() << "ms\n";



// ################################# Plotting in imgui ##################################################
    // Setup window
	if (!glfwInit())
		return 1;

    // Decide GL+GLSL versions.
    #if __APPLE__
    // GL 3.2 + GLSL 150.
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // Required on Mac
    #else
    // GL 3.0 + GLSL 130.
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+
    // only glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // 3.0+ only
    #endif

	// Create window with graphics context
	GLFWwindow *window = glfwCreateWindow(1280, 720, "Dear ImGui - Example", NULL, NULL);
	if (window == NULL)
		return 1;
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1); // Enable vsync

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))  // tie window context to glad's opengl funcs
		throw("Unable to context to OpenGL");

	int screen_width, screen_height;
	glfwGetFramebufferSize(window, &screen_width, &screen_height);
	glViewport(0, 0, screen_width, screen_height);

    // CustomImGui myimgui;
    TrajImGui myimgui;
	myimgui.Init(window, glsl_version);

    while (!glfwWindowShouldClose(window)) {

		glfwPollEvents();


		glClear(GL_COLOR_BUFFER_BIT);
		myimgui.NewFrame();

		myimgui.Update(params, alilqgame);

        myimgui.Render();
    
        glfwMakeContextCurrent(window);
		glfwSwapBuffers(window);

	}

	myimgui.Shutdown();

    delete alilqgame;

    return 0;
}