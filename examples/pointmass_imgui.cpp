#include "ALILQGames/ALILQGames.h"
#include "ALILQGames/pointmass.h"
#include "ALILQGames/NPlayerModel.h"
#include "ALILQGames/BoxConstraint.h"
#include "ALILQGames/costDiffDrive.h"
#include "ALILQGames/CollisionCost2D.h"
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

    int nx = 4;
    int nu = 2;
    int n_ag = 2;
    int Nx = n_ag*nx;
    int Nu = n_ag*nu;

    double dt = 0.1;
    int H = 100;

    SolverParams params;
    params.H = H;
    params.dt = dt;
    params.nx = nx;
    params.nu = nu;
    params.n_agents = n_ag;
    params.Nx = Nx;
    params.Nu = Nu;

    // Player 1 costs

    MatrixXd Q1 = MatrixXd::Zero(Nx, Nx);
    Q1.block(0, 0, nx, nx) = 0.1*MatrixXd::Identity(nx, nx);

    MatrixXd QN1 = MatrixXd::Zero(Nx, Nx);
    QN1.block(0, 0, nx, nx) = 20.0*MatrixXd::Identity(nx, nx);
    
    MatrixXd R1 = MatrixXd::Zero(Nu, Nu);
    R1.block(0, 0, nu, nu) = 2.0*MatrixXd::Identity(nu, nu);

    // Player 2 costs

    MatrixXd Q2 = MatrixXd::Zero(Nx, Nx);
    Q2.block(1*nx, 1*nx, nx, nx) = 0.1*MatrixXd::Identity(nx, nx);

    MatrixXd QN2 = MatrixXd::Zero(Nx, Nx);
    QN2.block(1*nx, 1*nx, nx, nx) = 20.0*MatrixXd::Identity(nx, nx);
    
    MatrixXd R2 = MatrixXd::Zero(Nu, Nu);
    R2.block(1*nu, 1*nu, nu, nu) = 2.0*MatrixXd::Identity(nu, nu);

    // Initialize a 2 player point mass
    VectorXd x0(Nx);

    VectorXd xgoal(Nx);
    VectorXd u(Nu);

    x0 << 5.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0;
    xgoal << 5.0, 10.0, 0.0, 0.0, 10.0, 5.0, 0.0, 0.0;

    u << 0.0, 0.0, 0.0, 0.0;



    Model* pm = new PointMass(dt);                                      // heap allocation

    NPlayerModel* Npm = new NPlayerModel(pm, n_ag);
    vector<std::shared_ptr<Cost>> pc;

    double rho = 500.0;
    VectorXd r_avoid(n_ag);

    r_avoid(0) = 2.0;                                                   
    r_avoid(1) = 2.0;

    // pc.push_back( shared_ptr<Cost> (new DiffDriveCost(Q1, QN1, R1, xgoal)) );   
    // pc.push_back( shared_ptr<Cost> (new DiffDriveCost(Q2, QN2, R2, xgoal)) );
    pc.push_back( shared_ptr<Cost> (new CollisionCost2D(params, Q1, QN1, R1, xgoal, r_avoid)) );   
    pc.push_back( shared_ptr<Cost> (new CollisionCost2D(params, Q2, QN2, R2, xgoal, r_avoid)) );


    ALILQGames* alilqgame = new ALILQGames(params, Npm, pc);                    // Declare pointer to the ILQR class.

    // alilqgame->init(H);

    alilqgame -> solve(x0);

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

		myimgui.Update(alilqgame);

        myimgui.Render();
    
        glfwMakeContextCurrent(window);
		glfwSwapBuffers(window);

	}

	myimgui.Shutdown();

    return 0;
}