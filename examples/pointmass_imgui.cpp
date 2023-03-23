#include "ALILQGames/ALILQGames.h"
#include "ALILQGames/pointmass.h"
#include "ALILQGames/NPlayerModel.h"
#include "ALILQGames/BoxConstraint.h"
#include "ALILQGames/costDiffDrive.h"
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

    Model* pm = new PointMass(dt);                    // heap allocation

    NPlayerModel* Npm = new NPlayerModel(pm, 2);
    vector<std::shared_ptr<Cost>> pc;  
    pc.push_back( shared_ptr<Cost> (new DiffDriveCost(Q1, QN1, R1, xgoal)) );   
    pc.push_back( shared_ptr<Cost> (new DiffDriveCost(Q2, QN2, R2, xgoal)) );


    ALILQGames* alilqgame = new ALILQGames(Npm, pc);                 // Declare pointer to the ILQR class.

    int H = 100;

    alilqgame->init(H);

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