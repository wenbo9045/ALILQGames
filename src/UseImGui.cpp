#include <iostream>
#include "ALILQGames/UseImGui.h"
#include <math.h>

/* 
Base class for rendering an imgui in an application.
Handles Initiailization of a ImGui for a GLFW window,
Starting a new frame, rendering, and shutting down/claning up the gui.

The update function is a virtual one is overriten for whatever windows you want.

*/
 

void UseImGui::Init(GLFWwindow* window, const char* glsl_version) {
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO &io = ImGui::GetIO();
	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(glsl_version);
	ImGui::StyleColorsDark();
}

void UseImGui::NewFrame() {
	// feed inputs to dear imgui, start new frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
}

void render_gui()
{
	ImDrawList *draw_list = ImGui::GetWindowDrawList();
    float samples[100];
    for (int n = 0; n < 100; n++){
        samples[n] = sinf(n * 0.2f + ImGui::GetTime() * 1.5f);
    }
    ImGui::PlotLines("Samples", samples, 100);
}


void UseImGui::Update(ALILQGames* solver) 
{
	ImGui::Begin("Trajectory Visualizer");                          // Create a window called "Conan Logo" and append into it.
	render_gui();  											// draw conan logo if user didn't override update
	ImGui::End();
}

void UseImGui::Render() {
	// Render dear imgui into screen
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void UseImGui::Shutdown() {
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}

ImVec2 UseImGui::WindowCenter() {
  	const ImVec2 window_pos = ImGui::GetWindowPos();
  	const float window_width = ImGui::GetWindowWidth();
  	const float window_height = ImGui::GetWindowHeight();

  	const float center_x = window_pos.x + 0.5 * window_width;
  	const float center_y = window_pos.y + 0.5 * window_height;
  	return ImVec2(center_x, center_y);
}