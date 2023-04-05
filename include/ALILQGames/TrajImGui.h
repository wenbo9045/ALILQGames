#include <iostream>
#include "ALILQGames/UseImGui.h"
#include "cost.h"
#include <stdio.h>
#include <math.h>

/* 
TrajImGui class for rendering a 2D trajectory.
inherits from the UseImGui base class for rendering 
an imgui in an application.

*/

using namespace std;

class TrajImGui : public UseImGui {
public:
	virtual void Update(SolverParams& params, Solver* solver) override {

		// Solver stuff
		int H = params.H - 1;
		int n_agents = params.n_agents;
		int nx = params.nx;
		int nu = params.nu;

		// render your GUI
		static float xs = 0.0f;
		static int selected_player_ = 0;		
		static ImGuiComboFlags flags = 0;


		ImGui::Begin("TrajImGui");              // Create a window called "Hello, world!" and append into it.

		ImGui::SliderFloat("AgentPosx", &xs, 0.0f, H*1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f


		// const char* items[] = { "AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "FFFF", "GGGG", "HHHH", "IIII", "JJJJ", "KKKK", "LLLLLLL", "MMMM", "OOOOOOO" };
        // static const char* item_current = items[0];            // Here our selection is a single pointer stored outside the object.
        // ImGui::BeginCombo("combo 1", item_current, flags); // The second parameter is the label previewed before opening the combo.
		
		// for (int n = 0; n < IM_ARRAYSIZE(items); n++)
		// {
		// 	bool is_selected = (item_current == items[n]);
		// 	if (ImGui::Selectable(items[n], is_selected))
		// 		item_current = items[n];
		// 	if (is_selected)
		// 		ImGui::SetItemDefaultFocus();   // Set the initial focus when opening the combo (scrolling + for keyboard navigation support in the upcoming navigation branch)
		// }
		// ImGui::EndCombo();
		// ImGui::BeginCombo("Player", to_string(selected_player_ + 1).c_str());
		// for (int agent_i = 0; agent_i < n_agents; agent_i++) 
		// {
		// 	const bool is_selected = (selected_player_ == agent_i);
		// 	if (ImGui::Selectable(to_string(agent_i + 1).c_str(), is_selected))
		// 		selected_player_ = agent_i;
		// 	if (is_selected) 
		// 		ImGui::SetItemDefaultFocus();
		// }

		// ImGui::EndCombo();


		bool clear_color_changed = ImGui::ColorEdit3("clear color", (float*)clear_color); // Edit 3 floats representing a color

		ImGui::SameLine();

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::End();

        ImGui::Begin("2D Visualizer Window");


		const float agent_radius = 10.0;

        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        const ImVec2 mouse_position = ImGui::GetMousePos();
        ImGui::BeginChild("User Guide");
        ImGui::TextUnformatted("Press \"c\" key to enable navigation.");
        ImGui::TextUnformatted("Press \"z\" key to change zoom.");

		const ImU32 trajectory_color = ImColor(ImVec4(1.0, 1.0, 1.0, 0.5));
  		const float trajectory_thickness = 1.0f;

		int k_step = static_cast<int>(xs);

		if (k_step > H)
		{
			k_step = H-1;
		}
	    std::vector<ImVec2> points(k_step);
		for (int i = 0; i < n_agents; i++){
			for (int k=0; k<k_step; k++)
			{
				points[k] =
				PositionToWindowCoordinates(solver->getState(k)[i*nx], solver->getState(k)[i*nx + 1]);


			}
			constexpr bool kPolylineIsClosed = false;
			draw_list->AddPolyline(points.data(), k_step,
                             trajectory_color, kPolylineIsClosed,
                             trajectory_thickness);

			// Note this is only for point mass
			float x = solver->getState(k_step)[i*nx];
			float y = solver->getState(k_step)[i*nx + 1];
			// float xdot = solver->getState(k_step)[i*nx + 2];
			// float ydot = solver->getState(k_step)[i*nx + 3];
			// float theta = atan2(xdot, ydot);

			// for differential drive robot
			float theta = solver->getState(k_step)[i*nx + 2];


			const ImVec2 robotPosition = PositionToWindowCoordinates(x,y);
        	
			draw_list->AddCircle(robotPosition, agent_radius, IM_COL32(255, 255, 0, 255));

			// for pointmass 
			// draw_list->AddLine(robotPosition, 
			// ImVec2(robotPosition.x + 2.0*agent_radius * (cos(theta - M_PI/2.0)), 
			// robotPosition.y + 2.0*agent_radius * (sin(theta - M_PI/2.0))), 
			// IM_COL32(255, 0, 0, 255));

			// for differential drive
			draw_list->AddLine(robotPosition, 
			ImVec2(robotPosition.x + 2.0*agent_radius * (sin(theta + M_PI/2.0)), 
			robotPosition.y + 2.0*agent_radius * (cos(theta + M_PI/2.0))), 
			IM_COL32(255, 0, 0, 255));
		}



        // const Point2 mouse_point = WindowCoordinatesToPosition(mouse_position);
        ImGui::Text("Mouse is at: (%3.1f, %3.1f)", mouse_position.x,
                    mouse_position.y);

        ImGui::EndChild();


        // const ImVec2 window_center = WindowCenter();

        // const ImVec2 p = ImGui::GetCursorScreenPos();
        // std::cout << p[0] << "\n";
        // float x = mouse_position.x, y =  mouse_position.y;



		ImGui::End();

		ImGui::Begin("Trajectory data");

		float accel[k_step];
		float posX[k_step];
		float cost[k_step];


		for (int i = 0; i < n_agents; i++){
			for (int k=0; k<k_step; k++)
			{
				const float xi = solver->getState(k)[i*nx];
				const float ui = solver->getControl(k)[i*nu];
				posX[k] = xi;
				accel[k] = ui;
				if (k < H)
				{
					// cost[k] = solver->pc[i]->StageCost(i, solver->getState(k), solver->getControl(k));
					cost[k] = solver->getStageCost(i,k);

				} 
				else{
					// cost[k] = solver->pc[i]->TerminalCost(i, solver->getState(k));
					cost[k] = solver->getTerminalCost(i);
				}
			}
			const std::string labelX = "Player " + std::to_string(i + 1) + " posX";

			const std::string labelU = "Player " + std::to_string(i + 1) + " U";
			const std::string labelCost = "Player " + std::to_string(i + 1) + " Cost";

			ImGui::PlotLines(labelX.c_str(), posX, k_step, 0, labelX.c_str(),
					FLT_MAX, FLT_MAX,
					ImVec2(500.0f,100.0f));
			ImGui::PlotLines(labelU.c_str(), accel, k_step, 0, labelU.c_str(),
					FLT_MAX, FLT_MAX,
					ImVec2(500.0f,100.0f));
			
			ImGui::PlotLines(labelCost.c_str(), cost, k_step, 0, labelCost.c_str(),
					FLT_MAX, FLT_MAX,
					ImVec2(500.0f,100.0f));
        	
		}

		ImGui::End();


		if (clear_color_changed) {
			change_clear_color(clear_color[0], clear_color[1], clear_color[2]);
		}

	}

private:
	float clear_color[3] = { .0f, .0f, .0f };
	void change_clear_color(float r, float g, float b) {
		glClearColor(r, g, b, 1.00f);
	}

	const float PtoW = 40.0;

	ImVec2 PositionToWindowCoordinates(float x, float y) {
		ImVec2 coords = WindowCenter();

		coords.x += PtoW*x;
		coords.y -= PtoW*y;
	return coords;
	}


};