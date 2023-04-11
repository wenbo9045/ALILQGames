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

class ReceedingHorizonImGui : public UseImGui {
public:
	virtual void Update(SolverParams& params, Solver* solver) override {

		// Solver params
		int H_all = params.H_all - 1;
		int H = params.H - 1;
		int n_agents = params.n_agents;
		int nx = params.nx;
		int nu = params.nu;

		// render your GUI
		static float xs = 0.0f;
		static float deltaStrategy = 0.0f;								// change in strategy for player i

		static int selected_player_ = 0;		
		static ImGuiComboFlags flags = 0;


		ImGui::Begin("TrajImGui");              						// Create a window called "TrajImGui" and append into it.





		ImGui::SliderFloat("AgentPosx", &xs, 0.0f, H_all*1.0f);            	// Slider for trajectory iterate
		ImGui::SliderFloat("DeltaStrategy", &deltaStrategy, -1.0f, 1.0f);            	// Slider for trajectory iterate


		// Scrolldown menu for selecting player you are intersted in
		if(ImGui::BeginCombo("Player", to_string(selected_player_ + 1).c_str()))
		{
			for (int agent_i = 0; agent_i < n_agents; agent_i++) 
			{
				const bool is_selected = (selected_player_ == agent_i);
				if (ImGui::Selectable(to_string(agent_i + 1).c_str(), is_selected))
					selected_player_ = agent_i;
				if (is_selected) 
					ImGui::SetItemDefaultFocus();
			}
			ImGui::EndCombo();
		}

		

		// if (ImGui::Button("Solve"))
		// {
		// 	solver -> ChangeStrategy(selected_player_, deltaStrategy);
		// 	solver -> solve(params, params.x0);
		// }


		bool clear_color_changed = ImGui::ColorEdit3("clear color", (float*)clear_color); // Edit 3 floats representing a color

		ImGui::SameLine();

		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

		ImGui::End();


		// ############################################### Plotting 2D Traj visualization ###############################################
        ImGui::Begin("2D Visualizer Window");
		
		const float agent_radius = 10.0;

        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        const ImVec2 mouse_position = ImGui::GetMousePos();
        ImGui::BeginChild("User Guide");
		ImGui::TextUnformatted("Green agent is the selected one.");
        ImGui::TextUnformatted("Press \"c\" key to enable navigation.");
        ImGui::TextUnformatted("Press \"z\" key to change zoom.");

		const ImU32 trajectory_color = ImColor(ImVec4(1.0, 1.0, 1.0, 0.5));
  		const float trajectory_thickness = 1.0f;

		int k_step = static_cast<int>(xs);

		if (k_step > H_all)
		{
			k_step = H_all-1;
		}
	    std::vector<ImVec2> points(k_step);
		for (int i = 0; i < n_agents; i++){
			// Plotting robot path
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
        	
			if (i == selected_player_)
			{
				// rgba(78, 233, 39, 0.8)
				draw_list->AddCircle(robotPosition, agent_radius, IM_COL32(78, 233, 40, 255));
			}

			else
			{
				draw_list->AddCircle(robotPosition, agent_radius, IM_COL32(255, 255, 0, 255));
			}

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


		ImGui::End();

		// ############################################ Plotting Costs, states, controls ##################################################


		ImGui::Begin("Trajectory data");

		float accel[k_step];
		float posX[k_step];
		float posY[k_step];
		float vel[k_step];
		// float cost[k_step];


		// for (int i = 0; i < n_agents; i++){
			for (int k=0; k<k_step; k++)
			{
				// X position of player i
				const float xi = solver->getState(k)[selected_player_*nx];	
				// Y position of player i
				const float yi = solver->getState(k)[selected_player_*nx + 1];
				// Velocity position of player i
				const float vi = solver->getState(k)[selected_player_*nx + 3];

				// First control input of player i (in turtlebot case, it is just acceleration)
				const float ui = solver->getControl(k)[selected_player_*nu];

				posX[k] = xi;
				posY[k] = yi;
				vel[k] = vi;
				accel[k] = ui;

				// if (k < H)
				// {
				// 	// cost[k] = solver->pc[i]->StageCost(i, solver->getState(k), solver->getControl(k));
				// 	cost[k] = solver->getStageCost(selected_player_, k);

				// } 
				// else{
				// 	// cost[k] = solver->pc[i]->TerminalCost(i, solver->getState(k));
				// 	cost[k] = solver->getTerminalCost(selected_player_);
				// }
			}
			const string labelX = "Player " + to_string(selected_player_ + 1) + " X";
			const string labelY = "Player " + to_string(selected_player_ + 1) + " Y";
			const string labelV = "Player " + to_string(selected_player_ + 1) + " V";

			const string labelU = "Player " + to_string(selected_player_ + 1) + " U";
			// const string labelCost = "Player " + to_string(selected_player_ + 1) + " Cost";

			// Plot X 
			ImGui::PlotLines(labelX.c_str(), posX, k_step, 0, labelX.c_str(),
					FLT_MAX, FLT_MAX,
					ImVec2(500.0f,100.0f));
			
			// Plot Y
			ImGui::PlotLines(labelY.c_str(), posY, k_step, 0, labelY.c_str(),
					FLT_MAX, FLT_MAX,
					ImVec2(500.0f,100.0f));
			
			// Plot Velocity
			ImGui::PlotLines(labelV.c_str(), vel, k_step, 0, labelV.c_str(),
					FLT_MAX, FLT_MAX,
					ImVec2(500.0f,100.0f));

			// Plot Acceleration
			ImGui::PlotLines(labelU.c_str(), accel, k_step, 0, labelU.c_str(),
					FLT_MAX, FLT_MAX,
					ImVec2(500.0f,100.0f));
			
			// // Plot Cost
			// ImGui::PlotLines(labelCost.c_str(), cost, k_step, 0, labelCost.c_str(),
			// 		FLT_MAX, FLT_MAX,
			// 		ImVec2(500.0f,100.0f));
        	
		// }

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