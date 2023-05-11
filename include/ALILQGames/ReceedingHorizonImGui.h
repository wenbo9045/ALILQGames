#include <iostream>
#include "ALILQGames/UseImGui.h"
#include "cost.h"
#include <stdio.h>
#include <math.h>

static inline ImVec2 operator+(const ImVec2& lhs, const ImVec2& rhs) { return ImVec2(lhs.x + rhs.x, lhs.y + rhs.y); }
static inline ImVec2 operator-(const ImVec2& lhs, const ImVec2& rhs) { return ImVec2(lhs.x - rhs.x, lhs.y - rhs.y); }
static inline ImVec2 operator*(const ImVec2& lhs, const ImVec2& rhs) { return ImVec2(lhs.x * rhs.x, lhs.y * rhs.y); }

/*
ReceedingHorizonImGui class for rendering a 2D trajectory.
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

		static bool show_grid = true;
		static bool show_bezier_pts = true;

		static ImVec2 scrolling = ImVec2(0.0f, 0.0f);

		static float xs = 0.0f;
		static float deltaStrategy = 0.0f;								// change in strategy for player i

		static int selected_player_ = 0;
		static ImGuiComboFlags flags = 0;

		static int bezier_pts = 0;
		static bool inited = false;
	    static std::vector<Agent> agents;
		// std::vector<double> pts_x;
		// std::vector<double> pts_y;
		// static ImVector<ImVec2> pts;




		// std::cout << "agents\n " << agents[0].control_pts[0][0] << "\n";




		ImGui::Begin("TrajImGui");              						// Create a window called "TrajImGui" and append into it.


		if (ImGui::Button("Solve"))
		{
			for (int i = 0; i < n_agents; i++)
			{
				solver -> ChangeStrategy(i, deltaStrategy);
				solver->getCostPtr(i)->setCtrlPts(agents);
			}
			solver->recedingHorizon(params, params.x0);
		}

		ImGui::Checkbox("Show grid", &show_grid);
		ImGui::Checkbox("Show Bezier Pts", &show_bezier_pts);


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

		if(ImGui::BeginCombo("Bezier Pts", to_string(bezier_pts + 1).c_str()))
		{
			for (int pts = 0; pts < 4; pts++)
			{
				const bool is_selected = (bezier_pts == pts);
				if (ImGui::Selectable(to_string(pts + 1).c_str(), is_selected))
					bezier_pts = pts;
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




		// ############################################### Plotting 2D Traj visualization ###############################################
        ImGui::Begin("2D Visualizer Window");

		ImGui::Text("Hold right mouse button to scroll (%.2f,%.2f)", scrolling.x, scrolling.y);
		// ImGui::SameLine(ImGui::GetWindowWidth() - 100);
		ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(1, 1));
    	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    	ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(60, 60, 70, 250));
		ImGui::BeginChild("scrolling_region", ImVec2(0, 0), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove);
		ImGui::PopStyleVar();
		ImGui::PushItemWidth(120.0f);

		ImGuiIO& io = ImGui::GetIO();

		const float agent_radius = 0.20*PtoW;

		ImVec2 offset = ImGui::GetCursorScreenPos() + scrolling;

		if (!inited)
    	{
			for (int i=0; i<n_agents; i++)
			{
				const Eigen::VectorXd AllGoal = solver->getCostPtr(i)->getGoal();
				// pts_x.clear();
				// pts_y.clear();
				const float goalX = AllGoal(i*nx);
				const float goalY = AllGoal(i*nx + 1);

				agents.push_back(Agent());
				agents[i].Idxs.push_back(concat(i,0));
				agents[i].control_pts.push_back(ImVec2(goalX, goalY));
			}
			inited = true;

			std::cout << agents[0].Idxs[0];
    	}



        ImDrawList *draw_list = ImGui::GetWindowDrawList();


		// Display grid
		if (show_grid)
		{
			ImU32 GRID_COLOR = IM_COL32(200, 200, 200, 40);
			float GRID_SZ = 64.0f;
			ImVec2 win_pos = ImGui::GetCursorScreenPos();
			ImVec2 canvas_sz = ImGui::GetWindowSize();
			for (float x = fmodf(scrolling.x, GRID_SZ); x < canvas_sz.x; x += GRID_SZ)
				draw_list->AddLine(ImVec2(x, 0.0f) + win_pos, ImVec2(x, canvas_sz.y) + win_pos, GRID_COLOR);
			for (float y = fmodf(scrolling.y, GRID_SZ); y < canvas_sz.y; y += GRID_SZ)
				draw_list->AddLine(ImVec2(0.0f, y) + win_pos, ImVec2(canvas_sz.x, y) + win_pos, GRID_COLOR);
		}


        const ImVec2 mouse_position = ImGui::GetMousePos();
		ImGui::TextUnformatted("Green agent is the selected one.");

		const ImU32 trajectory_color = ImColor(ImVec4(1.0, 1.0, 1.0, 0.5));
  		const float trajectory_thickness = 1.0f;

		int k_step = static_cast<int>(xs);

		if (k_step > H_all)
		{
			k_step = H_all-1;
		}
	    std::vector<ImVec2> points(k_step);

		constexpr bool kPolylineIsClosed = false;

	    std::vector<ImVec2> GoalPoints(H_all);

		for (int i = 0; i < n_agents; i++){
			// Plotting robot path
			for (int k=0; k<k_step; k++)
			{
				points[k] =
				PositionToWindowCoordinates(solver->getMPCState(k)[i*nx], solver->getMPCState(k)[i*nx + 1]) + offset;


			}

			draw_list->AddPolyline(points.data(), k_step,
                             trajectory_color, kPolylineIsClosed,
                             trajectory_thickness);


			// Note this is only for point mass
			const float x = solver->getMPCState(k_step)[i*nx];
			const float y = solver->getMPCState(k_step)[i*nx + 1];

			const Eigen::VectorXd AllGoal = solver->getCostPtr(i)->getGoal();

			const float xgoal = agents[i].control_pts[0][0];//AllGoal(i*nx);
			const float ygoal = agents[i].control_pts[0][1];//AllGoal(1 + i*nx);
			// float xdot = solver->getState(k_step)[i*nx + 2];
			// float ydot = solver->getState(k_step)[i*nx + 3];
			// float theta = atan2(xdot, ydot);

			// for differential drive robot
			const float theta = solver->getMPCState(k_step)[i*nx + 2];


			const ImVec2 robotPosition = PositionToWindowCoordinates(x,y) + offset;

			const ImVec2 goalPosition = PositionToWindowCoordinates(xgoal,ygoal) + offset;

			if (i == selected_player_)
			{
				// rgba(78, 233, 39, 0.8)
				draw_list->AddCircle(robotPosition, agent_radius, IM_COL32(78, 233, 40, 255));
				draw_list->AddCircle(goalPosition, agent_radius, IM_COL32(78, 100, 40, 255));
			}

			else
			{
				draw_list->AddCircle(robotPosition, agent_radius, IM_COL32(255, 255, 0, 255));
				draw_list->AddCircle(goalPosition, agent_radius, IM_COL32(255, 255, 0, 255));
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

		ImU32 agent_i_color = ImColor(IM_COL32(78, 233, 40, 255));
		// Probably the worst way to do it but here it goes ...
		// Need to read more on imgui
		for (int i = 0; i < agents.size(); i++)
		{

			int j = bezier_pts;
			while ((bezier_pts+1) > agents[i].control_pts.size())
			{
				agents[i].control_pts.push_back(ImVec2(0.5, 0.5));

				agents[i].Idxs.push_back(concat(i,j));

				j += 1;

			}


			while (bezier_pts + 1 < agents[i].control_pts.size())
			{
				agents[i].control_pts.pop_back();
				agents[i].Idxs.pop_back();
			}

			agents[i].Size = ImGui::GetItemRectSize()*ImVec2(0.1f, 1.0f);

			for (int k=0; k<H_all; k++)
			{
				solver->getCostPtr(i)->BezierCurveGoal(agents, k, H_all);

				GoalPoints[k] =
				PositionToWindowCoordinates(solver->getCostPtr(i)->getGoal()(i*nx), solver->getCostPtr(i)->getGoal()(i*nx +1)) + offset;
			}

			if (i == selected_player_)
			{
				agent_i_color = ImColor(IM_COL32(78, 233, 40, 255));
			}
			else
			{
				agent_i_color = ImColor(IM_COL32(255, 255, 0, 255));
			}
			draw_list->AddPolyline(GoalPoints.data(), H_all,
                             agent_i_color, kPolylineIsClosed,
                             trajectory_thickness);
		}

		// ImVec2 agent_pos;
		for (int i = 0; i < agents.size(); i++)
		{

			for (int j = 0; j < agents[i].control_pts.size(); j++)
			{

				// std::cout << "Agent: " << i << "\n" << agents[i].Idxs[j] << "\n";
		        ImGui::PushID(agents[i].Idxs[j]);

				ImVec2 node_rect_min = offset + PositionToWindowCoordinates(agents[i].control_pts[j]) - agents[i].Size*ImVec2(0.5f, 0.5f);


        		ImVec2 node_rect_max = node_rect_min + agents[i].Size;

				draw_list->ChannelsSetCurrent(0); // Background
        		ImGui::SetCursorScreenPos(node_rect_min);
        		ImGui::InvisibleButton("node", agents[i].Size);

				bool node_moving_active = ImGui::IsItemActive();

		        if (node_moving_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
					// ImVec2 delta = ImGui::GetMouseDragDelta(0);
					// std::cout << delta.x << "\n";
        		    agents[i].control_pts[j] = agents[i].control_pts[j] + WindowCoordinatesToPosition(io.MouseDelta);


				// ImVec2 GoalPts = offset + PositionToWindowCoordinates(agents[i].control_pts[j]);
				// draw_list->AddCircleFilled(GoalPts, agent_radius, IM_COL32(78, 100, 40, 255));
				if (show_bezier_pts)
				{
					if (i == selected_player_)
					{
						agent_i_color = ImColor(IM_COL32(78, 233, 40, 255));
					}
					else
					{
						agent_i_color = ImColor(IM_COL32(255, 255, 0, 255));
					}
					// std::cout << agents[i].control_pts[j].x << "\n";
					draw_list->AddRectFilled(node_rect_min, node_rect_max, agent_i_color, 20.0f);
					draw_list->AddRect(node_rect_min, node_rect_max, agent_i_color, 20.0f);
				}
        		// ImGui::SetCursorScreenPos(agent_pos);
				// ImGui::InvisibleButton("agent", ImVec2(15.3,15.3));
				// draw_list->AddCircleFilled(agent_pos, agent_radius, IM_COL32(255, 255, 0, 255));


		        ImGui::PopID();

			}
		}


        // const Point2 mouse_point = WindowCoordinatesToPosition(mouse_position);
        // ImGui::Text("Mouse is at: (%3.1f, %3.1f)", mouse_position.x,
        //             mouse_position.y);

		// Scrolling
		if (ImGui::IsWindowHovered() && !ImGui::IsAnyItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0f))
			scrolling = scrolling + io.MouseDelta;

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
				const float xi = solver->getMPCState(k)[selected_player_*nx];
				// Y position of player i
				const float yi = solver->getMPCState(k)[selected_player_*nx + 1];
				// Velocity position of player i
				const float vi = solver->getMPCState(k)[selected_player_*nx + 3];

				// First control input of player i (in turtlebot case, it is just acceleration)
				const float ui = solver->getMPCControl(k)[selected_player_*nu];

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
	const float WtoP = 0.025;		// 1/PtoW

	ImVec2 PositionToWindowCoordinates(float x, float y) {
		ImVec2 coords = WindowCenter();

		coords.x += PtoW*x;
		coords.y -= PtoW*y;
	return coords;
	}

	ImVec2 PositionToWindowCoordinates(const ImVec2 pos) {
		ImVec2 coords = WindowCenter();

		coords.x += PtoW*pos.x;
		coords.y -= PtoW*pos.y;
	return coords;
	}

	ImVec2 WindowCoordinatesToPosition(const ImVec2 window_pos) {
		ImVec2 coords;

		coords.x = WtoP*window_pos.x;
		coords.y = -WtoP*window_pos.y;
	return coords;
	}

	// Concatenate two ints
	int concat(int a, int b)
	{

		// Convert both the integers to string
		string s1 = to_string(a);
		string s2 = to_string(b);

		// Concatenate both strings
		string s = s1 + s2;

		// Convert the concatenated string
		// to integer
		int c = stoi(s);

		// return the formed integer
		return c;
	}

};