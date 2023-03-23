#pragma once

#include <iostream>
#include "ALILQGames/ALILQGames.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "glad.h"

/* 
Base class for rendering an imgui in an application.
Handles Initiailization of a ImGui for a GLFW window,
Starting a new frame, rendering, and shutting down/claning up the gui.

The update function is a virtual one is overriten for whatever windows you want.

*/

class UseImGui {
    public:
        void Init(GLFWwindow* window, const char* glsl_version);

        void NewFrame();

        virtual void Update(ALILQGames* solver);

        void Render();

        void Shutdown();

        ImVec2 WindowCenter();

};