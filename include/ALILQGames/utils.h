#pragma once
#include <iostream>
#include <vector>
#include "imgui.h"

// struct Vec2
// {
//     float x,y;
//     constexpr Vec2()                      : x(0.0f), y(0.0f) { }
//     constexpr Vec2(float _x, float _y)    : x(_x), y(_y) { }
// };

struct Agent
{
    std::vector<int> Idxs;

    std::vector<ImVec2> control_pts;
    ImVec2 Size;
    Agent() {}
};