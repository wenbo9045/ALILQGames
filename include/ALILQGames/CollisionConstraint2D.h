#pragma once
#include "GlobalConstraints.h"

/*
    2D collision constraint between n_agent robots class
    that inherits from the GlobalConstraints class
    Works with any 2D problem Homogenous agent problem, just need to specify 
    the dimension of one of the agents (i.e 4 for a 4D Diffdrive)

    Supposes that the constraints is a n by 1 vector 

    Sorry for indexing like that :(

    Example, for three players: n = n_agents*(n_agents - 1), since it's a 2D collision problem
    c(0): -(x₁-x₂)² - (y₁-y₂)² + r₁² ≤ 0
    c(1): -(x₁-x₃)² - (y₁-y₃)² + r₁² ≤ 0
    c(2): -(x₂-x₁)² - (y₂-y₁)² + r₂² ≤ 0
    c(3): -(x₂-x₃)² - (y₂-y₃)² + r₂² ≤ 0
    c(4): -(x₃-x₁)² - (y₃-y₁)² + r₃² ≤ 0
    c(5): -(x₃-x₂)² - (y₃-y₂)² + r₃² ≤ 0

    Jacobian:
    cᵢ   =  [∂cᵢ∂x₁ ∂cᵢ∂y₁ ∂cᵢ∂θ₁ ∂cᵢ∂v₁ ∂cᵢ∂x₂ ∂cᵢ∂y₂ ∂cᵢ∂θ₂ ∂cᵢ∂v₂ ∂cᵢ∂x₁ ∂cᵢ∂y₂ ∂cᵢ∂θ₃ ∂cᵢ∂v₃]
    c(1) =  [-2(x₁-x₂) -2(y₁-y₂) 0 0 | 2(x₁-x₂)  2(y₁-y₂) 0 0 |     0         0    0 0]
    c(2) =  [-2(x₁-x₃) -2(y₁-y₃) 0 0 |     0         0    0 0 | 2(x₁-x₃)  2(y₁-y₃) 0 0]
    c(3) =  [ 2(x₂-x₁)  2(y₂-y₁) 0 0 |-2(x₂-x₁) -2(y₂-y₁) 0 0 |     0         0    0 0]
    c(4) =  [     0         0    0 0 |-2(x₂-x₃) -2(y₂-y₃) 0 0 | 2(x₂-x₃)  2(y₂-y₃) 0 0]
    c(5) =  [ 2(x₃-x₁)  2(y₃-y₁) 0 0 |     0         0    0 0 |-2(x₃-x₁) -2(y₃-y₁) 0 0]
    c(6) =  [     0         0    0 0 | 2(x₃-x₂)  2(y₃-y₂) 0 0 |-2(x₃-x₂) -2(y₃-y₂) 0 0]
*/

class CollisionConstraint2D : public GlobalConstraints {

    public:
        CollisionConstraint2D(int n_dims, int n_agents, VectorXd& r_avoid) : nx(n_dims), n_ag(n_agents), r(r_avoid) {}

        void StateConstraint(VectorXd& c,const VectorXd& x) override {
            const int n = c.rows();
            assert(n == n_ag*(n_ag-1));                // Make sure that the number of constraints passed is n_agents*(n_agents - 1)
            assert(nx == x.rows()/n_ag);        // Make sure that the dimension of x is correct based on a single agent's 
                                                // dimension should be equal to the number of states divided by number of agents
            
            int v = -1;                         // Keep track of agent index
            int vnot = 0;                         // Keep track of agent all other agent's index
            bool first_v = true;

            for(int i=0; i < n; i++)
            {
                if (!(i%(n_ag-1))) {                    // once we're done with agent v, increment to the new agent
                    first_v = true;
                    v += 1;}    

                if (!(vnot%(n_ag))) {vnot = 0;}         // once we're done with agent not v, increment to the new agent
                
                if(v == vnot && first_v){               // handles v shouldn't be equal to vnot
                    vnot += 1;
                    first_v = false;
                }

                c(i) += r(v)*r(v);                                                          // add radius^2 to constraint
                c(i) -= (x(nx*v) - x(nx*vnot))*(x(nx*v) - x(nx*vnot));                      // add x distance 
                c(i) -= (x(nx*v+1) - x(nx*vnot + 1))*(x(nx*v+1) - x(nx*vnot + 1));          // add y distance

                vnot += 1;
            }
        }

        // Probably there is an easier way to do it, because the Jacobian has a nice pattern
        void StateConstraintJacob(MatrixXd& cx, const VectorXd& x) override {
            const int n = cx.rows();
            const int m = cx.cols();

            int v = -1;                         // Keep track of agent index
            int vnot = 0;                         // Keep track of agent all other agent's index
            bool first_v = true;

            for(int i=0; i < n; i++)
            {
                if (!(i%(n_ag-1))) {                    // once we're done with agent v, increment to the new agent
                    first_v = true;
                    v += 1;}    

                if (!(vnot%(n_ag))) {vnot = 0;}         // once we're done with agent not v, increment to the new agent


                if(v == vnot && first_v){               // handles v shouldn't be equal to vnot
                    vnot += 1;
                    first_v = false;
                }

                cx(i,nx*v) = -2*(x(nx*v) - x(nx*vnot));
                cx(i,nx*v+1) = -2*(x(nx*v+1) - x(nx*vnot+1));
                cx(i,nx*vnot) = 2*(x(nx*v) - x(nx*vnot));
                cx(i,nx*vnot+1) = 2*(x(nx*v+1) - x(nx*vnot+1));

                vnot += 1;
            }
        }

    private:

        VectorXd r;
        int nx;
        int n_ag; 

};
