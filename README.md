# ALILQGames

## Instructions for Installation:

### Clone the repository:

```git clone https://github.com/RobotJungle/ALILQGames.git ```

### Build:

```cd ALILQGames && mkdir build && cd build && cmake .. ```

### Make:

```make ```

### Test:

```cd examples && ./DiffDrive_imgui_ILQGame```

### Checking for leaks on macos:
```leaks --atExit -- ./3PlayerDiffDriveImguiALILQGame```

### Similarly, you can use Valgrind on Ubuntu 

# TODO:

## Important

- Add Target change (High-level decision making oracle based on proximity)
  - High-level oracle can also be an optimal solver that assigns goals based on splines (constant velocity)
- Add a reset function(s) for the solver
- ~~Fix goal changing~~
- ~~Forward/Initial Rollout should return total cost (saving multiple for loops)~~
  - ~~Total cost should be calculated with parallel forloop~~
    - ~~https://stackoverflow.com/questions/36246300/parallel-loops-in-c~~
- Add documentation
- Place public variabes (if there are) into private members
- Add tests
- Try Intel compiler


## Not Important
- Check for memory allocation of functions 
  - https://stackoverflow.com/questions/59913657/strange-values-of-get-rusage-maxrss-on-macos-and-linux
- Plot trajectories among iterates 
- Try to start with a symmetric strategy
- Check for Equilibria
- Add boxes in GUI to tell if the constraints are violated or not at each time step
- Pass SolverParams to all classes
- Check why the expected change in cost is acting weird
- ~~Make a Receding Horizon function and visualize it~~
- ~~Add plots to variables (i.e. velocity, controls)~~
