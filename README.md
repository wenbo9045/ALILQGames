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

TODO:
- Make a Receding Horizon function with and visualize it
- Add boxes in GUI to tell if the constraints are violated or not at each time step
- Pass SolverParams to all classes
- Check why the expected change in cost is acting weird
- Add plots to variables (i.e. velocity, controls)
