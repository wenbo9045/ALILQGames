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
- Check to parallelize for loops
- Forward/Initial Rollout should return total cost (saving multiple for loops)
- Add boxes in GUI to tell if the constraints are violated or not at each time step
- Pass SolverParams to all classes
- Check why the expected change in cost is acting weird
- <s>Make a Receding Horizon function and visualize it<s>
- <s>Add plots to variables (i.e. velocity, controls)<s>

### Checking for leaks on macos:
```leaks --atExit -- ./3PlayerDiffDriveImguiALILQGame```

### Similarly, you can use Valgrind on Ubuntu 