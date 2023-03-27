# ALILQGames

## Instructions for Installation:

### Clone the repository:

```git clone https://github.com/RobotJungle/ALILQGames.git ```

### Build:

```cd ALILQGames && mkdir build && cd build && cmake .. ```

### Make:

```make ```


### Test:

```cd examples && ./pointmass_imgui```

TODO:
- Pass SolverParams to all classes
- Test the augmented lagrangian solver, by passing it as an optional method
- Check why the expected change in cost is acting weird
- Add plots to variables (i.e. velocity, controls)
