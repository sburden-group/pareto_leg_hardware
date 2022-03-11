
## Folders
**Bringup Scripts** is for provisioning a new ODrive.

**lab\_experiments** is a "sketchbook" space for running lab experiments as scripts.

**leg\_control\_software** is the python package that enables control for the ODrive

**pareto-leg-control** is a Julia package for imposing particular dynamics on the robot in a control loop.

## Setup
**pareto-leg-control** is a separate Julia package with Python bindings installed as a submodule.
If you do not see it present as a directory, you need to download it with `git submodule init` followed by `git submodule update`.i
