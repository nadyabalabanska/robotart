# RobotArt Docker 2018

This is a container for all RobotArt files. Each part is an individual git submodule.

## Setup

Install Docker.

If your computer has Nvidia drivers install nvidia-docker version 1.

Clone this repository. Inside this repository, run `git submodule init` and then `git submodule update --remote --recursive`.

## Run 

`$ ./run.sh` or `./run.sh sim` for the Redhawk simulation.

To attach to the container, run: 

`$ docker exec -i -t roboteam /bin/bash`

### Launching 
`launch_paint.bash` will execute the main painting program. Artwork is read from `config/drawing.dxf`. 

`launch_rviz.bash` launches RViz configured to show the arm's workspace.
