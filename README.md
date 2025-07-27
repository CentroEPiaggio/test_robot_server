# ROS2 Bifrank workspace
Docker workspaces to use and deploy the bifrank system (bimanual franka manipulator), developing inside a container.

## Overview
This environment integrates ROS2 Humble with the Franka ecosystem and the Bifrank framework.
It also enables the use of GUI interfaces (e.g. Gazebo and RViz).

If you find this useful, you can cite [the original repository](https://github.com/GiorgioSimonini/docker_ws) and/or [me](https://github.com/GiorgioSimonini) in your work.


## Preliminaries
Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine).
You can follow the installation method through `apt`.
Note that it makes you verify the installation by running `sudo docker run hello-world`.
It is better to avoid running this command with `sudo` and instead follow the post installation steps first and then run the command without `sudo`.

Follow with the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for Linux.
This will allow you to run Docker without `sudo`.

You also would probably need [Visual Studio Code](https://code.visualstudio.com/) and the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension

## Usage
Clone the repository wherever you want on the system with recursive behavior(better to clone with linux for line endings, not tested with WSL but should work)
```bash
git clone --recursive https://github.com/GiorgioSimonini/bifrank_ws
```

The docker base image and the ROS version can be changed by modifying the `BASE_IMAGE` and the `BASE_TAG` in the `.devcontainer/Dockerfile`.

The `build.bash` and the `run.bash` files are used to build and run the image from terminal.
`attach.bash` can be used to connect from the terminal to the same container created with DevContaiers extension.

The first time the container is created on the computer, you probably need to run `./first_compile.bash` that initialize and build the stuffs from Franka. Then, you can compile your ROS2 packages with 

```bash
colcon build --symlink-install
```

There are two different ways to use this framework:
- Using DevContainers that creates a persistent container with name `ros2_bifrank_dev` in which all the installations persists since a new build. This scenario is usefull while you are developing. However, the changes will be lost when you recreate the container, so pay attention.
- Using the terminal, each container created in this way is erased upon exit and have no persistency. This is better for running demos or execute commands.

### Use with vscode
Just open the main folder with vscode and click on `Reopen in Container` when asked.
Or, if you ave already a running container, click on `Attach to Running Container..` int the DevContainers extension.

### Use with terminal
Build the docker image:
```shell
./build.bash
```

Run the container:
```shell
./run.bash
```

## Troubleshooting

## Author

[Giorgio Simonini](https://github.com/GiorgioSimonini)


## Acknowledgments

- [Davide De Benedittis](https://github.com/ddebenedittis)
- [Baptiste Busch](https://medium.com/@baptiste.busch/creating-a-ros-or-ros2-workspace-in-docker-part-1-912529c87708): creation of a ROS workspace in Docker
- [Docker best practises](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)

