## Simulation Setup

There many simulators integrated with ROS 2, the Turtlebot 4 uses Gazebo Fortress. It is usually more convenient to run a simulation of the a robot instead of working on the real one. For example, imagine there is a deadly virus, and you cannot access the real robot.

![gazebo](figures/gazebo.png)

In Gazebo we need a model of the robot that we want to simulate, and a model of the world in which we simulate the robot. A model of the Turtlebot has been provided by Clearpath, and we've uploaded a simple world representing the classroom to the [Gazebo Fuel](https://app.gazebosim.org/fuel/worlds) model repository, which will be automatically downloaded when you star the simulation the first time. This can take some time and might look like the process has crashed, and it very well may have but it also might just be downloading.

Gazebo models consists of a Collada .dae 3D model and an xml description on how to include it into the simulator.

To simulate the Turtlebot, we'll first need to [set up Gazebo Fortress](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html#ignition-gazebo). (note that the new version of Gazebo [used to be called Ignition](https://en.wikipedia.org/wiki/Gazebo_simulator#/media/File:Gazebo_Developement_Timeline.svg) but was later renamed due to trademark conflicts).

#### Option 1: Native install

The most easiest way to work with the Gazebo simulator is by having a native install of some flavor of Ubuntu 22.04 and a GPU with working drivers. In this case you can simply proceed with the install guide in the turtlebot manual.

#### Option 2: WSL and VM install

If using WSL or a VM image, you can install Gazebo as described above, but GPU acceleration may not be available, and you'll have to swich to software rendering by adding the following to your `.bashrc` file:

    export LIBGL_ALWAYS_INDIRECT=0
    export LIBGL_ALWAYS_SOFTWARE=1

Needless to say this option is not very performant and is unlikely to be fast enough for practical use.

#### Option 3: Docker image

If you [set up an X server with Docker](https://medium.com/@pigiuz/hw-accelerated-gui-apps-on-docker-7fd424fe813e), then you can run Gazebo and RViz with hardware acceleration in your host system. 