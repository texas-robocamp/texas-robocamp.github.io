---
title: "Firing Up the Robot"
tags: [robot]
keywords: robot 
sidebar: virtual 
permalink: robot_startup.html
---

{% include note.html content="There is reading material for you to enjoy while you wait for the tutorial to begin. The group section will start at the next synchronization marker. Don't worry about reading all of this material, and please proceed to the yellow synchronization marker on this page when it is time." %}

## Gazebo Simulator

We will be using ROS and the Gazebo simulator to design and test our robot software. Gazebo is a 3D robotics simulator that is used widely in the robotics community. It's also open-source! ROS is also open-source, but we'll talk a little more about ROS when we get to actually writing code on the robot. For now, we're going to walk you through running the simulator to make sure that your system can handle the various features we'll be implementing throughout the course of the camp.

## Installing Our Codebase

### Installing with VirtualBox

*TODO describe VirtualBox*

Steps to install:

1. Go to https://www.virtualbox.org/wiki/Downloads.
2. Download the latest version of VirtualBox for your computer.
3. Open the installer and follow the on-screen instructions to finish installing VirtualBox.
4. Download the VirtualBox appliance file through the link we provide. *TODO not sure whether this will be public*
5. Open the appliance file and click the "Import" button to add the Ubuntu virtual machine to VirtualBox. (When this has finished, you can safely delete the appliance file.)

Once the machine has been added, start it by selecting "Texas RoboCamp 2020" in the left sidebar of VirtualBox and clicking "Start".

{{site.data.alerts.tip}}
If you are ever asked to log in to the virtual machine, use "bevo" for the username and "robocamp" for the password.
{{site.data.alerts.end}}

### Installing on an existing Ubuntu installation

The code for this camp requires Ubuntu 18.04. It is not compatible with older or newer versions, such as Ubuntu 20.04. If you do not have Ubuntu 18.04 already installed, we recommend using the VirtualBox installation method above.

However, if you do have Ubuntu 18.04 installed, you can quickly install the packages for the camp. Just open a terminal and type the following command: *TODO the link is not yet public*

{{site.data.alerts.terminal_commands}}
wget -O [link TODO]/setup.sh | sudo sh
{{site.data.alerts.terminal_commands_end}}

{{site.data.alerts.tip}}
If you would like to upgrade to a newer version of Ubuntu after the camp, you will need to uninstall the camp code. You can do that by typing this command in a terminal: `sudo apt remove ros-melodic-*`
{{site.data.alerts.end}}

## Catkin Workspaces

**TODO** *Depending on how we end up doing the apt package, we might not need to introduce this until we get to robot programming!*

Typical robot development is contained inside a **catkin workspace**. Catkin is what we call a **build system**, which is what turns our human-readable source code into the binary one's and zero's that your computer can interpret. This particular system has rules for how your files need to be organized - we've gone ahead and provided the building blocks for you so that you don't have to get too caught up in the finer details of how this works.

For now, all you need to know is that your catkin workspace is located at `~/catkin_ws`. 

## Launching Gazebo

In order to actually use the robot, you'll need to run what are called **launch files**. These are files which group up smaller executable programs so that we don't have to run all of them individually. The syntax for this is

```
roslaunch <package_name> <file_name>
```

Let's go ahead and boot up the robot for the first time by using our demo launch file. 

{{site.data.alerts.terminal_commands}}
roslaunch robocamp demo.launch
{{site.data.alerts.terminal_commands_end}}


## Moving the Robot

Let's go ahead and get the robot moving!

We've already mentioned launch files, which are launched using the `roslaunch` command. Launch files aren't the only way to run a ROS program however; like we said earlier, they're actually launching smaller executables that are called ROS **nodes**. To launch a single node, do

```
rosrun <package_name> <file_name>
```

{{site.data.alerts.tip}}
The syntax for both <code>roslaunch</code> and <code>rosrun</code> commands are nearly identical! Don't forget that <code>roslaunch</code> is for launch files and <code>rosrun</code> is for individual nodes
{{site.data.alerts.end}}

We're going to use this exercise to verify that your robot operates properly.  

{{site.data.alerts.terminal_commands}}
rosrun robocamp teleop_robot
{{site.data.alerts.terminal_commands_end}}

You should see the following menu appear on your terminal:

```
Move Forward           : W
Turn Left              : A
Turn Right             : D
Reverse                : X
Stop                   : S
Increase Linear Speed  : I
Decrease Linear Speed  : O
Increase Angular Speed : K
Decrease Angular Speed : L
Reset Robot            : R
Quit Teleop            : Q

Linear Speed: 0.10
Angular Speed: 0.50
```

When you have this terminal pane selected, you can use this program to drive the robot around manually. Don't forget about this program, as you will likely find yourself needing to manually drive the robot around to test different functionalities later.

{% include callout_red_cup.html task="[Tutorial 2]" comment="Please flip your cup to red to indicate that you're ready to have your robot tested."%}

{% include note.html content="If anything didn't work, please <em>What are we doing here</em>"%}

## Next Step

Proceed to ["Introduction to C++ Programming"](programming_introduction.html)
