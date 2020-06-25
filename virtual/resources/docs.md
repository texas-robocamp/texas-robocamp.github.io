---
title: "Documentation"
tags: [resource]
keywords: documentation
sidebar: virtual
permalink: docs.html
---

This page contains a list of all the functions you'll be using when working with the robot this week. As programmers, we call these kinds of pages **Documentation**, because they document what everything means and how it is used. Feel free to refer back to this page to check up on what functions might be useful for your programs as you perform the exercies.

## Robot Documentation
These are the functions available to you for working with the robot:

### void move(float speedL, float speedR)

This function takes in speeds for the left and right wheels to drive the robot. Speeds are limited to [-100, 100]. Passing in zero for both speeds will result in stopping the robot.

### double leftObstacleSensor()

Returns the distance between the left obstacle sensor and the nearest obstacle. Output is from [0.1,10.0]

### double rightObstacleSensor()

Returns the distance between the right obstacle sensor and the nearest obstacle. Output is from [0.1,10.0]

### int readAdc(int sensorNum)

Returns the sensor value currently being read at the `sensorNum` [0-7] position of the line sensor. The value returned is from [0,255].

### int readButton()

Returns the value of which button was most recently pressed.
Possible return values:

---| ---
0 | no button pressed
1 | Button 1 pressed
2 | Button 2 pressed
3 | Button 3 pressed

### void ledLeft(bool on)

Sets the state of the left LED based on the value of `on`. True turns on the LED, false turns it off.

### void ledRight(bool on)

Sets the state of the right LED based on the value of `on`. True turns on the LED, false turns it off.

### void lcd1(const std::string &string)

Prints `string` to the top text on the GUI

### void lcd2(const std::string &string)

Prints `string` to the bottom text on the GUI

## ROS Documentation

### Roslaunch

Roslaunch commands look like this:

```
roslaunch <package_name> <launch_file>
```

Here's one example, which launches the basic world:

```
roslaunch robocamp empty_camp.launch
```
**TODO** *placeholder*

### Rosrun

Rosrun commands look like this:

```
rosrun <package_name> <file_name>
```

One common rosrun you'll probably be using is

```
rosrun robocamp teleop_robot
```
**TOOD** *kinda placeholder too*

### bool ros::Rate::sleep()

This function pauses the program for any remaining time left in the `Rate` cycle. The time for this is defined by the constructor for the instance of `ros::Rate` being used.

As an example:

```cpp
ros::Rate r(10);
r.sleep();
```


