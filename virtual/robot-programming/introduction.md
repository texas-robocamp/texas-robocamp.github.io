---
title: "Robot Programming Introduction"
tags: [robot programming]
keywords:
sidebar: virtual 
permalink: robot_programming_introduction.html
---


## Getting Started
Now that you've learned the basics of C++, we can move on to programming on the robot!

## A Brief Introduction to ROS
You will be using the Robot Operating System (ROS) to develop software for your robot. ROS is an open-source meta-operating system for robots which is used around the world for robotics software development and research. 

We've developed software for you to use ROS without needing to explore the complexities of this system, but if you're interested in exploring ROS after the camp, their [website](http://wiki.ros.org) provides many tutorials and has a vast community of developers.

### A Boilerplate ROS Program

Here's a simple, brief program that you can copy as a sort of "empty" program to start your projects with.

```cpp
#include <ros/ros.h>
#include "botnroll/botnroll_wrapper.h"

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "robot");

    BnrOneA bot;

    while(ros::ok()) {
        //Your code goes here!    

        ros::spinOnce();
        
    }

    return 0;
}

```

Let's explore what's going on here.

```cpp
#include <ros/ros.h>
#include "botnroll/botnroll_wrapper.h"
```

Hopefully these first three lines should seem vaguely familar - we're once again **including** other libraries to work with our code. In this case, the libraries we are including are the ROS libraries, and the robot libraries we've written to help you control your robot.

```cpp
int main(int argc, char **argv) {
```

This function should look somewhat familiar as well - it's the `main` function! Unlike previous `main` functions you've seen so far, this one has two parameters, argc and argv. These can be used to pass in arguments from your terminal, but since we won't ever be using these arguments, you don't need to worry about them.

**TODO** *Should we explain argc and argv? They're never gonna pass in args this way so I feel like it's just extra info that would be confusing..*

```cpp
ros::init(argc, argv, "robot");
```

This function starts up our ROS process. ROS calls these **nodes**, and allows for arguments to be passed in via the command line with `argc` and `argv`. The last argument defines the name of the node - in this case, the name is "robot".

```cpp
BnrOneA bot;
```

Here, we're creating an instance of the `BnrOneA` object, and calling it `bot`. 

This object is a **class** just like the `BankAccount` class you developed earlier in the programming portion of this camp, and much like the `BankAccount` class, it contains a number of variables and functions, some of which are private and some of which are public. We'll explore these in more detail later on.

```cpp
while(ros::ok()) {
```

This should seem familiar as well - it's another **while loop**. Whereas earlier you saw the condition be a logical expression involving numbers, this loop will execute for as long as the function `ros::ok()` returns `true`. In general, this will always be true until you close your program.

```cpp
ros::spinOnce();
```

This function lets our node update its information. For our purposes, it lets the BnrOneA object update its sensor data so that we can be aware of what's going on in the simulation. Without this function, our node would have no idea what is going on in the simulator, so it is vital that you leave this line here. We recommend writing all of your code before this line.

```cpp
return 0;
```

Here we see our usual return statement from main, which simply returns zero. This line will only be executed when `ros::ok()` is `false`, so don't write any code outside of the `while` loop because it will not be executed until after the ROS node is finished running.


## Hello World - On the Robot's LCD


Now that we're developing software on the actual robot, let's go ahead and write a new "Hello World" program. However, instead of just printing "Hello World" on the computer screen, we're going to print it on the robot's LCD screen! 

You might notice that there isn't actually an LCD screen on our robot. In the world of simulation, we developers have to make design choices that may differ from what a robot would look like in real life. 
This is a great example of one of these design choices. In simulation, printing out information on an actual screen on the robot would be difficult to read, since we're viewing the robot from above in Gazebo instead of being able to physically lean down and view what is being printed on the LCD. Because of this, we've built an additional UI where the LCD information will appear. 

We will continue to refer to the screen as an LCD and name our function `lcd` so that if there were actual development on the physical robot, we would only have to change the behavior of our robot object instead of the code printing to the LCD. 

### The `lcd` Function

```
void lcd#(const std::string &line)
```

This function is actually one of two `lcd#` functions. The robot has two lines of LCD output. Calling `lcd1` or `lcd2` specifies which line you would like for your text to appear on. There is no concept of a "carriage return," "\n," or `endl` that is useful here.

The parameter here `const std::string &line` might look a little confusing; this is because we are passing the argument by **reference**. We won't go too into detail about what passing by reference is - all you need to know is that strings are treated differently in C++.

Here is an example of how you could use these functions:

```
one.lcd1("How are you today?");
one.lcd2("I'm doing great thanks");
```

### Exercise 4.1.1

- Write your first Hello World program on the robot!

- Create a new Arduino program, go to "File -> New".

- Copy the short program above into your package to set up your program, and save your program inside your workspace (along with the other exercises you've done is a good idea). Just like the other exercises, give your program and the folder you work in the same name.

- Make this program say "Hello World!" on the first line in the LCD, and "Texas RoboCamp!" on the second line.


## Sleep

One function that you will find helpful this week is `sleep`. This is a function of the `ros::Rate` class, and allows for your program to wait for a short period of time. Although the `sleep` function doesn't have any parameters - this is because it uses the `frequency` member variable in the `ros::Rate` class .

```cpp
ros::Rate::Rate(double frequency)
```

This is the constructor for an instance of the `ros::Rate` class. Like we said earlier, it has a single parameter, `double frequency`. As a reminder, the unit used for frequencies is Hz, which is 1/seconds.This means that the larger the frequency is, the shorter the sleep time will be.

{{ site.data.alerts.tip }}
Did you notice the <em>namespace</em> <code>ros::</code> for this constructor? You'll need to make sure you type the namespace when you create an instance of a Rate object, just like we do with <code>ros::init</code>
{{ site.data.alerts.end }}

```cpp
bool ros::Rate::sleep()
```

This function pauses the program for any remaining time left in the cycle. 

Basically, you want to use `sleep` whenever you want the robot to enter a state for a specified period of time, or when you want to slow down and break down a program so you can see what the robot is doing. Maybe the robot should drive forward for 1 second and then stop, or maybe the robot should simply take a pause after doing something so you can see where one thing stops and another begins. `sleep` allows you do do this.

{{ site.data.alerts.tip }}
sleep time = 1/frequency seconds 
{{ site.data.alerts.end }}


### Exercise 4.1.2

Modify your program to:

- Say: "Hello World!" on the first line in the LCD, and "Texas RoboCamp!" on the second line.
- Wait 2 seconds
- Say: "Hook 'em" on the first line in the LCD, and "Horns!" on the second line.
- Then repeat this in a loop, after waiting 2 seconds with "Hook 'em Horns!" displayed.

{{ site.data.alerts.tip }}
You can modify this exercise to use whatever cheer you like. The point is to learn how the code works.
{{ site.data.alerts.end }}

{% include callout_red_cup.html task="[Exercise 4.1.1, Exercise 4.1.2]" %}

## Next Step

Proceed to ["First Few Functions & Exploring the Obstacle Detector"](/first_few_functions.html)
