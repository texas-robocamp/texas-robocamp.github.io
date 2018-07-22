---
title: "The Bot'n Roll Library"
tags: [robot programming]
keywords:
sidebar: tutorials
permalink: botnroll_library.html
---

In this tutorial, you will see how to the functionality from the Bot'n Roll Library. We're going to go through all of the functions with brief exercises, to familiarize you with your robot.

## Software Manual
The robot comes with a software manual that you may find helpful in this weeks adventure. You can find it [here](forms/software_manual.pdf)

## Function List

Here's a list of the functions from the Bot'n Roll Library, for your convenience.

![Bot'n Roll Functions](images/software_function_list.png)

You can find all of these functions in `BnrOneA.h`.

## `lcd#`

You already saw the `lcd#` function in the "Hello World" exercise on the robot.

As you can see from the function list, the `lcd#` function takes a variety of different arguments, to allow you to use it in various ways. The library does not come with a method for formatting text, though, as you become more advanced in your C++ programming, you will probably find that you are able to format a string without the assistance of the various methods provided.

## Setup Routines

### `spiConnect`

`void spiConnect(byte sspin);`

This initializes the connection between the Arduino microcontroller and the PIC microcontroller on the robot.

There really is no reason that you are likely to want to change this code during the camp.

### `minBat`

`void minBat(float batmin);`

Let's suppose that you only want the robot to operate if the batteries are charged above a certain threshold. Maybe the robot is racing, or is about to take on a long task. This function will prevent the robot from operating and display an error message if the battery is too low.

### `obstacleEmitters`

`void obstacleEmitters(boolean state)`

This turns on and off the LEDs which are used for obstacle detection.

This could be useful if, for instance, you wanted to try to control the robot using a television remote (an infrared one).

{{ site.data.alerts.tip }}
If `void obstacleEmitters(true)` is not called at the start of your program, then using the obstacle sensors on your robot may not work.
{{ site.data.alerts.end }}

### `setPID`

{{ site.data.alerts.tip }}
We're documenting this for the sake of completeness, but you probably won't tinker with this too much this week.
{{ site.data.alerts.end }}

`void setPID(int kp,int ki,int kd)`

The motors on the robot use what is called PID (proportional-integral-derivative) control to control the speeds at which the motors move. This is implemented on the PIC microcontroller. The ints `kp`, `ki`, and `kd` are called the <b>PID gains</b>. They are the parameters for PID control. The robot has good defaults for these, so there really isn't a reason to tamper with them. Setting different values may cause the robot to move faster, or it may cause it to accelerate and decelerate wildly, making your robot difficult for you to control.

{{ site.data.alerts.tip }}
Interested in learning more? <a href="http://students.iitk.ac.in/roboclub/lectures/PID.pdf">HERE</a> is a brief tutorial.
{{ site.data.alerts.end }}

## `obstacleSensors`

`byte obstacleSensors()`

```
one.move(speedL,speedR);
```

The `move` function takes in two parameters, `speedL` and `speedR`, and moves the left and right motors at the speeds specified. This function takes in values ranging from -100 to 100, where -100 is the maximum speed in reverse, and 100 corresponds to the maximum speed in the forward direction, and 0 stops the motors.

{{ site.data.alerts.tip }}
You can turn the robot by moving the motors at different speeds. 
<ul>
<li>-100, 100 would turn the robot as hard to the left as possible</li>
<li>100, 50, would turn the robot less sharply to the right, navigating the robot in a circle..</li>
</ul>
{{ site.data.alerts.end }}

### Task 8.2

Let's try out using the `delay()` function. Write a program that alternates between showing "Hello World" and "Texas RoboCamp" on the LCD. Have it switch between the two every 5 seconds.

{% include callout_red_cup.html task="8.2" %}

## Moving the Car

The remaining functions from the obstacle_avoidance code that we haven't talked about all involve controlling the movement of the car.

```
one.move(50,50);
```

The `move()` function takes two arguments: speedL and speedR. These values define the speed of each motor, which ranges from -100 to 100. -100 corresponds to the max speed in reverse, and 100 corresponds to the max speed in the forward direction. An input of 0 stops the motor.

```
one.brake(50,50);
```

The `brake()` function also takes two arguments: torqueL and torqueR. These values define the braking power of each motor, which ranges between 0 and 100. Zero corresponds to stopping without braking, whereas 100 corresponds to stopping with the maximum braking torque.

### Task 8.2
Now that you know how to make the car start and stop, let's test out these functions! Write a program that has the car drive forwards for 2 seconds, brakes, and repeats.

Before you write this program, however, you'll need to know about one more function, `void delay(milliseconds)`. This function returns no value, and has the program wait for a specified amount of milliseconds before executing the next line of code. So, if you wanted to wait 5 seconds before running the next line of code, you would do:

```
//Do something
delay(5000);
//Do something else
```


### Additional Functions

Technically, you now have all of the functions that you need in order to get the robot to avoid obstacles. However, there are some additional functions for the BnrOneA class that you may find useful. Remember that to call any of these functions, you have to type `one.functionName()`.

```
void stop()
```

This function cuts energy to both motors on the Bot'n'Roll, causing them to rotate freely until they stop. This is the same as calling `move(0,0)`

```
void led(state)
```

This function can set the LED on or off, depending on the state passed in. Passing in 0 turns the LED off, and passing in 1 turns the LED on.

## Beeping

The default behavior of the obstacle avoidance program is to turn away when the robot senses obstacles on one side, or back up when the robot senses obstacles on both sides. This is bad driving! The robot should tell us when it's about to turn or back up. The objective of your first robot program is signalling. We will let you decide how to signal. We suggest a beep for backing up, and lights for turning.

In order to make your car beep, you'll need two more functions, `tone(pin,frequency,duration)` and `noTone(pin)`.

The `tone(pin,frequency,duration)` function generates a square wave of the specified frequency on the specified pin. The `duration` parameter is optional. If you do not put a value for duration, the wave will continue until `noTone(pin)` is called. The pin you'll be using for sound is pin 9.

### Task 8.3
Now that you know how to produce sound on your car, write a program that makes the car beep for 5 seconds.
