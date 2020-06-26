---
title: "First Few Functions & Exploring the Obstacle Detector"
tags: [robot programming]
keywords:
sidebar: virtual 
permalink: first_few_functions.html
---




## Documentation 

We've gone ahead and included a separate page with all the documentation for using the robot [here](docs.html), but we'll be introducing you to the various functions our robot as they become useful. Feel free to come back to this page or the documentation to check up on what these functions do.


{{ site.data.alerts.tip }}

There are a handful of functions that you're simply unlikely to ever use on the robot. Most are documented in the Software Manual, though some are not, and are documented <a href="/less_used_functions.html">here</a>. Some of those functions appear in the above list.

{{ site.data.alerts.end }}

## `LEDs`

```cpp
void ledLeft(boolean state)
```

```cpp
void ledRight(boolean state)
```

These functions turn on and off the left and right LEDs respectively, based on whether it is passed `true` or `false`. **TODO** *Do we want to provide an example for this?*, so look there if you want to use the LED.

## `lcd#`

You already saw the `lcd#` function in the "Hello World" exercise on the robot. One other important feature of this function we should mention is that it can take any number of arguments - for example,

```cpp
bot.lcd1("Bob is ", 10, " years old!");
```

will print out "Bob is 10 years old!"


## Obstacle Sensors

```cpp
double leftObstacleSensor()
```

```cpp
double RightObstacleSensor()
```

These functions will return the distance (in meters) from the left or right obstacle sensor to the nearest obstacle. Each sensor is on the front of the robot, pointing out 45 degrees. The range for these values is [0.0,10.0]

### Exercise 4.2.1

**TODO** *This would be a good place to have them connect the LEDs with the obstacle sensors*

- Start by copying the "empty" program from ["Robot Programming Introduction"](/robot_programming_introduction.html) into your ~~Arduino IDE~~, and saving it in a sensible place.
- Write a short program that will print "Left Sensor Activated" when the left sensor is activated, "Right Sensor Activated" when the right sensor is activated, and "Both Sensors Activated" when both sensors are activated on the LCD on the robot.

{% include callout_red_cup.html task="[Exercises 4.2.1 - 4.2.5]" %}

## Next Step

Proceed to ["Move The Robot"](move_the_robot.html)
