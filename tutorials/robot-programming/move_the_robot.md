---
title: "Move the Robot"
tags: [robot programming]
keywords:
sidebar: tutorials
permalink: move_the_robot.html
---

In this tutorial, you will create a program that uses the buttons to control the speed of the left and right motors separately. In other words, once you complete all exercises on this page, you can make the robot speed up/down and turn by pushing the buttons! Let's get started by looking at the library function that moves the robot.

## The Move Function

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

## Obstacle Avoidance

```
one.brake(50,50);
```

The `brake()` function also takes two arguments: torqueL and torqueR. These values define the braking power of each motor, which ranges between 0 and 100. Zero corresponds to stopping without braking, whereas 100 corresponds to stopping with the maximum braking torque.

### Exercise 4.3.1

- Again, Start by copying the "empty" program from ["Robot Programming Introduction"](/robot_programming_introduction.html) into your Arduino IDE, and saving it in a sensible place.
- Fill in the program so the robot moves when it's started, and stops when it sees an obstacle.

{{ site.data.alerts.tip }}
We recommend setting both speeds to 25 for this exercise so you can easily observe the obstacle avoidance bahavior.
{{ site.data.alerts.end }}

{% include callout_red_cup.html task="[Exercise 4.3.1]" %}

## Read input from buttons

```
one.readButton();
```

The `readButton()` function indicates which of the pushbuttons PB1, PB2 or PB3 is being pressed. The function returns an int, with the possible values:

- 0: no button is being pressed
- 1: PB1 pressed
- 2: PB2 pressed
- 3: PB3 pressed

{{ site.data.alerts.tip }}
When handling these cases, switch-statement might help simplify your code.
{{ site.data.alerts.end }}

### Exercise 4.3.2

In this exercise, you will build on the previous exercise to make the robot do the following:

- When no button is pressed, keep moving with the same motor speeds until obstacle is detected.
- When PB1 is pressed, increase the speed of the currently selected motor by 1.
- When PB2 is pressed, decrease the speed of the currently selected motor by 1.
- When PB3 is pressed, switch the motor selection.

Further, display "left"/"right" on the first line of the LCD to indicate which motor is selected, and show the speed values on the second line.

{{ site.data.alerts.tip }}
<ul>
<li>This is so far the largest programming exercise on the robot. If you are not sure what this exercise expects, ask the counselors to show you a demo.</li>
<li>We recommend implementing the LCD part first to help with debugging. Comment out the moving code and make sure the buttons change the speed as expected before moving the robot.</li>
</ul>
{{ site.data.alerts.end }}

{% include callout_red_cup.html task="[Exercise 4.3.2]" %}

## Next Step

Proceed to ["Line Follower & Binary"](line_follower_binary.html)